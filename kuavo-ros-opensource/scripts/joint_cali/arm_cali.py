#!/opt/miniconda3/envs/joint_cali/bin/python
# -*- coding: utf-8 -*-

from arm_kinematics import HeadKinematics, ArmKinematics, get_package_path, quat_to_rot, rot_to_quat

import pinocchio as pin
import numpy as np
import os
import nlopt
import time
import argparse
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float64MultiArray
from apriltag_ros.msg import AprilTagDetectionArray

from kuavo_msgs.msg import sensorsData

from identifiability_analyzer import create_objective_function, identifiability_analyzer

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTextEdit, QComboBox, QFileDialog, QMessageBox, QLineEdit
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt5.QtCore import QObject, pyqtSignal, Qt, QTimer, QProcess
import sys
import cyipopt
import json
from datetime import datetime

from apriltag_cube import AprilTag3DRos
import tf.transformations as tf_trans
import tf2_ros
from geometry_msgs.msg import TransformStamped
import yaml

from kuavo_msgs.srv import changeArmCtrlModeRequest, changeArmCtrlModeResponse, changeArmCtrlMode
from target_tracker import TargetTracker

from ik_cmd import get_eef_pose_msg
from motion_capture_ik.msg import twoArmHandPoseCmd

def change_arm_ctrl_mode(control_mode):
    rospy.wait_for_service('/humanoid_change_arm_ctrl_mode')
    try:
        change_mode = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
        req = changeArmCtrlModeRequest()
        req.control_mode = control_mode
        res = change_mode(req)
        if res.result:
            rospy.loginfo("手臂控制模式已更改为 %d", control_mode)
        else:
            rospy.logerr("无法将手臂控制模式更改为 %d", control_mode)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

def change_kinematic_mpc_ctrl_mode(control_mode):
    rospy.wait_for_service('/mobile_manipulator_mpc_control')
    try:
        change_mode = rospy.ServiceProxy('/mobile_manipulator_mpc_control', changeArmCtrlMode)
        req = changeArmCtrlModeRequest()
        req.control_mode = control_mode
        res = change_mode(req)
        if res.result:
            rospy.loginfo("运动学mpc控制模式已更改为 %d", control_mode)
        else:
            rospy.logerr("无法将运动学mpc控制模式更改为 %d", control_mode)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

def change_wbc_trajectory_ctrl_mode(control_mode):
    rospy.wait_for_service('/enable_wbc_arm_trajectory_control')
    try:
        change_mode = rospy.ServiceProxy('/enable_wbc_arm_trajectory_control', changeArmCtrlMode)
        req = changeArmCtrlModeRequest()
        req.control_mode = control_mode
        res = change_mode(req)
        if res.result:
            rospy.loginfo("wbc轨迹控制模式已更改为 %d", control_mode)
        else:
            rospy.logerr("无法将wbc轨迹控制模式更改为 %d", control_mode)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)


class VisualizationSignals(QObject):
    update_signal = pyqtSignal()

class ArmCalibrator:
    def __init__(self, side='l', T_et=None, real_mode=False, plot_size=100):
        self.fk = None
        self.fk_arm =None
        self.head_fk = None
        self.side = side
        self.T_et = T_et
        self.real_mode = real_mode
        self.plot_size = plot_size

        self.is_hand_move_enabled = False

        self.data_dict_list = []
        self.result = None
        self.q = None
        
        self.pos_dis_threshold = 0.01 # m
        self.ori_dis_threshold = 0.01 # rad

        
        self.q_updated = False
        self.tag_pos_updated = False
        self.odom_updated = False

        # kinematics
        self.init_kinematics(side)

        # 
        self.condition_numbers = []

        # ros
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.cube_detector = self.set_up_tag_detector()
        
        # 初始化头部追踪器
        asset_path = get_package_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '40')
        urdf_path = os.path.join(asset_path, f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf")
        self.target_tracker = TargetTracker(urdf_path, np.deg2rad(-35))
        self.target_tracker.back_to_zero()

        if self.real_mode:
            print("real mode")
            self.q_sub = rospy.Subscriber("/sensor_data_motor/motor_pos", Float64MultiArray, self.real_sensor_data_callback)
        else:
            print("gazebo mode")
            self.q_sub = rospy.Subscriber("/share_memory/sensor_data_raw", sensorsData, self.gazebo_sensor_data_callback)
        # self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_pos_callback)
        # test
        self.test_data_list_dict = {
            'p_bt_meas': [],
            'quat_bt_meas': [],
            'p_bt_fk': [],
            'quat_bt_fk': []
        }
        
        # 确保在ROS节点初始化之前初始化Qt应用
        self.app = QApplication.instance()
        if self.app is None:
            self.app = QApplication(sys.argv)
        
        # 创建信号对象
        self.signals = VisualizationSignals()
        self.signals.update_signal.connect(self.update_visualization)
        
        self.init_visualization()
        
        # 添加头部追踪控制按钮
        self.setup_head_tracking_controls()
        
    def init_kinematics(self, side):
        # kinematics
        asset_path = get_package_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '40')
        urdf_path = os.path.join(asset_path, f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf")
        print(f"urdf_path: {urdf_path}")
        self.fk_arm = ArmKinematics(urdf_path, self.T_et)
        if side == "l":
            self.fk = self.fk_arm.FK_l
        else:
            self.fk = self.fk_arm.FK_r
        # head kinematics
        self.head_fk = HeadKinematics(urdf_path).FK
        
    def init_visualization(self):
        """Initialize visualization window and plots"""
        self.window = QMainWindow()
        self.window.setWindowTitle('Arm Calibration - ' + ('Left' if self.side == 'l' else 'Right'))
        
        # Create central widget and layout
        central_widget = QWidget()
        self.window.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left control panel
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        main_layout.addWidget(control_panel)
        
        # 添加左右手选择
        arm_select_group = QWidget()
        arm_select_layout = QHBoxLayout(arm_select_group)
        arm_select_layout.addWidget(QLabel('Select arm:'))
        self.arm_combo = QComboBox()
        self.arm_combo.addItems(['Left', 'Right'])
        self.arm_combo.currentIndexChanged.connect(self.change_arm)
        arm_select_layout.addWidget(self.arm_combo)
        control_layout.addWidget(arm_select_group)
        
        # Add optimizer selection
        optimizer_group = QWidget()
        optimizer_layout = QHBoxLayout(optimizer_group)
        optimizer_layout.addWidget(QLabel('Optimizer:'))
        self.optimizer_combo = QComboBox()
        self.optimizer_combo.addItems([
            'ipopt',        # IPOPT algorithm
            'nlopt_mma',    # NLopt LD_MMA algorithm
            'nlopt_lbfgs',  # NLopt LD_LBFGS algorithm
            'nlopt_slsqp',  # NLopt LD_SLSQP algorithm
            'nlopt_direct', # NLopt GN_DIRECT algorithm
            'nlopt_crs2',   # NLopt GN_CRS2_LM algorithm
            'nlopt_var2',   # NLopt LD_VAR2 algorithm
        ])
        optimizer_layout.addWidget(self.optimizer_combo)
        control_layout.addWidget(optimizer_group)
        
        # Add data import/export buttons
        io_layout = QHBoxLayout()
        
        self.export_btn = QPushButton('Export Data')
        self.export_btn.clicked.connect(self.export_data)
        io_layout.addWidget(self.export_btn)
        
        self.import_btn = QPushButton('Import Data')
        self.import_btn.clicked.connect(self.import_data)
        io_layout.addWidget(self.import_btn)
        
        # 添加清除数据按钮
        self.clear_btn = QPushButton('Clear Data')
        self.clear_btn.clicked.connect(self.clear_data)
        io_layout.addWidget(self.clear_btn)
        
        # 在IO布局后添加操作按钮布局
        operation_layout = QHBoxLayout()
        
        # 添加示教按钮
        self.hand_move_btn = QPushButton('Hand Move')
        self.hand_move_btn.setToolTip('execute prerecorded hand move')
        self.hand_move_btn.clicked.connect(self.execute_hand_move)
        operation_layout.addWidget(self.hand_move_btn)
        
        # 添加启用移动按钮
        self.enable_move_btn = QPushButton('Enable Move')
        self.enable_move_btn.setToolTip('enable robot arm move')
        self.enable_move_btn.clicked.connect(self.enable_move)
        operation_layout.addWidget(self.enable_move_btn)
        
        # 添加禁用移动按钮
        self.disable_move_btn = QPushButton('Disable Move')
        self.disable_move_btn.setToolTip('disable robot arm move')
        self.disable_move_btn.clicked.connect(self.disable_move)
        operation_layout.addWidget(self.disable_move_btn)
        
        # 将IO布局添加到控制面板
        control_layout.addLayout(io_layout)
        
        # 将操作按钮布局添加到控制面板的主布局中
        control_layout.addLayout(operation_layout)  # 确保这行存在
        # 在控制面板添加过滤按钮
        self.filter_btn = QPushButton('Filter Noise')
        self.filter_btn.setToolTip('Remove noisy data using statistical method')
        self.filter_btn.clicked.connect(self.remove_noisy_data)
        control_layout.addWidget(self.filter_btn)
        
        # Add calibration button
        self.calibrate_btn = QPushButton('Execute Calibration')
        self.calibrate_btn.setToolTip('run optimization algorithm to calibrate arm')
        self.calibrate_btn.clicked.connect(self.calibrate)
        control_layout.addWidget(self.calibrate_btn)
        
        # Add evaluation button
        self.eval_btn = QPushButton('Evaluate Calibration')
        self.eval_btn.setToolTip('evaluate calibration result(mean and standard deviation)')
        self.eval_btn.clicked.connect(self.evaluate_calibration)
        control_layout.addWidget(self.eval_btn)
        
        # Add data count display
        self.data_count_label = QLabel('Collected Data: 0')
        control_layout.addWidget(self.data_count_label)
        
        # Add calibration result display area
        result_group = QWidget()
        result_layout = QVBoxLayout(result_group)
        result_layout.addWidget(QLabel('Calibration Result:'))
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMinimumWidth(300)
        result_layout.addWidget(self.result_text)
        control_layout.addWidget(result_group)
        
        # Right plot panel
        plot_panel = QWidget()
        plot_layout = QVBoxLayout(plot_panel)
        main_layout.addWidget(plot_panel)
        
        # Add error plot
        self.fig_error = Figure(figsize=(12, 2))
        self.ax_error = self.fig_error.add_subplot(111)
        self.canvas_error = FigureCanvas(self.fig_error)
        plot_layout.addWidget(self.canvas_error)
        
        # Add position comparison plot
        self.fig_pos = Figure(figsize=(4, 8))
        self.ax_pos_x = self.fig_pos.add_subplot(311)
        self.ax_pos_y = self.fig_pos.add_subplot(312)
        self.ax_pos_z = self.fig_pos.add_subplot(313)
        self.canvas_pos = FigureCanvas(self.fig_pos)
        main_layout.addWidget(self.canvas_pos)
        
        # Condition number plot
        self.fig_cond = Figure(figsize=(12, 4))
        self.ax_cond_all = self.fig_cond.add_subplot(121)
        self.ax_cond_recent = self.fig_cond.add_subplot(122)
        self.canvas_cond = FigureCanvas(self.fig_cond)
        plot_layout.addWidget(self.canvas_cond)
        
        # Create joint distribution and sensitivity plot horizontal layout
        plot_bottom = QHBoxLayout()
        plot_layout.addLayout(plot_bottom)
        
        # Joint distribution plot
        self.fig_joint = Figure(figsize=(8, 4))
        self.ax_joint = self.fig_joint.add_subplot(111)
        self.canvas_joint = FigureCanvas(self.fig_joint)
        plot_bottom.addWidget(self.canvas_joint)
        
        # Sensitivity plot
        self.fig_sens = Figure(figsize=(2, 4))
        self.ax_sens = self.fig_sens.add_subplot(111)
        self.canvas_sens = FigureCanvas(self.fig_sens)
        plot_bottom.addWidget(self.canvas_sens)
        
        # Initialize position comparison lines
        self.fk_lines = []
        self.measured_lines = []
        for ax in [self.ax_pos_x, self.ax_pos_y, self.ax_pos_z]:
            fk_line, = ax.plot([], [], 'r-', label='FK')
            measured_line, = ax.plot([], [], 'b--', label='Measured')
            self.fk_lines.append(fk_line)
            self.measured_lines.append(measured_line)
            ax.set_title(f'{["X", "Y", "Z"][len(self.fk_lines)-1]} Position Comparison')
            ax.grid(True)
            ax.legend()
        
        self.fig_pos.tight_layout()
        
        # 在plot_layout中添加误差对比图表区域（在plot_bottom布局之后）
        # 误差对比图表
        self.fig_eval = Figure(figsize=(12, 4))
        self.ax_eval_pos = self.fig_eval.add_subplot(121)
        self.ax_eval_rot = self.fig_eval.add_subplot(122)
        self.canvas_eval = FigureCanvas(self.fig_eval)
        plot_layout.addWidget(self.canvas_eval)
        
        # 调整布局顺序
        plot_layout.addWidget(self.canvas_cond)  # 原条件数图表
        plot_layout.addWidget(self.canvas_eval)  # 新增评估图表
        plot_layout.addLayout(plot_bottom)       # 原关节分布和敏感度图表
        
        # Initialize plots
        self.update_visualization()
        
        # Set window size and position
        self.window.setGeometry(100, 100, 1600, 800)
        self.window.show()

    def change_arm(self, index):
        """切换左右手"""
        new_side = 'l' if index == 0 else 'r'
        if new_side != self.side:
            # 更新侧面
            self.side = new_side
            # 重新初始化运动学
            self.init_kinematics(self.side)
            # 重新设置标签检测器
            self.cube_detector = self.set_up_tag_detector()
            # 清除数据
            self.clear_data()
            # 更新窗口标题
            self.window.setWindowTitle(f'Arm Calibration - {"Left" if self.side == "l" else "Right"}')
            # 更新结果文本
            self.result_text.setText(f"switched to {'Left' if self.side == 'l' else 'Right'}")

    def clear_data(self):
        """清除所有收集的数据"""
        self.data_dict_list = []
        self.condition_numbers = []
        self.result = None
        self.test_data_list_dict = {
            'p_bt_meas': [],
            'quat_bt_meas': [],
            'p_bt_fk': [],
            'quat_bt_fk': []
        }
        # 更新数据计数
        self.data_count_label.setText('Collected Data: 0')
        # 更新结果文本
        self.result_text.setText("data cleared")
        # 更新可视化
        self.update_visualization()

    def calibrate(self):
        """Execute calibration and display result"""
        if len(self.data_dict_list) < 2:
            self.result_text.setText("Need at least 2 collected data to calibrate")
            return
            
        try:
            delta_result = self.cali_arm(len(self.data_dict_list))
            print(f"delta_result: {delta_result}")
            self.result = delta_result
            
            # Display result
            result_text = "Calibration Result:\n\n"
            result_text += "Joint Bias:\n"
            for i, bias in enumerate(delta_result):
                result_text += f"Joint {i+1}: {np.rad2deg(bias):.4f}°\n"
            
            self.result_text.setText(result_text)
            
            # 添加弹窗确认
            reply = QMessageBox.question(self.window, 'Message', 
                                       'Calibration finished, apply new zero point?',
                                       QMessageBox.Yes | QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                self.save_calibration_result(delta_result)
                self.result_text.setText(result_text + "\n\nZero point saved!")
            
        except Exception as e:
            self.result_text.setText(f"Calibration Error: {str(e)}")

    def save_calibration_result(self, delta_result):
        assert len(delta_result) == 7, "手的关节偏置量必须是7x1数组"
        """Save calibration result to file"""
        # negtive
        delta_result = (-1.0) * delta_result
        try:
            home_dir = os.path.expanduser('~')
            arm_zero_file_path = os.path.join(home_dir, '.config/lejuconfig/arms_zero.yaml')
            print(f"arm_zero_file_path: {arm_zero_file_path}")
            # update ruiwo and elmo bias
            extend_delta_result_ruiwo = np.zeros(12)
            extend_delta_result_elmo = np.zeros(2)
            if self.side == 'l':
                extend_delta_result_ruiwo[:6] = delta_result[1:]
                extend_delta_result_elmo[0] = delta_result[0]
            else:
                extend_delta_result_ruiwo[6:] = delta_result[1:]
                extend_delta_result_elmo[1] = delta_result[0]
            self.modify_arm_zero_yaml(arm_zero_file_path, extend_delta_result_ruiwo)
            self.modify_arm_elmo_offset_csv(extend_delta_result_elmo)
            
        except Exception as e:
            error_msg = f"保存失败: {str(e)}"
            print(error_msg)
            self.result_text.setText(self.result_text.toPlainText() + "\n\n" + error_msg)

    def update_visualization(self):
        """Update all visualization plots"""
        try:
            self.update_error_plot()
            self.update_condition_plot()
            self.update_joint_distribution()
            self.update_sensitivity_plot()
            self.update_position_comparison()
            self.app.processEvents()
        except Exception as e:
            print(f"Error in update_visualization: {e}")

    def update_error_plot(self):
        """Update position error plot"""
        try:
            self.ax_error.clear()
            
            if len(self.data_dict_list) > 0:
                # Calculate FK and measured positions
                fk_positions = []
                measured_positions = []
                
                for data in self.data_dict_list:
                    # Calculate FK position
                    q_arm = data['q'][12:19] if self.side == 'l' else data['q'][19:26]
                    fk_pos, fk_ori, _ = self.fk(q_arm)
                    self.publish_tf(fk_pos, rot_to_quat(fk_ori), "base_link", "cube_l_fk")
                    # print(f"fk_ori: {fk_ori}")
                    fk_positions.append(fk_pos)
                    
                    # Calculate measured position from tag
                    q_head = data['q'][-2:]
                    p_bi, R_bi = self.head_fk(q_head)
                    p_bt = p_bi + R_bi @ data['tag_pos']
                    R_bt = R_bi @ quat_to_rot(data['tag_ori'])
                    self.publish_tf(p_bt, rot_to_quat(R_bt), "base_link", "cube_l_tag")
                    # R_err = fk_ori @ R_bt.T
                    # print(f"R_bt: {R_bt}")
                    # print(f"quat_bt: {rot_to_quat(R_bt)}")
                    # print(f"quat_fk: {rot_to_quat(fk_ori)}")
                    # q_err = rot_to_quat(R_err)
                    # print(f"q_err: {q_err}")
                    
                    measured_positions.append(p_bt)
                
                fk_positions = np.array(fk_positions)
                measured_positions = np.array(measured_positions)
                
                # Calculate absolute errors for each axis
                errors = np.abs(fk_positions - measured_positions)
                
                # Calculate mean errors
                mean_errors = np.mean(errors, axis=0)
                
                # Plot errors
                x = range(len(errors))
                self.ax_error.plot(x, errors[:, 0], 'r-', label=f'X Error (mean: {mean_errors[0]:.3f}m)')
                self.ax_error.plot(x, errors[:, 1], 'g-', label=f'Y Error (mean: {mean_errors[1]:.3f}m)')
                self.ax_error.plot(x, errors[:, 2], 'b-', label=f'Z Error (mean: {mean_errors[2]:.3f}m)')
                
                self.ax_error.set_title('Position Errors')
                self.ax_error.set_xlabel('Sample Index')
                self.ax_error.set_ylabel('Absolute Error (m)')
                self.ax_error.grid(True)
                self.ax_error.legend()
                
                self.fig_error.tight_layout()
                self.canvas_error.draw()
                
        except Exception as e:
            print(f"Error in update_error_plot: {e}")

    def update_condition_plot(self):
        """Update condition number plot"""
        try:
            # Clear old plot
            self.ax_cond_all.clear()
            self.ax_cond_recent.clear()
            
            # Calculate current condition number
            if len(self.data_dict_list) > 0:
                q_list = [data_dict['q'][12:19] if self.side == 'l' else data_dict['q'][19:26] 
                         for data_dict in self.data_dict_list]
                analyzer = identifiability_analyzer(self.fk, q_list)
                cond_num = analyzer.get_cond_num_i()
                
                # Only add condition number when new data is collected
                if len(self.condition_numbers) < len(self.data_dict_list):
                    self.condition_numbers.append(cond_num)
            
            # Plot all historical data
            self.ax_cond_all.semilogy(range(1, len(self.condition_numbers) + 1), 
                                     self.condition_numbers, 'b-', marker='o')
            self.ax_cond_all.set_title('All Historical Condition Numbers')
            self.ax_cond_all.grid(True)
            
            # Plot recent 5 data
            recent_data = self.condition_numbers[-5:] if len(self.condition_numbers) > 5 else self.condition_numbers
            x_recent = range(max(1, len(self.condition_numbers) - 4), len(self.condition_numbers) + 1)
            
            line = self.ax_cond_recent.plot(x_recent, recent_data, 'r-', marker='o')[0]
            
            # Add value labels
            for x, y in zip(x_recent, recent_data):
                self.ax_cond_recent.annotate(f'{y:.2e}', 
                                           (x, y),
                                           textcoords="offset points",
                                           xytext=(0, 10),
                                           ha='center')
            
            self.ax_cond_recent.set_title('Recent 5 Condition Numbers')
            self.ax_cond_recent.grid(True)
            self.fig_cond.tight_layout()
            self.canvas_cond.draw()
        except Exception as e:
            print(f"Error in update_condition_plot: {e}")

    def update_joint_distribution(self):
        """Update joint distribution plot"""
        try:
            self.ax_joint.clear()
            
            if len(self.data_dict_list) > 0:
                q_list = [data_dict['q'][12:19] if self.side == 'l' else data_dict['q'][19:26] 
                         for data_dict in self.data_dict_list]
                joint_angles = np.array(q_list)
                
                colors = plt.cm.rainbow(np.linspace(0, 1, len(joint_angles)))
                
                for i in range(7):
                    x = (i + 1) + np.random.normal(0, 0.1, size=len(joint_angles))
                    self.ax_joint.scatter(x, np.rad2deg(joint_angles[:, i]), 
                                        c=colors, alpha=0.6)
                
                self.ax_joint.set_title('Joint Angle Distribution')
                self.ax_joint.set_xlabel('Joint index')
                self.ax_joint.set_ylabel('Joint angle (deg)')
                self.ax_joint.grid(True)
                self.fig_joint.tight_layout()
                self.canvas_joint.draw()
        except Exception as e:
            print(f"Error in update_joint_distribution: {e}")

    def update_sensitivity_plot(self):
        """Update sensitivity heat map"""
        try:
            self.ax_sens.clear()
            
            if len(self.data_dict_list) > 0:
                q_list = [data_dict['q'][12:19] if self.side == 'l' else data_dict['q'][19:26] 
                         for data_dict in self.data_dict_list]
                analyzer = identifiability_analyzer(self.fk, q_list)
                s_list, U_list, Vt_list = analyzer.get_singular_values_list()
                s = s_list[0]
                Vt = Vt_list[0]
                
                sensitivity_ratios = analyzer.parameter_sensitivity_ratio(s, Vt, 1e-12)
                normalized_ratios = sensitivity_ratios/max(sensitivity_ratios)
                
                data = normalized_ratios.reshape(-1, 1)
                im = self.ax_sens.imshow(data, aspect='auto', cmap='YlOrRd')
                
                for i in range(7):
                    self.ax_sens.text(0, i, f'{data[i][0]:.3f}', 
                                    ha='center', va='center')
                
                self.ax_sens.set_title('Normalized Sensitivity')
                self.fig_sens.tight_layout()
                self.canvas_sens.draw()
        except Exception as e:
            print(f"Error in update_sensitivity_plot: {e}")

    def update_position_comparison(self):
        """Update position comparison plots"""
        try:
            self.ax_pos_x.clear()
            self.ax_pos_y.clear()
            self.ax_pos_z.clear()
            
            if len(self.data_dict_list) > 0:
                fk_positions = []
                measured_positions = []
                
                for data in self.data_dict_list:
                    # Calculate FK position
                    q_arm = data['q'][12:19] if self.side == 'l' else data['q'][19:26]
                    fk_pos = self.fk(q_arm)[0]
                    fk_positions.append(fk_pos)
                    
                    # Calculate measured position from tag
                    q_head = data['q'][-2:]
                    p_bi, R_bi = self.head_fk(q_head)
                    p_bt = p_bi + R_bi @ data['tag_pos']
                    measured_positions.append(p_bt)
                
                fk_positions = np.array(fk_positions)
                measured_positions = np.array(measured_positions)
                x = range(len(fk_positions))
                
                # Plot X positions
                self.ax_pos_x.plot(x, fk_positions[:, 0], 'r-', label='FK')
                self.ax_pos_x.plot(x, measured_positions[:, 0], 'b--', label='Measured')
                self.ax_pos_x.set_title('X Position Comparison')
                self.ax_pos_x.grid(True)
                self.ax_pos_x.legend()
                
                # Plot Y positions
                self.ax_pos_y.plot(x, fk_positions[:, 1], 'r-', label='FK')
                self.ax_pos_y.plot(x, measured_positions[:, 1], 'b--', label='Measured')
                self.ax_pos_y.set_title('Y Position Comparison')
                self.ax_pos_y.grid(True)
                self.ax_pos_y.legend()
                
                # Plot Z positions
                self.ax_pos_z.plot(x, fk_positions[:, 2], 'r-', label='FK')
                self.ax_pos_z.plot(x, measured_positions[:, 2], 'b--', label='Measured')
                self.ax_pos_z.set_title('Z Position Comparison')
                self.ax_pos_z.grid(True)
                self.ax_pos_z.legend()
                
                self.fig_pos.tight_layout()
                self.canvas_pos.draw()
                
        except Exception as e:
            print(f"Error in update_position_comparison: {e}")

    def real_sensor_data_callback(self, msg):
        self.q = np.array(msg.data).flatten()
        self.q_updated = True
        self.track_target()

    def gazebo_sensor_data_callback(self, msg):
        self.q = np.array(msg.joint_data.joint_q).flatten()
        self.q_updated = True
        self.track_target()

    def track_target(self):
        try:
            # 更新头部追踪目标
            q_arm = self.q[12:19] if self.side == 'l' else self.q[19:26]
            p_bt_fk, R_bt_fk, _ = self.fk(q_arm)
            p_te = np.array([0, 0.16, 0])
            p_be = p_bt_fk + R_bt_fk @ p_te
            # print(f"p_bt_fk: {p_bt_fk}")
            # print(f"p_be: {p_be}")
            self.target_tracker.track_target(p_be)
            
        except Exception as e:
            print(f"Error in update_from_cube_detector: {e}")

    # def tag_pos_callback(self, msg):
    #     # print(f"tag_pos_callback: {msg}")
    #     if self.q is None:
    #         return

    #     for detection in msg.detections:
    #         pose = detection.pose.pose.pose
    #         if detection.id[0] == self.tag_id:
    #             self.tag_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    #             self.tag_orientation = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    #             self.tag_pos_updated = True
                
    #             # save tag data with newest q_arm
    #             curr_data_dict = {
    #                 'tag_pos': self.tag_pos,
    #                 'tag_ori': self.tag_orientation,
    #                 'q': self.q
    #             }
                
    #             # check if the new data is far from previous data
    #             if len(self.data_dict_list) > 0:
    #                 prev_data_dict = self.data_dict_list[-1]
    #                 pos_dis = np.linalg.norm(curr_data_dict['tag_pos'] - prev_data_dict['tag_pos'])
    #                 # Calculate quaternion distance
    #                 ori_dis = 1 - np.abs(np.dot(curr_data_dict['tag_ori'], prev_data_dict['tag_ori']))
                    
    #                 if pos_dis > self.pos_dis_threshold or ori_dis > self.ori_dis_threshold:
    #                     self.data_dict_list.append(curr_data_dict)
    #                     # Update data count
    #                     self.data_count_label.setText(f'Data Collected: {len(self.data_dict_list)}')
    #                     print(f"Collected pose {len(self.data_dict_list)}")
    #                     self.signals.update_signal.emit()
                        
    #             else:
    #                 self.data_dict_list.append(curr_data_dict)
    #                 # Update data count
    #                 self.data_count_label.setText(f'Data Collected: {len(self.data_dict_list)}')
    #                 print("Collected first pose.")
    #                 self.signals.update_signal.emit()

    def update_from_cube_detector(self):
        # print(f"update_from_cube_detector")
        """从立方体检测器获取最新的位姿数据"""
        if self.q is None:
            return
            
        position, orientation, timestamp, detected_tags = self.cube_detector.get_cube_pose()
        # print(f"position: {position}, detected_tags: {detected_tags}")
        
        # 只有在检测到立方体且时间戳有效时才处理数据
        if timestamp is not None and len(detected_tags) > 0:
            # 立方体位置作为标签位置
            self.tag_pos = position
            
            # 获取方向四元数 (w,x,y,z格式)
            quat = rot_to_quat(orientation)
            self.tag_orientation = quat
            self.tag_pos_updated = True
            
            # 保存标签数据与当前关节角度
            curr_data_dict = {
                'tag_pos': self.tag_pos,
                'tag_ori': self.tag_orientation,
                'q': self.q
            }
            # print(f"curr_data_dict: {curr_data_dict}")
            # 检查新数据是否与之前的数据有足够差异
            if len(self.data_dict_list) > 0:
                prev_data_dict = self.data_dict_list[-1]
                pos_dis = np.linalg.norm(curr_data_dict['tag_pos'] - prev_data_dict['tag_pos'])
                # 计算四元数距离
                ori_dis = 1 - np.abs(np.dot(curr_data_dict['tag_ori'], prev_data_dict['tag_ori']))
                
                if pos_dis > self.pos_dis_threshold or ori_dis > self.ori_dis_threshold:
                    self.data_dict_list.append(curr_data_dict)
                    # 更新数据计数
                    self.data_count_label.setText(f'Data Collected: {len(self.data_dict_list)}')
                    print(f"Collected pose {len(self.data_dict_list)}")
                    self.signals.update_signal.emit()
            else:
                self.data_dict_list.append(curr_data_dict)
                # 更新数据计数
                self.data_count_label.setText(f'Data Collected: {len(self.data_dict_list)}')
                print("Collected first pose.")
                self.signals.update_signal.emit()


    @staticmethod
    def optimize_arm(fk, data_dict, optimizer_type='nlopt_mma'):
        """Optimize arm calibration parameters using selected optimizer
        
        Args:
            fk: Forward kinematics function
            data_dict: Dictionary containing calibration data
            optimizer_type: Type of optimizer to use ('nlopt_mma', 'nlopt_lbfgs', or 'ipopt')
            
        Returns:
            numpy.ndarray: Optimized joint bias parameters
        """
        if optimizer_type == 'nlopt_mma':
            return ArmCalibrator.optimize_with_nlopt(fk, data_dict, nlopt.LD_MMA)
        elif optimizer_type == 'nlopt_lbfgs':
            return ArmCalibrator.optimize_with_nlopt(fk, data_dict, nlopt.LD_LBFGS)
        elif optimizer_type == 'nlopt_slsqp':
            return ArmCalibrator.optimize_with_nlopt(fk, data_dict, nlopt.LD_SLSQP)
        elif optimizer_type == 'nlopt_direct':
            return ArmCalibrator.optimize_with_nlopt(fk, data_dict, nlopt.GN_DIRECT)
        elif optimizer_type == 'nlopt_crs2':
            return ArmCalibrator.optimize_with_nlopt(fk, data_dict, nlopt.GN_CRS2_LM)
        elif optimizer_type == 'nlopt_var2':
            return ArmCalibrator.optimize_with_nlopt(fk, data_dict, nlopt.LD_VAR2)
        elif optimizer_type == 'ipopt':
            return ArmCalibrator.optimize_with_ipopt(fk, data_dict)
        else:
            print(f"Unknown optimizer type: {optimizer_type}")
            return None

    @staticmethod
    def optimize_with_nlopt(fk, data_dict, algorithm):
        """Optimize using NLopt"""
        n_vars = 7
        opt = nlopt.opt(algorithm, n_vars)
        opt.set_lower_bounds([-np.deg2rad(10)]*n_vars)
        opt.set_upper_bounds([+np.deg2rad(10)]*n_vars)
        
        objective = create_objective_function(fk, data_dict)
        opt.set_min_objective(objective)
        
        opt.set_ftol_rel(1e-8)
        opt.set_maxeval(1000)
        
        initial_delta = np.zeros(n_vars)
        
        # 使用字典映射算法ID到名称
        alg_names = {
            nlopt.LD_MMA: "MMA",
            nlopt.LD_LBFGS: "LBFGS",
            nlopt.LD_SLSQP: "SLSQP",
            nlopt.GN_DIRECT: "DIRECT",
            nlopt.GN_CRS2_LM: "CRS2",
            nlopt.LD_VAR2: "VAR2"
        }
        alg_name = alg_names.get(algorithm, str(algorithm))
        
        try:
            time_start = time.time()
            result = opt.optimize(initial_delta)
            time_end = time.time()
            print(f"NLopt ({alg_name}) final error:", opt.last_optimum_value())
            print(f"NLopt ({alg_name}) found bias:", result)
            print(f"Optimization time: {1e3*(time_end - time_start):.2f} ms")
            return result
        except Exception as e:
            print(f"NLopt ({alg_name}) optimization error:", e)
            return None

    @staticmethod
    def optimize_with_ipopt(fk, data_dict):
        """Optimize using IPOPT"""
        class CalibrationProblem:
            def __init__(self, fk_func, data_dict):
                self.fk_func = fk_func
                self.data_dict = data_dict
                self.n_vars = 7
                self.iter_count = 0
                
            def objective(self, x):
                """Calculate objective function value"""
                total_error = 0.0
                for q, true_pos, true_rot in zip(self.data_dict['q'], 
                                               self.data_dict['true_pos'],
                                               self.data_dict['true_rot']):
                    q = np.array(q).flatten()
                    pred_pos, pred_rot, _ = self.fk_func(q + x)
                    
                    error_pos = pred_pos - true_pos
                    error_rot = pin.log3(pred_rot @ true_rot.T)
                    error = np.concatenate([error_pos, error_rot])
                    total_error += np.sum(error**2)
                return float(total_error)
                
            def gradient(self, x):
                """Calculate gradient using finite differences"""
                grad = np.zeros(self.n_vars)
                eps = 1e-6
                for i in range(self.n_vars):
                    x_plus = x.copy()
                    x_plus[i] += eps
                    obj_plus = self.objective(x_plus)
                    
                    x_minus = x.copy()
                    x_minus[i] -= eps
                    obj_minus = self.objective(x_minus)
                    
                    grad[i] = (obj_plus - obj_minus) / (2 * eps)
                return grad
                
            def intermediate(self, alg_mod, iter_count, obj_value, inf_pr, inf_du,
                           mu, d_norm, regularization_size, alpha_du, alpha_pr,
                           ls_trials):
                """Callback after each iteration"""
                self.iter_count = iter_count
                if iter_count % 5 == 0:  # Print every 5 iterations
                    print(f"Iteration {iter_count}, objective value: {obj_value}")

        try:
            time_start = time.time()
            
            # Create problem instance
            problem = CalibrationProblem(fk, data_dict)
            
            # Set bounds
            lb = [-np.deg2rad(10)] * 7
            ub = [np.deg2rad(10)] * 7
            
            # Initial guess
            x0 = np.zeros(7)
            
            # Create IPOPT problem
            nlp = cyipopt.Problem(
                n=7,                          # Number of variables
                m=0,                          # Number of constraints
                problem_obj=problem,          # Problem instance
                lb=lb,                        # Lower bounds
                ub=ub,                        # Upper bounds
            )
            
            # Set IPOPT options
            nlp.add_option('max_iter', 1000)
            nlp.add_option('tol', 1e-8)
            nlp.add_option('print_level', 0)  # Reduce IPOPT output
            
            # Solve the problem
            result = nlp.solve(x0)[0]
            
            time_end = time.time()
            print("IPOPT final error:", problem.objective(result))
            print("IPOPT found bias:", result)
            print(f"Optimization time: {1e3*(time_end - time_start):.2f} ms")
            print(f"Total iterations: {problem.iter_count}")
            
            return result
            
        except Exception as e:
            print("IPOPT optimization error:", e)
            return None

    def cali_arm(self, n_poses):
        while len(self.data_dict_list) < n_poses:
            # print(f"Waiting for {n_poses - len(self.data_dict_list)} poses...")
            time.sleep(0.01)
        print(f"Collected {len(self.data_dict_list)} poses, start optimizing...")
        # # 执行数据清洗
        # self.remove_noisy_data()  # 新增数据清洗步骤
        # # 检查清洗后数据量
        # if len(self.data_dict_list) < 2:
        #     self.result_text.setText("清洗后有效数据不足，无法校准")
        #     return
        
        # # 原有数据处理流程...
        # print(f"清洗后数据量: {len(self.data_dict_list)}，开始优化...")
        # trans data_dict_list to data_dict
        data_dict = {}
        data_dict['q'] = []
        data_dict['true_pos'] = []
        data_dict['true_rot'] = []
        for data in self.data_dict_list:
            q_arm = data['q'][12:12+7] if self.side == 'l' else data['q'][12+7:12+14]
            # print(f"q_arm: {q_arm}")
            data_dict['q'].append(q_arm)
            q_head = data['q'][-2:]
            # print(f"q_head: {q_head}")
            p_it = data['tag_pos']
            quat_it = data['tag_ori']
            # print(f"p_it: {p_it}, quat_it: {quat_it}")
            p_bi, R_bi = self.head_fk(q_head)
            p_bt = p_bi + R_bi @ p_it
            R_bt = R_bi @ quat_to_rot(quat_it)
            data_dict['true_pos'].append(p_bt)
            data_dict['true_rot'].append(R_bt)
            p_bt_fk = self.fk(q_arm)[0]
            R_bt_fk = self.fk(q_arm)[1]
            print(f"p_bt_meas: {p_bt}")
            print(f"p_bt_fk  : {p_bt_fk}")
            print(f"quat_bt_meas: {rot_to_quat(R_bt)}")
            print(f"quat_bt_fk  : {rot_to_quat(R_bt_fk)}")

        print("--------------------------------------------------------")
        print("--------------------------------------------------------")
        # Get selected optimizer type
        optimizer_type = self.optimizer_combo.currentText()
        delta = self.optimize_arm(self.fk, data_dict, optimizer_type)
        if delta is not None:
            print(f"optimized delta: {delta}")
            return delta
        else:
            print("Failed to optimize arm")
            return None

    def test(self, q_arm, q_head, p_it, quat_it):
        p_bt_fk, R_bt_fk, _ = self.fk(q_arm)
        p_bi, R_bi = self.head_fk(q_head)
        # print(f"p_bi: {p_bi}")
        # print(f"quat_bi: {rot_to_quat(R_bi)}")
        p_bt_meas = p_bi + R_bi @ p_it
        R_bt_meas = R_bi @ quat_to_rot(quat_it)
        print(f"p_bt fk  : {p_bt_fk}")
        print(f"p_bt meas: {p_bt_meas}")
        print(f"quat_bt fk  : {rot_to_quat(R_bt_fk)}")
        print(f"quat_bt meas: {rot_to_quat(R_bt_meas)}")
        print('--------------------------------')
        print('--------------------------------')
        self.test_data_list_dict['p_bt_meas'].append(p_bt_meas)
        self.test_data_list_dict['quat_bt_meas'].append(rot_to_quat(R_bt_meas))
        self.test_data_list_dict['p_bt_fk'].append(p_bt_fk)
        self.test_data_list_dict['quat_bt_fk'].append(rot_to_quat(R_bt_fk))
        if len(self.test_data_list_dict['p_bt_meas']) > self.plot_size:
            self.plot_test_data(self.test_data_list_dict)

    @staticmethod
    def plot_test_data(test_data_list_dict):
        # fig, axs = plt.subplots(3, 1, figsize=(10, 15))
        p_bt_meas = np.array(test_data_list_dict['p_bt_meas'])
        p_bt_fk = np.array(test_data_list_dict['p_bt_fk'])
        quat_bt_meas = np.array(test_data_list_dict['quat_bt_meas'])
        quat_bt_fk = np.array(test_data_list_dict['quat_bt_fk'])

        # print(f"p_bt_meas: {p_bt_meas}")
        x = np.arange(len(p_bt_meas))
        plt.plot(x, p_bt_meas[:,0], 'r--', label='p_bt_meas_x')
        plt.plot(x, p_bt_meas[:,1], 'g--', label='p_bt_meas_y')
        plt.plot(x, p_bt_meas[:,2], 'b--', label='p_bt_meas_z')
        plt.plot(x, p_bt_fk[:,0], 'r-', label='p_bt_fk_x')
        plt.plot(x, p_bt_fk[:,1], 'g-', label='p_bt_fk_y')
        plt.plot(x, p_bt_fk[:,2], 'b-', label='p_bt_fk_z')
        plt.title('real line: fk, dashed line: tag')
        # ply.plot(x, p_bt_fk[:,0], label='p_bt_fk')
        plt.legend(loc='upper left')
        # axs[1].plot(x, quat_bt_meas[:,0], label='quat_bt_meas')
        # axs[1].plot(x, quat_bt_fk[:,0], label='quat_bt_fk')
        # axs[1].legend()
        # plt.show()
        plt.savefig('arm_cali.png')
        plt.clf() # clear the figure

    def run(self):
        """Run ROS node and Qt application"""
        try:
            while not rospy.is_shutdown():
                self.app.processEvents()
                rospy.sleep(0.01)  # Give other threads some time
                self.update_from_cube_detector()
        except Exception as e:
            print(f"Error in run: {e}")
        finally:
            self.window.close()

    def import_data(self):
        """Import calibration data from JSON file"""
        try:
            # Open file dialog
            filename, _ = QFileDialog.getOpenFileName(
                self.window,
                "Select Calibration Data File",
                "",
                "JSON Files (*.json)"
            )
            
            if not filename:  # User cancelled
                return
                
            # Load data from file
            with open(filename, 'r') as f:
                imported_data = json.load(f)
            
            # Verify data format
            if 'data' not in imported_data:
                raise ValueError("Invalid data format: 'data' key not found")
                
            data_dict = imported_data['data']
            required_keys = ['q', 'true_pos', 'true_rot', 'tag_pos', 'tag_ori']
            if not all(key in data_dict for key in required_keys):
                raise ValueError("Invalid data format: missing required keys")
            
            # Convert lists to numpy arrays
            self.data_dict_list = []
            for q, pos, rot, tag_pos, tag_ori in zip(data_dict['q'], 
                                 data_dict['true_pos'], 
                                 data_dict['true_rot'],
                                 data_dict['tag_pos'],
                                 data_dict['tag_ori']):
                data_point = {
                    'q': np.array(q),
                    'tag_pos': np.array(pos),
                    'tag_ori': np.array(rot),
                    'tag_pos': np.array(tag_pos),
                    'tag_ori': np.array(tag_ori)
                }
                self.data_dict_list.append(data_point)
            
            # Update data count display
            self.data_count_label.setText(f'Collected Data: {len(self.data_dict_list)}')
            
            # Update result text
            if hasattr(self, 'result_text'):
                self.result_text.setText(f"Successfully imported {len(self.data_dict_list)} data points from:\n{filename}")
            
            print(f"Imported {len(self.data_dict_list)} data points from {filename}")
            
            # Optional: Update plots or other visualizations if needed
            self.update_visualization()  # 如果有这个方法的话
            
        except Exception as e:
            error_msg = f"Error importing data: {str(e)}"
            print(error_msg)
            if hasattr(self, 'result_text'):
                self.result_text.setText(error_msg)

    def export_data(self):
        """Export collected data to JSON file"""
        if not self.data_dict_list:
            print("No data to export")
            return
            
        try:
            # Create data dictionary in the same format as used for optimization
            data_dict = {
                'q': [],
                'true_pos': [],
                'true_rot': [],
                'tag_pos': [],
                'tag_ori': []
            }
            
            # Process each data point
            for data in self.data_dict_list:
                # Get arm joint angles
                # q_arm = data['q'][12:12+7] if self.side == 'l' else data['q'][12+7:12+14]
                data_dict['q'].append(data['q'].tolist())  # Convert to list
                
                # Calculate transformed positions and rotations
                q_head = data['q'][-2:]
                p_bi, R_bi = self.head_fk(q_head)
                p_bt = p_bi + R_bi @ data['tag_pos']
                R_bt = R_bi @ quat_to_rot(data['tag_ori'])
                
                data_dict['true_pos'].append(p_bt.tolist())  # Convert to list
                data_dict['true_rot'].append(R_bt.tolist())  # Convert to list
                data_dict['tag_pos'].append(data['tag_pos'].tolist())  # Convert to list
                data_dict['tag_ori'].append(data['tag_ori'].tolist())  # Convert to list

            # Add metadata
            export_data = {
                'timestamp': datetime.now().strftime('%Y%m%d_%H%M%S'),
                'side': self.side,
                'data': data_dict
            }
            
            # Open file dialog for saving
            filename, _ = QFileDialog.getSaveFileName(
                self.window,
                "Save Calibration Data",
                f'calibration_data_{export_data["timestamp"]}_{self.side}_arm.json',
                "JSON Files (*.json)"
            )
            
            if not filename:  # User cancelled
                return
                
            # Save to file
            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=4)
            
            print(f"Data exported to {filename}")
            
            # Update result text in UI
            if hasattr(self, 'result_text'):
                current_text = self.result_text.toPlainText()
                self.result_text.setText(f"{current_text}\n\nData exported to:\n{filename}")
                
        except Exception as e:
            error_msg = f"Error exporting data: {e}"
            print(error_msg)
            if hasattr(self, 'result_text'):
                self.result_text.setText(error_msg)

    def set_up_tag_detector(self):
        """从YAML配置文件读取立方体参数并设置标签检测器"""
        try:
            # 尝试加载配置文件
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config', 'cube_config.yaml')
            
            # 如果指定了配置文件路径，则使用指定路径
            if hasattr(self, 'config_path') and self.config_path:
                config_path = self.config_path
                
            # 检查文件是否存在
            if not os.path.exists(config_path):
                rospy.logwarn(f"配置文件不存在: {config_path}，将使用默认参数")
                return self._set_up_tag_detector_with_defaults()
                
            # 加载YAML文件
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                
            # 读取立方体参数
            cube_params = config.get('cube_params', {})
            cube_size = cube_params.get('cube_size', 0.11)        # 默认立方体边长11cm
            tag_size = cube_params.get('tag_size', 0.088)         # 默认标签边长8.8cm
            connector_height = cube_params.get('connector_height', 0.165)  # 默认连接高度16.5cm
            
            # 读取面旋转配置
            face_rotations = config.get('face_rotations', {
                0: 0,   # 前面：0°
                1: 0,   # 后面：0°
                2: 0,   # 左面：0°
                3: 0,   # 右面：0°
                4: 0    # 顶面：0°
            })
            
            # 读取标签ID映射
            tag_mappings = config.get('tag_id_mappings', {})
            
            # 根据机器人侧面选择对应的标签映射
            side_key = 'left' if self.side == 'l' else 'right'
            tag_id_mapping = tag_mappings.get(side_key, None)
            
            if tag_id_mapping is None:
                rospy.logwarn(f"未找到{side_key}侧的标签ID映射，将使用默认映射")
                # 使用默认映射
                if self.side == 'l':
                    tag_id_mapping = {
                        0: 0,  # ID 0 -> 前面
                        1: 1,  # ID 1 -> 后面
                        2: 2,  # ID 2 -> 左面
                        3: 3,  # ID 3 -> 右面
                        4: 4   # ID 4 -> 顶面
                    }
                else:  # self.side == 'r'
                    tag_id_mapping = {
                        5: 0,  # ID 5 -> 前面
                        6: 1,  # ID 6 -> 后面
                        7: 2,  # ID 7 -> 左面
                        8: 3,  # ID 8 -> 右面
                        9: 4   # ID 9 -> 顶面
                    }
            
            # 读取面权重配置
            face_weights = config.get('face_weights', {})
            
            # 创建立方体检测器
            cube_detector = AprilTag3DRos(
                a=cube_size, 
                b=tag_size,
                face_rotations=face_rotations,
                h=connector_height,
                tag_id_mapping=tag_id_mapping
            )
            
            # 设置面检测权重
            for face_id, weight in face_weights.items():
                cube_detector.set_face_weight(int(face_id), float(weight))
            
            # 读取过滤阈值
            filter_thresholds = config.get('filter_thresholds', {})
            position_thresh = filter_thresholds.get('position', 0.05)
            rotation_thresh = filter_thresholds.get('rotation', 0.2)
            
            # 设置过滤阈值
            cube_detector.set_filter_thresholds(
                position_thresh=position_thresh, 
                rotation_thresh=rotation_thresh
            )
            
            rospy.loginfo(f"成功从{config_path}加载立方体配置")
            # 打印立方体检测器参数
            cube_detector.print_params()
            return cube_detector
            
        except Exception as e:
            rospy.logerr(f"加载立方体配置时出错: {e}")
            return self._set_up_tag_detector_with_defaults()
    
    def _set_up_tag_detector_with_defaults(self):
        """使用默认参数设置标签检测器"""
        # 定义立方体参数
        cube_size = 0.11       # 立方体边长11cm
        tag_size = 0.088       # 标签边长8.8cm
        connector_height = 0.165  # 连接结构到中心的距离16.5cm
        
        # 定义各面旋转方向
        face_rotations = {
            0: 0,   # 前面：0°
            1: 0,   # 后面：0°
            2: 0,   # 左面：0°
            3: 0,   # 右面：0°
            4: 0    # 顶面：0°
        }
        
        # 定义标签ID映射
        tag_id_mapping = None
        if self.side == 'l':
            tag_id_mapping = {
                0: 0,  # ID 0 -> 前面
                1: 1,  # ID 1 -> 后面
                2: 2,  # ID 2 -> 左面
                3: 3,  # ID 3 -> 右面
                4: 4   # ID 4 -> 顶面
            }
        elif self.side == 'r':
            tag_id_mapping = {
                5: 0,  # ID 5 -> 前面
                6: 1,  # ID 6 -> 后面
                7: 2,  # ID 7 -> 左面
                8: 3,  # ID 8 -> 右面
                9: 4   # ID 9 -> 顶面
            }
        
        # 创建立方体检测器
        cube_detector = AprilTag3DRos(
            a=cube_size, 
            b=tag_size,
            face_rotations=face_rotations,
            h=connector_height,
            tag_id_mapping=tag_id_mapping
        )
        
        # 设置面检测的权重
        cube_detector.set_face_weight(0, 1.2)  # 增加前面的权重
        cube_detector.set_face_weight(1, 1.2)  # 增加后面的权重
        
        # 设置过滤阈值
        cube_detector.set_filter_thresholds(position_thresh=0.05, rotation_thresh=0.2)
        
        rospy.logwarn("使用默认立方体配置")
        cube_detector.print_params()
        return cube_detector

    def publish_tf(self, p, quat, parent_frame_id, child_frame_id):
        # 发布TF转换
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame_id
        
        transform.child_frame_id = child_frame_id
        
        # 设置位置
        transform.transform.translation.x = p[0]
        transform.transform.translation.y = p[1]
        transform.transform.translation.z = p[2]
        
        # 设置旋转
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        
        # 广播TF
        self.tf_broadcaster.sendTransform(transform)

    def evaluate_calibration(self):
        """评估标定前后的误差"""
        if not self.data_dict_list:
            self.result_text.setText("Please collect data and complete calibration first")
            return
        
        try:
            # 获取标定结果
            delta_result = self.result if self.result is not None else np.zeros(7)
            
            # 存储误差数据
            errors = {
                'pre_pos': [],
                'post_pos': [],
                'pre_rot': [],
                'post_rot': []
            }
            
            # 遍历所有数据点
            for data in self.data_dict_list:
                q_arm = data['q'][12:19] if self.side == 'l' else data['q'][19:26]
                q_head = data['q'][-2:]
                
                # 真实测量值
                p_bi, R_bi = self.head_fk(q_head)
                p_bt_true = p_bi + R_bi @ data['tag_pos']
                R_bt_true = R_bi @ quat_to_rot(data['tag_ori'])
                
                # 标定前预测值
                p_bt_pre, R_bt_pre, _ = self.fk(q_arm)
                
                # 标定后预测值
                p_bt_post, R_bt_post, _ = self.fk(q_arm + delta_result)
                
                # 计算位置误差
                errors['pre_pos'].append(np.linalg.norm(p_bt_pre - p_bt_true))
                errors['post_pos'].append(np.linalg.norm(p_bt_post - p_bt_true))
                
                # 计算姿态误差（使用旋转矩阵的轴角表示）
                R_err_pre = R_bt_pre @ R_bt_true.T
                R_err_post = R_bt_post @ R_bt_true.T
                errors['pre_rot'].append(np.linalg.norm(pin.log3(R_err_pre)))
                errors['post_rot'].append(np.linalg.norm(pin.log3(R_err_post)))
            
            # 转换为numpy数组
            for k in errors:
                errors[k] = np.array(errors[k])
            
            # 计算统计量
            stats = {
                'pos_pre_mean': np.mean(errors['pre_pos']),
                'pos_pre_std': np.std(errors['pre_pos']),
                'pos_post_mean': np.mean(errors['post_pos']),
                'pos_post_std': np.std(errors['post_pos']),
                'rot_pre_mean': np.mean(errors['pre_rot']),
                'rot_pre_std': np.std(errors['pre_rot']),
                'rot_post_mean': np.mean(errors['post_rot']),
                'rot_post_std': np.std(errors['post_rot'])
            }
            
            # 生成结果文本
            result_text = "Calibration Evaluation Result:\n\n"
            result_text += "Position Error (m):\n"
            result_text += f"Before Calibration: Mean={stats['pos_pre_mean']:.4f} ± {stats['pos_pre_std']:.4f}\n"
            result_text += f"After Calibration: Mean={stats['pos_post_mean']:.4f} ± {stats['pos_post_std']:.4f}\n\n"
            result_text += "Orientation Error (rad):\n"
            result_text += f"Before Calibration: Mean={stats['rot_pre_mean']:.4f} ± {stats['rot_pre_std']:.4f}\n"
            result_text += f"After Calibration: Mean={stats['rot_post_mean']:.4f} ± {stats['rot_post_std']:.4f}"
            
            # 更新文本显示
            self.result_text.setText(result_text)
            
            # 绘制误差对比图
            self.plot_error_comparison(errors)
            
        except Exception as e:
            self.result_text.setText(f"Evaluation Error: {str(e)}")

    def plot_error_comparison(self, errors):
        """更新主界面中的评估图表"""
        # 清空原有绘图
        self.ax_eval_pos.clear()
        self.ax_eval_rot.clear()
        
        # 位置误差对比
        self.ax_eval_pos.boxplot([errors['pre_pos'], errors['post_pos']], 
                               labels=['Before Calibration', 'After Calibration'],
                               patch_artist=True,
                               boxprops=dict(facecolor='lightblue'))
        self.ax_eval_pos.set_title('Position Error Comparison (m)')
        self.ax_eval_pos.grid(True)
        
        # 姿态误差对比
        self.ax_eval_rot.boxplot([errors['pre_rot'], errors['post_rot']], 
                               labels=['Before Calibration', 'After Calibration'],
                               patch_artist=True,
                               boxprops=dict(facecolor='lightgreen'))
        self.ax_eval_rot.set_title('Orientation Error Comparison (rad)')
        self.ax_eval_rot.grid(True)
        
        # 更新画布
        self.fig_eval.tight_layout()
        self.canvas_eval.draw()
        
        # 移除创建新窗口的代码

    @staticmethod
    def modify_arm_zero_yaml(yaml_file_path, delta_q_arm):
        assert len(delta_q_arm) == 12, "手的关节偏置量必须是12x1数组"
        # 读取 YAML 文件
        with open(yaml_file_path, 'r') as file:
            data = yaml.safe_load(file)

        # 备份原始文件
        yaml_backup_path = yaml_file_path + ".arm_cali.bak"
        with open(yaml_backup_path, 'w') as file:
            yaml.dump(data, file, default_flow_style=False, allow_unicode=True)
            print(f"YAML backup saved to {yaml_backup_path}")

        # 检查是否存在 arms_zero_position
        if "arms_zero_position" in data:
            # 获取倒数第二和倒数第一个值
            original_value = data["arms_zero_position"]
            print(f"Original value: {original_value}")

            # 解析 NumPy 的二进制格式（如果存在）
            def parse_numpy_scalar(value):
                if isinstance(value, dict) and "!!binary |" in value:
                    binary_data = base64.b64decode(value["!!binary |"])
                    return struct.unpack('<d', binary_data)[0]
                return value

            for i in range(12):
                data["arms_zero_position"][i] = parse_numpy_scalar(original_value[i]) + float(delta_q_arm[i])

            # 打印修改后的值
            print("Modified values:")
            for i in range(12):
                print(f"Joint {i+1} value: {data['arms_zero_position'][i]}")

            # 将修改后的内容写回 YAML 文件
            with open(yaml_file_path, 'w') as file:
                yaml.dump(data, file, default_flow_style=False, allow_unicode=True)
        else:
            print("arms_zero_position key not found in the YAML file.")

    @staticmethod
    def modify_arm_elmo_offset_csv(delta_q):
        """修改手的elmo关节偏置
        
        Args:
            delta_q: 手的elmo关节偏置量, 2x1数组, 单位为度
        """
        try:
            # 确保输入是2x1数组
            assert len(delta_q) == 2, "手的elmo关节偏置量必须是2x1数组"
            
            # 获取offset.csv文件路径
            home_dir = os.path.expanduser('~')
            offset_file_path = os.path.join(home_dir, '.config/lejuconfig/offset.csv')
            print(f"offset_file_path: {offset_file_path}")
            
            # 检查文件是否存在
            if not os.path.exists(offset_file_path):
                print(f"错误：文件不存在 {offset_file_path}")
                return False
                
            # 读取CSV文件内容
            with open(offset_file_path, 'r') as file:
                lines = file.readlines()
                
            # 确保文件至少有14行
            if len(lines) < 14:
                print(f"错误：文件行数不足，需要至少14行，但只有{len(lines)}行")
                return False
                
            # 创建备份文件
            backup_file_path = offset_file_path + ".arm_cali.bak"
            with open(backup_file_path, 'w') as file:
                file.writelines(lines)
            print(f"备份文件已保存到 {backup_file_path}")
            
            # 修改第13和14行的值（索引为12和13）
            for i in range(12, 14):
                if i < len(lines):
                    try:
                        # 解析当前值
                        current_value = float(lines[i].strip())
                        # 添加偏置量（转换为角度）
                        new_value = current_value + np.rad2deg(delta_q[i-12])
                        # 更新行
                        lines[i] = f"{new_value}\n"
                        print(f"第{i+1}行: {current_value} -> {new_value}")
                    except ValueError:
                        print(f"错误：无法解析第{i+1}行的值: {lines[i]}")
                        return False
            
            # 将修改后的内容写回文件
            with open(offset_file_path, 'w') as file:
                file.writelines(lines)
                
            print(f"手的elmo关节偏置已成功修改")
            return True
            
        except Exception as e:
            print(f"修改手的elmo关节偏置时出错: {str(e)}")
            return False

    def execute_hand_move(self):
        if not self.is_hand_move_enabled:
            self.result_text.append("The robot arm move function is disabled! Please enable it first.")
            return
        """执行预录制的示教轨迹"""
        try:
            from PyQt5.QtCore import QProcess
            
            # 获取当前脚本所在目录
            script_dir = os.path.dirname(os.path.abspath(__file__))
            bag_file = os.path.join(script_dir, "bags/hand_move_demo_" + ('left' if self.side == 'l' else 'right') + ".bag")
            
            if not os.path.exists(bag_file):
                QMessageBox.warning(self.window, 'Warning', 
                                  f'Hand move demo bag file not found: {bag_file}',
                                  QMessageBox.Ok)
                return
            
            # 创建并配置QProcess
            self.rosbag_process = QProcess()
            self.rosbag_process.setProcessChannelMode(QProcess.MergedChannels)
            self.rosbag_process.readyReadStandardOutput.connect(self.handle_rosbag_output)
            
            # 设置命令参数
            command = "rosbag"
            args = ["play", bag_file, "--clock"]
            
            # 显示执行提示
            self.result_text.append("\nStart executing hand move demo...")
            
            # 启动进程
            self.rosbag_process.start(command, args)
            
            # 添加进程结束处理
            self.rosbag_process.finished.connect(lambda: self.result_text.append("Hand move demo finished"))
            
        except Exception as e:
            QMessageBox.critical(self.window, 'Error', 
                               f'Execution failed: {str(e)}',
                               QMessageBox.Ok)

    def handle_rosbag_output(self):
        """处理rosbag输出到日志"""
        data = self.rosbag_process.readAllStandardOutput().data().decode()
        self.result_text.append(f"[ROSbag] {data.strip()}")

    # 添加空的事件处理函数，由用户自行实现
    def enable_move(self):
        """启用机器人移动功能"""
        self.enable_head_tracking()
        change_arm_ctrl_mode(2)
        change_kinematic_mpc_ctrl_mode(1)
        # change_wbc_trajectory_ctrl_mode(1)
        self.is_hand_move_enabled = True
        self.result_text.append("The robot arm move function is enabled")

    def disable_move(self):
        """禁用机器人移动功能"""
        self.disable_head_tracking()
        change_arm_ctrl_mode(1)
        change_kinematic_mpc_ctrl_mode(0)
        change_wbc_trajectory_ctrl_mode(0)
        self.is_hand_move_enabled = False
        self.result_text.append("The robot arm move function is disabled")

    def setup_head_tracking_controls(self):
        """设置头部追踪控制按钮"""
        # 创建头部追踪控制布局
        tracking_layout = QHBoxLayout()
        
        # 添加启用头部追踪按钮
        self.enable_tracking_btn = QPushButton('Enable Head Tracking')
        self.enable_tracking_btn.setToolTip('Enable head tracking')
        self.enable_tracking_btn.clicked.connect(self.enable_head_tracking)
        tracking_layout.addWidget(self.enable_tracking_btn)
        
        # 添加禁用头部追踪按钮
        self.disable_tracking_btn = QPushButton('Disable Head Tracking')
        self.disable_tracking_btn.setToolTip('Disable head tracking')
        self.disable_tracking_btn.clicked.connect(self.disable_head_tracking)
        tracking_layout.addWidget(self.disable_tracking_btn)
        
        # 添加验证标定结果按钮
        self.validate_btn = QPushButton('Validate Calibration')
        self.validate_btn.setToolTip('Send desired pose and validate calibration result')
        self.validate_btn.clicked.connect(self.validate_calibration)
        tracking_layout.addWidget(self.validate_btn)
        
        # 将追踪控制布局添加到控制面板
        # 获取控制面板布局
        control_panel = self.window.centralWidget().layout().itemAt(0).widget()
        control_layout = control_panel.layout()
        control_layout.addLayout(tracking_layout)
    
    def enable_head_tracking(self):
        """启用头部追踪"""
        self.target_tracker.set_tracking(True)
        self.result_text.append("Head tracking function is enabled")
    
    def disable_head_tracking(self):
        """禁用头部追踪"""
        self.target_tracker.set_tracking(False)
        self.target_tracker.back_to_zero()
        self.result_text.append("Head tracking function is disabled")

    def remove_noisy_data(self):
        """基于误差分析去除噪声数据"""
        if len(self.data_dict_list) < 2:
            return

        pos_errors = []
        rot_errors = []

        # 第一阶段：计算所有数据点的原始误差
        for data in self.data_dict_list:
            # 计算预测位置
            q_arm = data['q'][12:19] if self.side == 'l' else data['q'][19:26]
            p_bt_fk, R_bt_fk, _ = self.fk(q_arm)
            
            # 计算实际测量位置
            q_head = data['q'][-2:]
            p_bi, R_bi = self.head_fk(q_head)
            p_bt_meas = p_bi + R_bi @ data['tag_pos']
            R_bt_meas = R_bi @ quat_to_rot(data['tag_ori'])
            
            # 计算位置误差
            pos_error = np.linalg.norm(p_bt_meas - p_bt_fk)
            pos_errors.append(pos_error)
            
            # 计算方向误差
            R_error = R_bt_fk @ R_bt_meas.T
            rot_error = np.linalg.norm(pin.log3(R_error))
            rot_errors.append(rot_error)
    
        # 第二阶段：基于MAD的离群值检测
        def mad_based_outlier(data):
            median = np.median(data)
            mad = np.median(np.abs(data - median))
            return median, mad
        
        # 位置误差过滤
        pos_median, pos_mad = mad_based_outlier(pos_errors)
        pos_threshold = pos_median + 3 * pos_mad
        
        # 方向误差过滤  
        rot_median, rot_mad = mad_based_outlier(rot_errors)
        rot_threshold = rot_median + 3 * rot_mad
        
        # 第三阶段：执行过滤
        filtered_data = []
        for data, pos_err, rot_err in zip(self.data_dict_list, pos_errors, rot_errors):
            if pos_err <= pos_threshold and rot_err <= rot_threshold:
                filtered_data.append(data)
        
        # 更新数据并显示过滤结果
        original_count = len(self.data_dict_list)
        self.data_dict_list = filtered_data
        filtered_count = len(self.data_dict_list)
        
        info_text = f"Data filtering completed:\nOriginal data: {original_count}\nFiltered data: {filtered_count}\n"
        info_text += f"Position threshold: {pos_threshold:.4f}m\nOrientation threshold: {rot_threshold:.4f}rad"
        self.result_text.setText(info_text)
        self.update_error_plot()

    def validate_calibration(self):
        """验证标定结果"""
        try:
            # 创建新窗口
            self.validation_window = QMainWindow()
            self.validation_window.setWindowTitle('Validation of Calibration Result')
            self.validation_window.setGeometry(200, 200, 1200, 800)
            
            # 创建中央部件
            central_widget = QWidget()
            self.validation_window.setCentralWidget(central_widget)
            main_layout = QVBoxLayout(central_widget)
            
            # 创建控制面板
            control_panel = QWidget()
            control_layout = QHBoxLayout(control_panel)
            main_layout.addWidget(control_panel)
            
            # 添加位置输入控件
            pos_group = QWidget()
            pos_layout = QHBoxLayout(pos_group)
            pos_layout.addWidget(QLabel('Desired position (m):'))
            
            self.pos_x_input = QLineEdit('0.4')
            self.pos_y_input = QLineEdit('0.0')
            self.pos_z_input = QLineEdit('0.3')
            
            pos_layout.addWidget(QLabel('X:'))
            pos_layout.addWidget(self.pos_x_input)
            pos_layout.addWidget(QLabel('Y:'))
            pos_layout.addWidget(self.pos_y_input)
            pos_layout.addWidget(QLabel('Z:'))
            pos_layout.addWidget(self.pos_z_input)
            
            control_layout.addWidget(pos_group)
            
            # 添加姿态输入控件 (欧拉角)
            ori_group = QWidget()
            ori_layout = QHBoxLayout(ori_group)
            ori_layout.addWidget(QLabel('Desired orientation (deg):'))
            
            self.ori_roll_input = QLineEdit('0')
            self.ori_pitch_input = QLineEdit('-90')
            self.ori_yaw_input = QLineEdit('0')
            
            ori_layout.addWidget(QLabel('Roll:'))
            ori_layout.addWidget(self.ori_roll_input)
            ori_layout.addWidget(QLabel('Pitch:'))
            ori_layout.addWidget(self.ori_pitch_input)
            ori_layout.addWidget(QLabel('Yaw:'))
            ori_layout.addWidget(self.ori_yaw_input)
            
            control_layout.addWidget(ori_group)
            
            # 添加发送按钮
            self.send_pose_btn = QPushButton('Send desired pose')
            self.send_pose_btn.clicked.connect(self.send_desired_pose)
            control_layout.addWidget(self.send_pose_btn)
            
            # 添加开始/停止验证按钮
            self.start_validation_btn = QPushButton('Start validation')
            self.start_validation_btn.clicked.connect(self.start_validation)
            control_layout.addWidget(self.start_validation_btn)
            
            self.stop_validation_btn = QPushButton('Stop validation')
            self.stop_validation_btn.clicked.connect(self.stop_validation)
            control_layout.addWidget(self.stop_validation_btn)
            
            # 创建图表区域
            plot_widget = QWidget()
            plot_layout = QVBoxLayout(plot_widget)
            main_layout.addWidget(plot_widget)
            
            # 位置误差图表
            self.fig_pos_error = Figure(figsize=(12, 4))
            self.ax_pos_error = self.fig_pos_error.add_subplot(111)
            self.canvas_pos_error = FigureCanvas(self.fig_pos_error)
            plot_layout.addWidget(self.canvas_pos_error)
            
            # 姿态误差图表
            self.fig_ori_error = Figure(figsize=(12, 4))
            self.ax_ori_error = self.fig_ori_error.add_subplot(111)
            self.canvas_ori_error = FigureCanvas(self.fig_ori_error)
            plot_layout.addWidget(self.canvas_ori_error)
            
            # 数据表格区域
            self.error_text = QTextEdit()
            self.error_text.setReadOnly(True)
            self.error_text.setMaximumHeight(150)
            main_layout.addWidget(self.error_text)
            
            # 初始化验证数据
            self.validation_data = {
                'time': [],
                'p_d': [],  # 期望位置
                'p_q': [],  # 关节计算位置
                'p_t': [],  # 标签测量位置
                'q_d': [],  # 期望姿态
                'q_q': [],  # 关节计算姿态
                'q_t': []   # 标签测量姿态
            }
            
            # 初始化验证定时器
            self.validation_timer = QTimer()
            self.validation_timer.timeout.connect(self.update_validation)
            self.validation_active = False
            
            # 初始化ROS发布者
            self.desired_pose_pub = rospy.Publisher('/ik/two_arm_hand_pose_cmd', twoArmHandPoseCmd, queue_size=10)
            
            # 显示窗口
            self.validation_window.show()
            
        except Exception as e:
            QMessageBox.critical(self.window, 'Error', 
                               f'Failed to create validation window: {str(e)}',
                               QMessageBox.Ok)

    def send_desired_pose(self):
        """发送期望位姿到ROS话题"""
        try:
            # 获取输入的位置
            x = float(self.pos_x_input.text())
            y = float(self.pos_y_input.text())
            z = float(self.pos_z_input.text())
            
            # 获取输入的姿态（欧拉角）并转换为四元数
            roll = np.deg2rad(float(self.ori_roll_input.text()))
            pitch = np.deg2rad(float(self.ori_pitch_input.text()))
            yaw = np.deg2rad(float(self.ori_yaw_input.text()))
            
            quat_xyzw = tf_trans.quaternion_from_euler(roll, pitch, yaw)
            p_l0, R_l0 = self.fk_arm.eef_FK_l(np.zeros(7))
            p_r0, R_r0 = self.fk_arm.eef_FK_r(np.zeros(7))
            
            # 创建消息
            p_be = np.array([x, y, z])
            if self.side == 'l':
                quat_0_wxyz = rot_to_quat(R_r0)                
                quat_0 = [*quat_0_wxyz[1:], quat_0_wxyz[0]]
                eef_pose_msg = get_eef_pose_msg(p_be, quat_xyzw, p_r0, quat_0)
            else:
                quat_0_wxyz = rot_to_quat(R_l0)
                quat_0 = [*quat_0_wxyz[1:], quat_0_wxyz[0]]
                eef_pose_msg = get_eef_pose_msg(p_l0, quat_0, p_be, quat_xyzw)
            self.desired_pose_pub.publish(eef_pose_msg)
            
            # 更新验证数据中的期望位姿
            q_be = np.array([quat_xyzw[3], *quat_xyzw[:3]])  # w,x,y,z格式
            self.p_d = p_be
            self.q_d = q_be
            
            # 显示确认信息
            self.error_text.append(f"All arm has sent desired pose: position=[{x:.3f}, {y:.3f}, {z:.3f}], "
                                 f"orientation=[{np.rad2deg(roll):.1f}°, {np.rad2deg(pitch):.1f}°, {np.rad2deg(yaw):.1f}°]")
            
        except ValueError as e:
            self.error_text.append(f"Input error: {str(e)}")
        except Exception as e:
            self.error_text.append(f"Send pose failed: {str(e)}")

    def start_validation(self):
        """开始验证过程"""
        if not self.validation_active:
            # 清空之前的数据
            self.validation_data = {
                'time': [],
                'p_d': [],
                'p_q': [],
                'p_t': [],
                'q_d': [],
                'q_q': [],
                'q_t': []
            }
            
            # 设置初始期望位姿
            self.send_desired_pose()
            
            # 启动定时器
            self.validation_timer.start(10)  # 10Hz
            self.validation_active = True
            self.error_text.append("Start validation...")

    def stop_validation(self):
        """停止验证过程"""
        if self.validation_active:
            self.validation_timer.stop()
            self.validation_active = False
            self.error_text.append("Validation stopped")
            
            # 计算并显示统计结果
            self.show_validation_statistics()

    def update_validation(self):
        """更新验证数据和图表"""
        try:
            if self.q is None:
                return
            
            # 获取当前时间
            current_time = time.time()
            if len(self.validation_data['time']) > 0:
                relative_time = current_time - self.validation_data['time'][0]
            else:
                relative_time = 0
            
            # 获取关节角度计算的末端位姿
            q_arm = self.q[12:19] if self.side == 'l' else self.q[19:26]
            p_q, R_q = self.fk_arm.eef_FK_l(q_arm) if self.side == 'l' else self.fk_arm.eef_FK_r(q_arm)
            q_q = rot_to_quat(R_q)
            # print(f"p_q: {p_q}, q_q: {q_q}")
            
            # 获取标签测量的末端位姿
            position, orientation, timestamp, detected_tags = self.cube_detector.get_cube_pose()
            # print(f"position: {position}, orientation: {orientation}")
            # print(f"timestamp is not None: {timestamp is not None}, detected_tags length is {len(detected_tags)}")
            if timestamp is not None and len(detected_tags) > 0:
                q_head = self.q[-2:]
                p_bi, R_bi = self.head_fk(q_head)
                p_bt = p_bi + R_bi @ position
                R_bt = R_bi @ orientation
                q_bt = rot_to_quat(R_bt)
                
                p_te, R_te = self.fk_arm.get_T_teef(self.side)
                p_be = p_bt + R_bt @ p_te
                R_be = R_bt @ R_te
                q_be = rot_to_quat(R_be)
                # print(f"p_t: {p_t}, q_t: {q_t}")
                # 添加数据点
                self.validation_data['time'].append(relative_time)
                self.validation_data['p_d'].append(self.p_d.copy())
                self.validation_data['p_q'].append(p_q.copy())
                self.validation_data['p_t'].append(p_be.copy())
                self.validation_data['q_d'].append(self.q_d.copy())
                self.validation_data['q_q'].append(q_q.copy())
                self.validation_data['q_t'].append(q_be.copy())
                
                # 更新图表
                self.update_validation_plots()
                
                # 更新文本显示
                if len(self.validation_data['time']) % 10 == 0:  # 每10个数据点更新一次
                    self.update_validation_text()
                
        except Exception as e:
            self.error_text.append(f"Validation update error: {str(e)}")

    def update_validation_plots(self):
        """更新验证图表"""
        try:
            # 清空图表
            self.ax_pos_error.clear()
            self.ax_ori_error.clear()
            
            if len(self.validation_data['time']) < 2:
                return
            
            time_data = np.array(self.validation_data['time'])
            
            # 转换为numpy数组
            p_d = np.array(self.validation_data['p_d'])
            p_q = np.array(self.validation_data['p_q'])
            p_t = np.array(self.validation_data['p_t'])
            
            # 计算位置误差
            err_dq = np.linalg.norm(p_d - p_q, axis=1)
            err_dt = np.linalg.norm(p_d - p_t, axis=1)
            err_qt = np.linalg.norm(p_q - p_t, axis=1)
            
            # 绘制位置误差
            self.ax_pos_error.plot(time_data, err_dq, 'r-', label='Desired-Joint Error')
            self.ax_pos_error.plot(time_data, err_dt, 'g-', label='Desired-Tag Error')
            self.ax_pos_error.plot(time_data, err_qt, 'b-', label='Joint-Tag Error')
            self.ax_pos_error.set_title('Position Error (m)')
            self.ax_pos_error.set_xlabel('Time (s)')
            self.ax_pos_error.set_ylabel('Error (m)')
            self.ax_pos_error.grid(True)
            self.ax_pos_error.legend()
            
            # 计算姿态误差 (四元数距离)
            q_d = np.array(self.validation_data['q_d'])
            q_q = np.array(self.validation_data['q_q'])
            q_t = np.array(self.validation_data['q_t'])
            
            # 计算四元数距离 (1 - |q1·q2|)
            def quat_distance(q1, q2):
                return 1 - np.abs(np.sum(q1 * q2, axis=1))
            
            ori_err_dq = quat_distance(q_d, q_q)
            ori_err_dt = quat_distance(q_d, q_t)
            ori_err_qt = quat_distance(q_q, q_t)
            
            # 绘制姿态误差
            self.ax_ori_error.plot(time_data, ori_err_dq, 'r-', label='Desired-Joint Error')
            self.ax_ori_error.plot(time_data, ori_err_dt, 'g-', label='Desired-Tag Error')
            self.ax_ori_error.plot(time_data, ori_err_qt, 'b-', label='Joint-Tag Error')
            self.ax_ori_error.set_title('Orientation Error (Quaternion Distance)')
            self.ax_ori_error.set_xlabel('Time (s)')
            self.ax_ori_error.set_ylabel('Error')
            self.ax_ori_error.grid(True)
            self.ax_ori_error.legend()
            
            # 更新画布
            self.fig_pos_error.tight_layout()
            self.canvas_pos_error.draw()
            
            self.fig_ori_error.tight_layout()
            self.canvas_ori_error.draw()
            
        except Exception as e:
            self.error_text.append(f"Update plots error: {str(e)}")

    def update_validation_text(self):
        """更新验证文本显示"""
        try:
            if len(self.validation_data['time']) < 1:
                return
            
            # 获取最新数据
            p_d = self.validation_data['p_d'][-1]
            p_q = self.validation_data['p_q'][-1]
            p_t = self.validation_data['p_t'][-1]
            
            # 计算误差
            err_dq = np.linalg.norm(p_d - p_q)
            err_dt = np.linalg.norm(p_d - p_t)
            err_qt = np.linalg.norm(p_q - p_t)
            
            # 更新文本
            self.error_text.append(f"Current position error (m): Desired-Joint={err_dq:.4f}, "
                                 f"Desired-Tag={err_dt:.4f}, Joint-Tag={err_qt:.4f}")
            
        except Exception as e:
            self.error_text.append(f"Update text error: {str(e)}")

    def show_validation_statistics(self):
        """显示验证统计结果"""
        try:
            if len(self.validation_data['time']) < 2:
                self.error_text.append("Not enough data to calculate statistics")
                return
            
            # 转换为numpy数组
            p_d = np.array(self.validation_data['p_d'])
            p_q = np.array(self.validation_data['p_q'])
            p_t = np.array(self.validation_data['p_t'])
            
            # 计算位置误差
            err_dq = np.linalg.norm(p_d - p_q, axis=1)
            err_dt = np.linalg.norm(p_d - p_t, axis=1)
            err_qt = np.linalg.norm(p_q - p_t, axis=1)
            
            # 计算姿态误差
            q_d = np.array(self.validation_data['q_d'])
            q_q = np.array(self.validation_data['q_q'])
            q_t = np.array(self.validation_data['q_t'])
            
            # 计算四元数距离
            def quat_distance(q1, q2):
                return 1 - np.abs(np.sum(q1 * q2, axis=1))
            
            ori_err_dq = quat_distance(q_d, q_q)
            ori_err_dt = quat_distance(q_d, q_t)
            ori_err_qt = quat_distance(q_q, q_t)
            
            # 计算统计量
            stats = {
                'pos_dq_mean': np.mean(err_dq),
                'pos_dq_std': np.std(err_dq),
                'pos_dt_mean': np.mean(err_dt),
                'pos_dt_std': np.std(err_dt),
                'pos_qt_mean': np.mean(err_qt),
                'pos_qt_std': np.std(err_qt),
                'ori_dq_mean': np.mean(ori_err_dq),
                'ori_dq_std': np.std(ori_err_dq),
                'ori_dt_mean': np.mean(ori_err_dt),
                'ori_dt_std': np.std(ori_err_dt),
                'ori_qt_mean': np.mean(ori_err_qt),
                'ori_qt_std': np.std(ori_err_qt)
            }
            
            # 显示统计结果
            result_text = "Validation statistics:\n\n"
            result_text += "Position error (m):\n"
            result_text += f"Desired-Joint: mean={stats['pos_dq_mean']:.4f} ± {stats['pos_dq_std']:.4f}\n"
            result_text += f"Desired-Tag: mean={stats['pos_dt_mean']:.4f} ± {stats['pos_dt_std']:.4f}\n"
            result_text += f"Joint-Tag: mean={stats['pos_qt_mean']:.4f} ± {stats['pos_qt_std']:.4f}\n\n"
            
            result_text += "Orientation error (Quaternion Distance):\n"
            result_text += f"Desired-Joint: mean={stats['ori_dq_mean']:.4f} ± {stats['ori_dq_std']:.4f}\n"
            result_text += f"Desired-Tag: mean={stats['ori_dt_mean']:.4f} ± {stats['ori_dt_std']:.4f}\n"
            result_text += f"Joint-Tag: mean={stats['ori_qt_mean']:.4f} ± {stats['ori_qt_std']:.4f}"
            
            self.error_text.clear()
            self.error_text.append(result_text)
            
        except Exception as e:
            self.error_text.append(f"Calculate statistics error: {str(e)}")


if __name__ == "__main__":
    # parse args
    parser = argparse.ArgumentParser(description='arm_cali')
    parser.add_argument('--size', type=int, default=10, help='sample size')
    parser.add_argument('--real', action='store_true', help='real mode')
    parser.add_argument('--plot_size', type=int, default=1, help='plot size')
    args = parser.parse_args()
    
    # ctrl c
    import signal
    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)


    T_et = np.eye(4)
    quaternion = tf_trans.quaternion_from_euler(-np.pi/2, 0, -np.pi/2, 'sxyz')
    x,y,z,w = quaternion
    quat = np.array([w, x, y, z])
    T_et[:3, :3] = quat_to_rot(quat)
    T_et[:3, 3] = np.array([0.0, 0.0, -0.0])
    # ros
    rospy.init_node('arm_cali', anonymous=True)
    
    # 创建单个实例，默认为左手
    arm_cali_node = ArmCalibrator(side='l', T_et=T_et, real_mode=args.real, plot_size=args.plot_size)
    arm_cali_node.run()

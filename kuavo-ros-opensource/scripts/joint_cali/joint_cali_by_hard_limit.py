#!/opt/miniconda3/envs/joint_cali/bin/python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QSlider, QLabel, QGroupBox, QProgressBar, QPushButton, QMessageBox)
from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtGui import QColor, QPalette
import sys
import numpy as np
from identifiability_analyzer import ArmKinematics, get_package_path
import os
import rospy
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import sensorsData
import copy
from kuavo_msgs.srv import changeArmCtrlMode
from std_msgs.msg import Float64MultiArray

class SignalRelay(QObject):
    update_signal = pyqtSignal(dict)
    shutdown_signal = pyqtSignal()

class JointCaliUI(QMainWindow):
    def __init__(self, env):
        super().__init__()
        self.is_ui_valid = True
        robot_version = os.environ.get('ROBOT_VERSION', '40')
        self.setWindowTitle(f'V{robot_version}机器人关节标定')
        
        # 初始化运动学
        self.init_kinematics(robot_version)
        self.joint_upper_limits, self.joint_lower_limits = self.get_joint_limits()
        bias = 10*np.ones(14)
        # self.joint_upper_limits = self.joint_upper_limits + bias
        # self.joint_lower_limits = self.joint_lower_limits - bias
        
        # 标定相关变量初始化
        self.cali_active = False
        self.cali_joint_index = -1
        self.cali_compensated_torque = 0.0
        self.cali_current_position = 0.0
        self.joint_pos_measured = np.zeros(14)
        
        # 初始化UI
        self.sliders = []
        self.angle_labels = []
        self.torque_labels = []
        self.comp_bars = []
        self.comp_labels = []
        self.init_ui()
        
        # ROS初始化
        self.env = env
        self.init_ros_components()
        
        # 信号中继
        self.signal_relay = SignalRelay()
        self.signal_relay.update_signal.connect(self.safe_update_ui)
        self.signal_relay.shutdown_signal.connect(self.cleanup_resources)

    def init_kinematics(self, robot_version):
        asset_path = get_package_path("kuavo_assets")
        urdf_path = os.path.join(asset_path, f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf")
        T_et = np.eye(4)
        R_mod = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float32)
        T_et[:3, :3] = R_mod
        T_et[:3, 3] = np.array([0.12, 0.0, -0.06+0.02])
        self.arm_kinematics = ArmKinematics(urdf_path, T_et)

    def get_joint_limits(self):
        joint_upper = []
        joint_lower = []
        
        # 左臂限制
        robot_l = self.arm_kinematics.robot['l']
        for name in robot_l.model.names[1:]:
            jid = robot_l.model.getJointId(name)
            joint_upper.append(np.rad2deg(robot_l.model.upperPositionLimit[jid-1]))
            joint_lower.append(np.rad2deg(robot_l.model.lowerPositionLimit[jid-1]))
        
        # 右臂限制
        robot_r = self.arm_kinematics.robot['r']
        for name in robot_r.model.names[1:]:
            jid = robot_r.model.getJointId(name)
            joint_upper.append(np.rad2deg(robot_r.model.upperPositionLimit[jid-1]))
            joint_lower.append(np.rad2deg(robot_r.model.lowerPositionLimit[jid-1]))
        
        return np.array(joint_upper), np.array(joint_lower)

    def init_ros_components(self):
        rospy.init_node('joint_cali_ui', anonymous=True)
        self.joint_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=5)
        
        # 初始化关节状态
        self.joint_state = JointState()
        self.joint_state.name = [f'joint_{i+1}' for i in range(14)]
        self.joint_state.position = [0.0]*14
        
        # 非真实模式订阅
        if self.env == 'real':
            self.q_sub_real = rospy.Subscriber("/sensor_data_motor/motor_pos", Float64MultiArray, self.real_sensor_data_callback)
        elif self.env == 'mujoco':
            self.q_sub_mujoco = rospy.Subscriber("/sensors_data_raw", sensorsData, 
                                        self.sensor_callback,
                                        queue_size=5)
        elif self.env == 'gazebo':
            self.q_sub_gazebo = rospy.Subscriber("/share_memory/sensor_data_raw", sensorsData, 
                                        self.sensor_callback,
                                        queue_size=5)
        else:
            raise ValueError(f"Invalid environment: {self.env}")
        
        # 安全定时器
        self.update_timer = rospy.Timer(rospy.Duration(0.2), self.trigger_update)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # 左右臂控制面板
        left_panel = self.create_arm_panel('Left', 0)
        right_panel = self.create_arm_panel('Right', 7)
        
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel)

        # 添加控制按钮
        button_layout = QVBoxLayout()  # 创建一个新的垂直布局
        
        # 添加 enable 按钮
        enable_button = QPushButton("Enable")
        enable_button.clicked.connect(self.enable_arm_control)
        button_layout.addWidget(enable_button)

        # 添加 disable 按钮
        disable_button = QPushButton("Disable")
        disable_button.clicked.connect(self.disable_arm_control)
        button_layout.addWidget(disable_button)
        
        # 添加一键标定按钮
        auto_cali_button = QPushButton("Auto Calibration")
        auto_cali_button.clicked.connect(self.auto_calibrate_all_joints)
        button_layout.addWidget(auto_cali_button)

        main_layout.addLayout(button_layout)  # 将垂直布局添加到主布局中

        self.setMinimumSize(1200, 400)

    def create_arm_panel(self, side, start_idx):
        panel = QGroupBox(f"{side} Arm Control")
        layout = QVBoxLayout()
        
        for i in range(7):
            joint_idx = start_idx + i
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(self.joint_lower_limits[joint_idx]))
            slider.setMaximum(int(self.joint_upper_limits[joint_idx]))
            
            # 组件初始化
            slider_layout = QHBoxLayout()
            label = QLabel(f"Joint {joint_idx+1}:")
            value_label = QLabel("0°")
            torque_label = QLabel("Torque: 0.0")
            
            # 添加 cali 按钮
            cali_button = QPushButton("Cali")
            cali_button.clicked.connect(lambda checked=False, idx=joint_idx: self.calibrate_joint(idx))
            
            # 补偿扭矩显示
            comp_bar = QProgressBar()
            comp_bar.setRange(-100, 100)
            comp_bar.setTextVisible(False)
            comp_bar.setFixedSize(100, 15)
            comp_bar.setStyleSheet("""
                QProgressBar {
                    border: 1px solid #888;
                    background: #EEE;
                    border-radius: 3px;
                }
                QProgressBar::chunk {
                    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 #FF4444, stop:0.5 #FFDD33, stop:1 #44FF44);
                }
            """)
            comp_label = QLabel("Comp: 0.0")
            
            # 布局组装
            slider_layout.addWidget(cali_button)  # 添加 cali 按钮
            slider_layout.addWidget(label)
            slider_layout.addWidget(slider)
            slider_layout.addWidget(value_label)
            slider_layout.addWidget(torque_label)
            slider_layout.addWidget(comp_bar)
            slider_layout.addWidget(comp_label)
            layout.addLayout(slider_layout)
            
            # 保存引用
            self.sliders.append(slider)
            self.angle_labels.append(value_label)
            self.torque_labels.append(torque_label)
            self.comp_bars.append(comp_bar)
            self.comp_labels.append(comp_label)
            
            # 信号连接
            slider.valueChanged.connect(lambda v, idx=joint_idx: 
                                       self.on_slider_change(idx, v))
        
        panel.setLayout(layout)
        return panel

    def on_slider_change(self, index, value):
        if 0 <= index < 14:
            self.joint_state.position[index] = value
            self.angle_labels[index].setText(f"{value}°")
            self.joint_pub.publish(self.joint_state)

    def sensor_callback(self, msg):
        if not self.is_ui_valid:
            return
        
        try:
            # 数据准备
            update_data = {
                'positions': [0.0]*14,
                'torques': [0.0]*14,
                'compensated': [0.0]*14
            }
            
            # 计算重力补偿
            q_left = np.deg2rad([self.joint_state.position[i] for i in range(7)])
            q_right = np.deg2rad([self.joint_state.position[i] for i in range(7,14)])
            
            tau_left = self.arm_kinematics.gravityBiasTorque(q_left, 'l')
            tau_right = self.arm_kinematics.gravityBiasTorque(q_right, 'r')
            
            # 合并数据
            n_leg = 12
            for i in range(14):
                if i < len(msg.joint_data.joint_current):
                    raw_pos = msg.joint_data.joint_q[n_leg + i]
                    raw_pos = np.rad2deg(raw_pos)
                    raw_torque = msg.joint_data.joint_current[n_leg + i]
                    comp_torque = raw_torque - (tau_left[i] if i <7 else tau_right[i-7])
                    
                    self.joint_pos_measured[i] = raw_pos
                    update_data['positions'][i] = raw_pos
                    update_data['torques'][i] = raw_torque
                    update_data['compensated'][i] = comp_torque
                    
                    # 支持标定过程
                    if hasattr(self, 'cali_active') and self.cali_active and i == self.cali_joint_index:
                        self.cali_compensated_torque = comp_torque
                        self.cali_current_position = raw_pos
            
            # 发送更新信号
            self.signal_relay.update_signal.emit(copy.deepcopy(update_data))
            
        except Exception as e:
            rospy.logerr(f"传感器回调错误: {str(e)}")

    def trigger_update(self, event):
        if rospy.is_shutdown() or not self.is_ui_valid:
            return

    def safe_update_ui(self, data):
        if not self.is_ui_valid or not all([
            len(self.torque_labels) >=14,
            len(self.comp_bars) >=14,
            len(self.comp_labels) >=14
        ]):
            return
        
        try:
            for i in range(14):
                # 扭矩显示
                self.torque_labels[i].setText(f"Torque: {data['torques'][i]:.2f}")
                
                # 补偿值处理
                comp_value = data['compensated'][i]
                self.comp_labels[i].setText(f"Comp: {comp_value:.2f}")
                
                # 进度条更新
                bar_value = max(-100, min(100, int(comp_value * 10)))
                self.comp_bars[i].setValue(bar_value)
                
                # 颜色设置
                palette = QPalette()
                if abs(comp_value) < 1.0:
                    palette.setColor(QPalette.Highlight, QColor("#44FF44"))
                elif abs(comp_value) < 3.0:
                    palette.setColor(QPalette.Highlight, QColor("#FFDD33"))
                else:
                    palette.setColor(QPalette.Highlight, QColor("#FF4444"))
                self.comp_bars[i].setPalette(palette)
                
        except IndexError as ie:
            rospy.logwarn(f"UI更新索引错误: {str(ie)}")
        except Exception as e:
            rospy.logerr(f"UI更新失败: {str(e)}")

    def cleanup_resources(self):
        # ROS资源清理
        if rospy.is_initialized():
            self.update_timer.shutdown()
            self.joint_pub.unregister()
            if hasattr(self, 'q_sub'):
                self.q_sub.unregister()
            rospy.signal_shutdown("应用关闭")
        
        # Qt对象清理
        for comp in [self.signal_relay] + self.comp_bars:
            try:
                comp.deleteLater()
            except:
                pass
        
        self.is_ui_valid = False

    def closeEvent(self, event):
        self.is_ui_valid = False
        self.signal_relay.shutdown_signal.emit()
        event.accept()

    def enable_arm_control(self):
        self.change_arm_control_mode(True)

    def disable_arm_control(self):
        self.change_arm_control_mode(False)
        for i in range(14):
            self.joint_state.position[i] = 0.0
        self.joint_pub.publish(self.joint_state)

    def change_arm_control_mode(self, enable):
        try:
            # 调用 ROS 服务以使能机械臂控制
            rospy.wait_for_service('/humanoid_change_arm_ctrl_mode')
            change_mode_service = rospy.ServiceProxy('/humanoid_change_arm_ctrl_mode', changeArmCtrlMode)
            change_mode_service(2 if enable else 1)  # 设置控制模式为 2
            
            rospy.wait_for_service('/enable_wbc_arm_trajectory_control')
            enable_service = rospy.ServiceProxy('/enable_wbc_arm_trajectory_control', changeArmCtrlMode)
            enable_service(1 if enable else 0)  # 设置控制模式为 1
            
            rospy.loginfo("机械臂控制已使能")
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {str(e)}")

    def calibrate_joint(self, joint_index):
        rospy.loginfo(f"标定关节 {joint_index}")
        # define move orientation
        # 1->positive, -1->negative
        move_l = [1, 1, 1, 1, 1, 1, 1]
        move_r = [1, -1, 1, 1, 1, -1, 1]
        move_order = [*move_l, *move_r]
        
        # 设置扭矩阈值
        TORQUE_THRESHOLD = 2.0  # 可调参数
        # 设置移动速度和步长
        STEP_SIZE = 1.0  # 度
        SLEEP_TIME = 0.1  # 秒
        
        # 确定运动方向
        direction = move_order[joint_index]
        
        bias = 20.0 * np.ones(14)
        # 确定目标位置（关节限位）
        if direction > 0:
            target_position = self.joint_upper_limits[joint_index] + bias[joint_index]
            limit_value = self.joint_upper_limits[joint_index]
        else:
            target_position = self.joint_lower_limits[joint_index] - bias[joint_index]
            limit_value = self.joint_lower_limits[joint_index]
        
        rospy.loginfo(f"关节 {joint_index} 将向{'正' if direction > 0 else '负'}方向移动到限位")
        
        # 保存当前位置
        current_position = self.joint_state.position[joint_index]
        
        # 标志变量
        is_calibrating = True
        reached_limit = False
        last_comp_torque = 0.0
        
        # 创建一个订阅标志变量
        self.cali_active = True
        self.cali_joint_index = joint_index
        self.cali_compensated_torque = 0.0
        self.cali_current_position = 0.0
        
        try:
            while is_calibrating and not rospy.is_shutdown():
                # 移动一小步
                current_position += direction * STEP_SIZE
                
                # 确保不超过限制
                if direction > 0:
                    current_position = min(current_position, target_position)
                else:
                    current_position = max(current_position, target_position)
                
                # 更新关节状态并发布
                self.joint_state.position[joint_index] = current_position
                self.joint_pub.publish(self.joint_state)
                
                # 更新UI
                self.sliders[joint_index].setValue(int(current_position))
                
                # 等待一段时间以观察扭矩
                rospy.sleep(SLEEP_TIME)
                
                # 检查补偿后的扭矩
                comp_torque = abs(self.cali_compensated_torque)
                
                rospy.loginfo(f"设定位置: {target_position:.2f} deg, 当前位置: {self.cali_current_position:.2f} deg, 补偿扭矩: {comp_torque:.2f} Nm")
                
                # 判断是否达到限位
                if comp_torque > TORQUE_THRESHOLD:
                    rospy.loginfo(f"检测到扭矩超过阈值: {comp_torque:.2f} > {TORQUE_THRESHOLD}")
                    reached_limit = True
                    is_calibrating = False
                
                # 判断是否已到达极限位置
                if (direction > 0 and current_position >= target_position) or \
                   (direction < 0 and current_position <= target_position):
                    rospy.loginfo("已到达设定的极限位置")
                    is_calibrating = False
            
            # 标定完成后计算误差
            if reached_limit:
                actual_position = self.cali_current_position
                position_error = actual_position - limit_value
                rospy.loginfo(f"标定完成: 理论限位: {limit_value:.2f}, 实际检测位置: {actual_position:.2f}")
                rospy.loginfo(f"位置误差: {position_error:.2f} 度")
                
                # 将结果保存到文件
                file_path = os.path.expanduser("~/joint_calibration_results.txt")
                with open(file_path, "a") as f:
                    f.write(f"关节 {joint_index} 标定结果:\n")
                    f.write(f"方向: {'正' if direction > 0 else '负'}\n")
                    f.write(f"理论限位: {limit_value:.2f}\n")
                    f.write(f"实际位置: {actual_position:.2f}\n")
                    f.write(f"误差: {position_error:.2f} 度\n")
                    f.write("----------------------------\n")
                
                rospy.loginfo(f"标定结果已保存到 {file_path}")
            else:
                rospy.logwarn("标定未检测到扭矩限位，请检查参数或重试")
        
        except Exception as e:
            rospy.logerr(f"标定过程出错: {str(e)}")
        
        finally:
            # 清理标定状态
            self.cali_active = False
            
            # 关节往反方向回退5度，避免长时间处于大扭矩状态
            try:
                rospy.loginfo("标定结束，关节回退中...")
                # 获取当前位置
                current_pos = self.joint_pos_measured[joint_index]
                # 计算安全位置（反方向移动10度）
                safe_position = current_pos - direction * 5.0
                
                # 确保安全位置在关节限制范围内
                safe_position = max(self.joint_lower_limits[joint_index], 
                                   min(self.joint_upper_limits[joint_index], safe_position))
                
                # 更新关节状态并发布
                self.joint_state.position[joint_index] = safe_position
                self.joint_pub.publish(self.joint_state)
                
                # 更新UI
                self.sliders[joint_index].setValue(int(safe_position))
                rospy.loginfo(f"关节已回退到安全位置: {safe_position:.2f}")
            except Exception as e:
                rospy.logerr(f"关节回退过程出错: {str(e)}")

    def auto_calibrate_all_joints(self):
        """一键标定所有关节"""
        # 询问用户确认
        reply = QMessageBox.question(self, 'Start Auto Calibration', 
                                     "Start auto calibration for all joints?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        
        if reply == QMessageBox.No:
            return
            
        # 设置标定顺序，可以根据需要调整
        calibration_order = list(range(14))  # 0-13，所有关节
        
        for joint_idx in calibration_order:
            # 检查是否被取消或ROS已关闭
            if rospy.is_shutdown():
                rospy.logwarn("calibration process interrupted")
                break
                
            rospy.loginfo(f"即将开始标定关节 {joint_idx+1}")
            # 等待用户确认继续
            reply = QMessageBox.question(self, f'Calibrate Joint {joint_idx+1}', 
                                        f"Prepare to calibrate joint {joint_idx+1}, click OK to start, cancel to skip",
                                        QMessageBox.Ok | QMessageBox.Cancel, QMessageBox.Ok)
            
            if reply == QMessageBox.Cancel:
                rospy.loginfo(f"Skip calibration for joint {joint_idx+1}")
                continue
                
            # 标定当前关节
            try:
                self.calibrate_joint(joint_idx)
            except Exception as e:
                rospy.logerr(f"Error calibrating joint {joint_idx+1}: {str(e)}")
                reply = QMessageBox.question(self, 'Error', 
                                            f"Error calibrating joint {joint_idx+1}, continue calibration?",
                                            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if reply == QMessageBox.No:
                    break
            
            # 每个关节标定后短暂休息
            rospy.sleep(1.0)
        
        rospy.loginfo("All joints calibrated or canceled")
        QMessageBox.information(self, 'Completed', "Auto calibration process completed")

def main():
    app = QApplication(sys.argv)
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--env', type=str, default='real')
    args = parser.parse_args()
    
    try:
        window = JointCaliUI(args.env)
        window.show()
        ret = app.exec_()
        window.cleanup_resources()
        sys.exit(ret)
    except Exception as e:
        print(f"致命错误: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main()
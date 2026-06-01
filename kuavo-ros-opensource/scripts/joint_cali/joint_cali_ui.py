#!/opt/miniconda3/envs/joint_cali/bin/python
# -*- coding: utf-8 -*-

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QSlider, QLabel, QPushButton, QGroupBox,
                           QComboBox, QFileDialog)
from PyQt5.QtCore import Qt
import sys
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from identifiability_analyzer import ArmKinematics, get_package_path, identifiability_analyzer, create_objective_function
from arm_kinematics import HeadKinematics, quat_to_rot, rot_to_quat
import os
import matplotlib.pyplot as plt
import nlopt
import cyipopt
import json
from datetime import datetime

class JointCaliUI(QMainWindow):
    def __init__(self, true_bias=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])):
        super().__init__()
        self.setWindowTitle('关节标定数据采集模拟器')
        self.init_kinematics()
        # self.joint_upper_limits, self.joint_lower_limits = self.get_joint_limits()
        # self.joint_lower_limits = np.array([-100, -20, -80, -50, -20, -60, -40])
        # self.joint_upper_limits = np.array([-20, 30, 0, 0, 40, 20, 40])
        self.joint_lower_limits = np.array([-100, -20, -80, -50, -20, -60, -40])
        self.joint_upper_limits = np.array([+40,   30,  80,    0,  40,  20,  40])
        self.init_ui()
        self.collected_poses = []
        self.condition_numbers = []
        self.optimizer_type = 'nlopt_mma'  # 默认使用nlopt的LD_MMA算法
        self.true_bias = true_bias
        self.noise_std = 0.01  # 默认噪声标准差
        self.sample_count = 1  # 默认采样次数
        self.random_collect_count = 1  # 默认随机采集次数
        self.true_positions = []
        self.true_rotations = []

    def init_kinematics(self):
        # 初始化运动学对象
        asset_path = get_package_path("kuavo_assets")
        robot_version = os.environ.get('ROBOT_VERSION', '40')
        urdf_path = os.path.join(asset_path, f"models/biped_s{robot_version}/urdf/biped_s{robot_version}.urdf")
        
        T_et = np.eye(4)
        R_mod = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float32)
        T_et[:3, :3] = R_mod
        T_et[:3, 3] = np.array([0.12, 0.0, -0.06+0.02])
        self.arm_kinematics = ArmKinematics(urdf_path, T_et)
        self.head_fk = HeadKinematics(urdf_path).FK
        
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板
        control_panel = QGroupBox('Joint Angle Control')
        control_layout = QVBoxLayout()
        
        # 创建7个关节的滑动条
        self.sliders = []
        self.angle_labels = []
        for i in range(7):
            slider_layout = QHBoxLayout()
            label = QLabel(f'Joint {i+1}:')
            slider = QSlider(Qt.Horizontal)
            # 使用从URDF获取的实际关节限制
            slider.setMinimum(int(self.joint_lower_limits[i]))
            slider.setMaximum(int(self.joint_upper_limits[i]))
            slider.setValue(0)
            value_label = QLabel('0°')
            
            slider.valueChanged.connect(lambda v, label=value_label: label.setText(f'{v}°'))
            
            self.sliders.append(slider)
            self.angle_labels.append(value_label)
            
            slider_layout.addWidget(label)
            slider_layout.addWidget(slider)
            slider_layout.addWidget(value_label)
            control_layout.addLayout(slider_layout)
        
        # 添加噪声标准差控制
        noise_layout = QHBoxLayout()
        noise_label = QLabel('Gaussian Noise Std:')
        self.noise_slider = QSlider(Qt.Horizontal)
        self.noise_slider.setMinimum(0)
        self.noise_slider.setMaximum(100)
        self.noise_slider.setValue(100)  # 默认值0.01对应滑块值100
        self.noise_value_label = QLabel('0.01')
        
        def update_noise_value(value):
            std = value / 10000
            self.noise_std = std
            self.noise_value_label.setText(f'{std:.3f}')
        
        self.noise_slider.valueChanged.connect(update_noise_value)
        
        noise_layout.addWidget(noise_label)
        noise_layout.addWidget(self.noise_slider)
        noise_layout.addWidget(self.noise_value_label)
        control_layout.addLayout(noise_layout)
        
        # 添加采样次数控制
        sample_count_layout = QHBoxLayout()
        sample_count_label = QLabel('Sampling Times:')
        self.sample_count_slider = QSlider(Qt.Horizontal)
        self.sample_count_slider.setMinimum(1)
        self.sample_count_slider.setMaximum(10)
        self.sample_count_slider.setValue(1)  # 默认值1
        self.sample_count_value_label = QLabel('1')
        
        def update_sample_count(value):
            self.sample_count = value
            self.sample_count_value_label.setText(str(value))
        
        self.sample_count_slider.valueChanged.connect(update_sample_count)
        
        sample_count_layout.addWidget(sample_count_label)
        sample_count_layout.addWidget(self.sample_count_slider)
        sample_count_layout.addWidget(self.sample_count_value_label)
        control_layout.addLayout(sample_count_layout)
        
        # 添加随机采集次数控制
        random_collect_layout = QHBoxLayout()
        random_collect_label = QLabel('Random Collect Times:')
        self.random_collect_slider = QSlider(Qt.Horizontal)
        self.random_collect_slider.setMinimum(1)
        self.random_collect_slider.setMaximum(50)
        self.random_collect_slider.setValue(1)  # 默认值1
        self.random_collect_value_label = QLabel('1')
        
        def update_random_collect_count(value):
            self.random_collect_count = value
            self.random_collect_value_label.setText(str(value))
        
        self.random_collect_slider.valueChanged.connect(update_random_collect_count)
        
        random_collect_layout.addWidget(random_collect_label)
        random_collect_layout.addWidget(self.random_collect_slider)
        random_collect_layout.addWidget(self.random_collect_value_label)
        control_layout.addLayout(random_collect_layout)
        
        # 在按钮布局中添加导入/导出按钮
        io_layout = QHBoxLayout()
        
        self.export_btn = QPushButton('Export Data')
        self.export_btn.clicked.connect(self.export_data)
        io_layout.addWidget(self.export_btn)
        
        self.import_btn = QPushButton('Import Data')
        self.import_btn.clicked.connect(self.import_data)
        io_layout.addWidget(self.import_btn)
        
        control_layout.addLayout(io_layout)
        
        # 在按钮布局中添加优化器选择下拉框
        optimizer_layout = QHBoxLayout()
        optimizer_label = QLabel('Optimizer Type:')
        self.optimizer_combo = QComboBox()
        self.optimizer_combo.addItems([
            'nlopt_mma',    # NLopt LD_MMA算法
            'nlopt_lbfgs',  # NLopt LD_LBFGS算法
            'ipopt'         # IPOPT算法
        ])
        self.optimizer_combo.currentTextChanged.connect(self.on_optimizer_changed)
        optimizer_layout.addWidget(optimizer_label)
        optimizer_layout.addWidget(self.optimizer_combo)
        control_layout.addLayout(optimizer_layout)
        
        # 添加按钮布局
        button_layout = QHBoxLayout()
        
        # 添加随机位姿按钮
        random_btn = QPushButton('Random Pose')
        random_btn.clicked.connect(self.generate_random_pose)
        button_layout.addWidget(random_btn)
        
        # 添加采集按钮
        collect_btn = QPushButton('Collect Pose')
        collect_btn.clicked.connect(self.collect_pose)
        button_layout.addWidget(collect_btn)
        
        # 添加随机采集按钮
        random_collect_btn = QPushButton('Random Collect')
        random_collect_btn.clicked.connect(self.random_collect)
        button_layout.addWidget(random_collect_btn)
        
        # 添加标定按钮
        calibrate_btn = QPushButton('Calibrate')
        calibrate_btn.clicked.connect(self.calibrate)
        button_layout.addWidget(calibrate_btn)
        
        control_layout.addLayout(button_layout)
        
        control_panel.setLayout(control_layout)
        layout.addWidget(control_panel)
        
        # 右侧条件数图表
        plot_panel = QGroupBox('Condition Number')
        plot_layout = QVBoxLayout()
        self.figure = Figure(figsize=(6, 8))
        self.ax1 = self.figure.add_subplot(211)
        self.ax2 = self.figure.add_subplot(212)
        self.canvas = FigureCanvas(self.figure)
        plot_layout.addWidget(self.canvas)
        plot_panel.setLayout(plot_layout)
        layout.addWidget(plot_panel)
        
        # 最右侧灵敏度热图
        sensitivity_panel = QGroupBox('Joint Sensitivity')
        sensitivity_layout = QVBoxLayout()
        self.sensitivity_figure = Figure(figsize=(2, 8))
        self.sensitivity_ax = self.sensitivity_figure.add_subplot(111)
        self.sensitivity_canvas = FigureCanvas(self.sensitivity_figure)
        sensitivity_layout.addWidget(self.sensitivity_canvas)
        sensitivity_panel.setLayout(sensitivity_layout)
        layout.addWidget(sensitivity_panel)
        
        # 最右侧关节角度散点图
        joint_scatter_panel = QGroupBox('Joint Angle Distribution')
        joint_scatter_layout = QVBoxLayout()
        self.joint_scatter_figure = Figure(figsize=(6, 8))
        self.joint_scatter_ax = self.joint_scatter_figure.add_subplot(111)
        self.joint_scatter_canvas = FigureCanvas(self.joint_scatter_figure)
        joint_scatter_layout.addWidget(self.joint_scatter_canvas)
        joint_scatter_panel.setLayout(joint_scatter_layout)
        layout.addWidget(joint_scatter_panel)
        
        # 添加标定误差图表
        calibration_panel = QGroupBox('Calibration Error')
        calibration_layout = QVBoxLayout()
        self.calibration_figure = Figure(figsize=(6, 4))
        self.calibration_ax = self.calibration_figure.add_subplot(111)
        self.calibration_canvas = FigureCanvas(self.calibration_figure)
        calibration_layout.addWidget(self.calibration_canvas)
        calibration_panel.setLayout(calibration_layout)
        layout.addWidget(calibration_panel)
        
        self.figure.tight_layout(pad=3.0)
        self.sensitivity_figure.tight_layout(pad=3.0)
        self.joint_scatter_figure.tight_layout(pad=3.0)
        self.calibration_figure.tight_layout(pad=3.0)
        
        self.setMinimumSize(1400, 800)  # 增加窗口宽度以适应新图表
        
    def update_sensitivity_plot(self):
        """更新灵敏度热图"""
        if not self.collected_poses:
            return
            
        analyzer = identifiability_analyzer(self.arm_kinematics.FK_l, self.collected_poses)
        s_list, U_list, Vt_list = analyzer.get_singular_values_list()
        s = s_list[0]
        Vt = Vt_list[0]
        
        # 计算归一化的灵敏度
        sensitivity_ratios = analyzer.parameter_sensitivity_ratio(s, Vt, 1e-12)
        normalized_ratios = sensitivity_ratios/max(sensitivity_ratios)
        
        # 清除旧图
        self.sensitivity_ax.clear()
        
        # 创建热图数据
        data = normalized_ratios.reshape(-1, 1)
        
        # 绘制热图
        im = self.sensitivity_ax.imshow(data, aspect='auto', cmap='YlOrRd')
        
        # 设置y轴标签
        self.sensitivity_ax.set_yticks(range(7))
        self.sensitivity_ax.set_yticklabels([f'Joint {i+1}' for i in range(7)])
        
        # 在每个格子中显示具体数值
        for i in range(7):
            self.sensitivity_ax.text(0, i, f'{data[i][0]:.3f}', 
                                  ha='center', va='center')
        
        # 移除x轴刻度
        self.sensitivity_ax.set_xticks([])
        
        self.sensitivity_ax.set_title('Normalized\nSensitivity')
        self.sensitivity_figure.tight_layout()
        self.sensitivity_canvas.draw()

    def update_joint_scatter_plot(self):
        """更新关节角度散点图"""
        if not self.collected_poses:
            return
            
        self.joint_scatter_ax.clear()
        
        # 将所有采集的关节角度数据转换为度数
        joint_angles = np.rad2deg(np.array(self.collected_poses))
        
        # 为每个采集点生成不同的颜色
        colors = plt.cm.rainbow(np.linspace(0, 1, len(joint_angles)))
        
        # 为每个关节绘制散点
        for i in range(7):
            # 添加轻微的水平随机偏移使重叠的点可见
            x = i + np.random.normal(0, 0.1, size=len(joint_angles))
            self.joint_scatter_ax.scatter(x, joint_angles[:, i], 
                                        c=colors, alpha=0.6, 
                                        label=f'Joint {i+1}')
            
        # 添加关节限制范围
        for i in range(7):
            self.joint_scatter_ax.hlines([self.joint_lower_limits[i], self.joint_upper_limits[i]], 
                                       i-0.3, i+0.3, 
                                       colors='red', linestyles='dashed', alpha=0.5)
        
        self.joint_scatter_ax.set_xlabel('Joint Index')
        self.joint_scatter_ax.set_ylabel('Joint Angle (deg)')
        self.joint_scatter_ax.set_title('Joint Angle Distribution')
        
        # 设置x轴刻度
        self.joint_scatter_ax.set_xticks(range(7))
        self.joint_scatter_ax.set_xticklabels([f'J{i+1}' for i in range(7)])
        
        # 设置y轴范围略大于限制范围
        max_limit = max(self.joint_upper_limits) * 1.1
        min_limit = min(self.joint_lower_limits) * 1.1
        self.joint_scatter_ax.set_ylim(min_limit, max_limit)
        
        # 添加网格
        self.joint_scatter_ax.grid(True, linestyle='--', alpha=0.3)
        
        # 添加零线
        self.joint_scatter_ax.axhline(y=0, color='k', linestyle='-', alpha=0.2)
        
        self.joint_scatter_figure.tight_layout()
        self.joint_scatter_canvas.draw()

    def collect_pose(self):
        """Collect current pose and generate corresponding true data"""
        # Get current joint angles
        current_pose = np.array([np.deg2rad(slider.value()) for slider in self.sliders])
        
        # 根据采样次数重复采集数据
        for _ in range(self.sample_count):
            self.collected_poses.append(current_pose)
            
            # Generate true data with a fixed bias
            pos, rot, _ = self.arm_kinematics.FK_l(current_pose + self.true_bias)
            noisy_pos = pos + np.random.normal(0.0, self.noise_std, 3)
            
            # Store true data
            if not hasattr(self, 'true_positions'):
                self.true_positions = []
                self.true_rotations = []
            self.true_positions.append(noisy_pos)
            self.true_rotations.append(rot)
            
            # Calculate condition number
            analyzer = identifiability_analyzer(self.arm_kinematics.FK_l, self.collected_poses)
            cond_num = analyzer.get_cond_num_i()
            self.condition_numbers.append(cond_num)
        
        # Update all plots
        self.update_plot()
        self.update_sensitivity_plot()
        self.update_joint_scatter_plot()

    def update_plot(self):
        self.figure.clear()
        
        # 上半部分：显示所有历史数据
        self.ax1 = self.figure.add_subplot(211)
        self.ax1.semilogy(range(1, len(self.condition_numbers) + 1), 
                     self.condition_numbers, 'b-', marker='o')
        self.ax1.set_xlabel('Collection Count')
        self.ax1.set_ylabel('Condition Number (log scale)')
        self.ax1.set_title('All Historical Condition Numbers')
        self.ax1.grid(True)
        
        # 下半部分：只显示最近5个数据
        self.ax2 = self.figure.add_subplot(212)
        recent_data = self.condition_numbers[-5:] if len(self.condition_numbers) > 5 else self.condition_numbers
        x_recent = range(max(1, len(self.condition_numbers) - 4), len(self.condition_numbers) + 1)
        
        # 绘制线条和点
        line = self.ax2.plot(x_recent, recent_data, 'r-', marker='o', linewidth=2)[0]
        
        # 在每个点上添加数值标签
        for x, y in zip(x_recent, recent_data):
            self.ax2.annotate(f'{y:.2e}',  # 使用科学计数法显示
                            (x, y),
                            textcoords="offset points",
                            xytext=(0, 10),  # 在点上方10个单位显示文本
                            ha='center',
                            fontsize=9)
        
        self.ax2.set_xlabel('Collection Count')
        self.ax2.set_ylabel('Condition Number')
        self.ax2.set_title('Recent 5 Condition Numbers')
        self.ax2.grid(True)
        
        # 如果数据点少于5个，调整x轴范围使图表更美观
        if len(recent_data) < 5:
            self.ax2.set_xlim(max(0, len(self.condition_numbers) - 4.5), 
                            len(self.condition_numbers) + 0.5)
        
        # 调整子图间距
        self.figure.tight_layout(pad=3.0)
        
        self.canvas.draw()

    def generate_random_pose(self):
        """生成随机位姿并更新滑动条"""
        # 为每个关节生成随机角度
        for i, slider in enumerate(self.sliders):
            random_angle = np.random.uniform(self.joint_lower_limits[i], 
                                          self.joint_upper_limits[i])
            slider.setValue(int(random_angle))

    def get_joint_limits(self):
        """从URDF文件中获取关节限制的上下限"""
        joint_upper_limits = []
        joint_lower_limits = []
        robot = self.arm_kinematics.robot['l']
        
        # 获取所有关节名称
        joint_names = robot.model.names[1:]  # 跳过第一个，通常是 'universe'
        
        # 遍历关节获取限制
        for joint_name in joint_names:
            joint_id = robot.model.getJointId(joint_name)
            # joint_id 是从1开始的，但数组索引从0开始，所以需要减1
            upper_limit = np.rad2deg(robot.model.upperPositionLimit[joint_id - 1])
            lower_limit = np.rad2deg(robot.model.lowerPositionLimit[joint_id - 1])
            joint_upper_limits.append(upper_limit)
            joint_lower_limits.append(lower_limit)
            
        self.joint_upper_limits = np.array(joint_upper_limits)
        self.joint_lower_limits = np.array(joint_lower_limits)
        return self.joint_upper_limits, self.joint_lower_limits

    def on_optimizer_changed(self, text):
        """当优化器选择改变时调用"""
        self.optimizer_type = text

    def calibrate(self):
        """Execute calibration and display errors"""
        if len(self.collected_poses) < 2:
            print("At least 2 collected points are needed for calibration")
            return
        
        # Prepare data for calibration
        data_dict = {
            'q': self.collected_poses,
            'true_pos': self.true_positions,
            'true_rot': self.true_rotations
        }
        
        # Get true bias for error comparison
        # true_bias = np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07])
        
        # Execute calibration with selected optimizer
        if self.optimizer_type == 'nlopt_mma':
            result = self.calibrate_with_nlopt(data_dict, nlopt.LD_MMA)
        elif self.optimizer_type == 'nlopt_lbfgs':
            result = self.calibrate_with_nlopt(data_dict, nlopt.LD_LBFGS)
        else:  # ipopt
            result = self.calibrate_with_ipopt(data_dict)
            
        if result is not None:
            self.update_calibration_error_plot(result, self.true_bias)

    def calibrate_with_nlopt(self, data_dict, algorithm):
        """使用nlopt进行标定
        
        Args:
            data_dict: 标定数据字典
            algorithm: nlopt优化算法类型（如nlopt.LD_MMA或nlopt.LD_LBFGS）
        """
        n_vars = 7
        opt = nlopt.opt(algorithm, n_vars)
        opt.set_lower_bounds([-np.deg2rad(15)]*n_vars)
        opt.set_upper_bounds([np.deg2rad(15)]*n_vars)
        
        objective = create_objective_function(self.arm_kinematics.FK_l, data_dict)
        opt.set_min_objective(objective)
        
        opt.set_ftol_rel(1e-8)
        opt.set_maxeval(1000)
        alg_name = 'MMA' if algorithm == nlopt.LD_MMA else 'LBFGS'
        
        try:
            initial_delta = np.zeros(n_vars)
            result = opt.optimize(initial_delta)
            final_error = opt.last_optimum_value()
            print(f"NLopt ({alg_name}) Final Error: {final_error}")
            print(f"NLopt ({alg_name}) Found Bias (rad): {result}")
            print(f"NLopt ({alg_name}) Found Bias (deg): {np.rad2deg(result)}")
            return result
        except Exception as e:
            print(f"NLopt ({alg_name}) Optimization Error: {e}")
            return None

    def calibrate_with_ipopt(self, data_dict):
        """使用ipopt进行标定"""
        class CalibrationProblem:
            def __init__(self, fk_func, data_dict):
                self.fk_func = fk_func
                self.data_dict = data_dict
                self.n_vars = 7
                
            def objective(self, x):
                total_error = 0
                for i, q in enumerate(self.data_dict['q']):
                    pos, rot, _ = self.fk_func(q + x)
                    true_pos = self.data_dict['true_pos'][i]
                    pos_error = np.linalg.norm(pos - true_pos)
                    total_error += pos_error
                return total_error
                
            def gradient(self, x):
                # 使用有限差分计算梯度
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
                
            def constraints(self, x):
                return np.array([])  # 无约束
                
            def jacobian(self, x):
                return np.array([])  # 无约束
                
            def intermediate(self, alg_mod, iter_count, obj_value, inf_pr, inf_du,
                           mu, d_norm, regularization_size, alpha_du, alpha_pr,
                           ls_trials):
                print(f"Iteration {iter_count}, objective value: {obj_value}")

        problem = CalibrationProblem(self.arm_kinematics.FK_l, data_dict)
        
        # 设置优化边界
        lb = [-np.deg2rad(15)] * 7
        ub = [np.deg2rad(15)] * 7
        
        x0 = np.zeros(7)  # 初始猜测值
        
        nlp = cyipopt.Problem(
            n=7,                    # 变量数量
            m=0,                    # 约束数量
            problem_obj=problem,    # 问题对象
            lb=lb,                  # 变量下界
            ub=ub,                  # 变量上界
        )
        
        # 设置求解器参数
        nlp.add_option('max_iter', 1000)
        nlp.add_option('tol', 1e-8)
        
        try:
            result = nlp.solve(x0)[0]
            print("Ipopt found bias (rad):", result)
            print("Ipopt found bias (deg):", np.rad2deg(result))
            return result
        except Exception as e:
            print("Ipopt optimization error:", e)
            return None

    def update_calibration_error_plot(self, estimated_bias, true_bias):
        """更新标定误差图表"""
        self.calibration_ax.clear()
        
        # 转换为角度显示
        estimated_bias_deg = np.rad2deg(estimated_bias)
        true_bias_deg = np.rad2deg(true_bias)
        errors_deg = np.abs(estimated_bias_deg - true_bias_deg)
        # 创建条形图
        joint_indices = np.arange(len(estimated_bias_deg))
        bars = self.calibration_ax.bar(joint_indices, errors_deg)
        
        # 在每个条形上添加具体数值
        for bar, error in zip(bars, errors_deg):
            height = bar.get_height()
            self.calibration_ax.text(bar.get_x() + bar.get_width()/2., height,
                                   f'{abs(error):.3f}°',
                                   ha='center', va='bottom')
        
        # 设置图表属性
        self.calibration_ax.set_xlabel('Joint Index')
        self.calibration_ax.set_ylabel('Absolute Error (deg)')
        self.calibration_ax.set_title('Joint Calibration Errors')
        self.calibration_ax.set_xticks(joint_indices)
        self.calibration_ax.set_xticklabels([f'J{i+1}' for i in range(len(errors_deg))])
        
        # 添加网格
        self.calibration_ax.grid(True, linestyle='--', alpha=0.3)
        
        # 更新画布
        self.calibration_figure.tight_layout()
        self.calibration_canvas.draw()

    def export_data(self):
        """Export collected data to JSON file"""
        if not self.collected_poses:
            print("No data to export")
            return
            
        try:
            # Create data dictionary using stored true data
            data_dict = {
                'q': [q.tolist() for q in self.collected_poses],
                'true_pos': [pos.tolist() for pos in self.true_positions],
                'true_rot': [rot.tolist() for rot in self.true_rotations]
            }
            
            # Add metadata
            export_data = {
                'timestamp': datetime.now().strftime('%Y%m%d_%H%M%S'),
                'data': data_dict
            }
            
            # Open file dialog for saving
            filename, _ = QFileDialog.getSaveFileName(
                self,
                "Save Calibration Data",
                f'calibration_data_{export_data["timestamp"]}.json',
                "JSON Files (*.json)"
            )
            
            if not filename:  # User cancelled
                return
                
            # Save to file
            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=4)
            
            print(f"Data exported to {filename}")
            
        except Exception as e:
            print(f"Error exporting data: {e}")

    def import_data(self):
        """Import calibration data from JSON file"""
        try:
            # Open file dialog
            filename, _ = QFileDialog.getOpenFileName(
                self,
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
            required_keys = ['q', 'tag_pos', 'tag_ori']
            if not all(key in data_dict for key in required_keys):
                raise ValueError("Invalid data format: missing required keys")
            
            # Convert lists to numpy arrays and update all data
            for q, tag_pos, tag_ori in zip(data_dict['q'], data_dict['tag_pos'], data_dict['tag_ori']):
                self.collected_poses.append(np.array(q[12:12+7] - self.true_bias))
                q_head = q[-2:]
                p_bi, R_bi = self.head_fk(q_head)
                p_bt = p_bi + R_bi @ tag_pos
                R_bt = R_bi @ quat_to_rot(tag_ori)
                self.true_positions.append(np.array(p_bt))
                self.true_rotations.append(np.array(R_bt))
            
            # Recalculate condition numbers
            self.condition_numbers = []
            analyzer = identifiability_analyzer(self.arm_kinematics.FK_l, self.collected_poses)
            for i in range(2, len(self.collected_poses)):
                self.condition_numbers.append(analyzer.get_cond_num_i(i))
            
            # Update all plots
            self.update_plot()
            self.update_sensitivity_plot()
            self.update_joint_scatter_plot()
            
            print(f"Successfully imported {len(self.collected_poses)} data points")
            
        except Exception as e:
            print(f"Error importing data: {str(e)}")

    def random_collect(self):
        """执行多次随机位姿采集"""
        for _ in range(self.random_collect_count):
            self.generate_random_pose()  # 生成随机位姿
            self.collect_pose()          # 采集当前位姿

def main():
    app = QApplication(sys.argv)
    # true_bias = -np.array([0.05, -0.14, 0.13, 0.02, 0.01, -0.1, 0.15])
    true_bias = np.zeros(7)
    window = JointCaliUI(true_bias)
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 
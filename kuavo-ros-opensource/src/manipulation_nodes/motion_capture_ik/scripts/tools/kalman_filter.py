import numpy as np
import scipy

class KalmanFilter3D:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise, dt=0.01):
        """
        初始化卡尔曼滤波器
        :param initial_state: 初始状态 [x, y, z, vx, vy, vz]
        :param initial_covariance: 初始协方差矩阵
        :param process_noise: 过程噪声协方差矩阵
        :param measurement_noise: 测量噪声协方差矩阵
        :param dt: 时间步长
        """
        self.state = initial_state.astype(np.float64)  # 状态 [x, y, z, vx, vy, vz]
        self.covariance = initial_covariance
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.dt = dt

        # 状态转移矩阵 F（会随 dt 更新）
        self.F = self._construct_F(dt)

        # 测量矩阵 H：只测量位置
        self.H = np.hstack((np.eye(3), np.zeros((3, 3))))

    @staticmethod
    def _construct_F(dt):
        """构建状态转移矩阵 F"""
        F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        return F

    def predict(self):
        """预测步骤"""
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.process_noise

    def update(self, measurement):
        """
        更新步骤
        :param measurement: 测量值 [x, y, z]
        """
        y = measurement - self.H @ self.state
        S = self.H @ self.covariance @ self.H.T + self.measurement_noise
        K = self.covariance @ self.H.T @ scipy.linalg.inv(S)
        self.state = self.state + K @ y
        self.covariance = (np.eye(6) - K @ self.H) @ self.covariance

    def filter(self, measurement):
        """
        执行一次滤波
        :param measurement: 测量值 [x, y, z]
        :return: 滤波后的状态 [x, y, z]
        """
        self.predict()
        self.update(measurement)
        return self.state[:3]

    def get_velocity(self):
        """返回当前速度估计 [vx, vy, vz]"""
        return self.state[3:]

    def get_state(self):
        """返回完整状态 [x, y, z, vx, vy, vz]"""
        return self.state.copy()

    def predict_future_pos(self, time_ahead):
        transMat = self._construct_F(time_ahead)
        state_future = transMat @ self.state
        return state_future[:3]

class ESKFQuaternion:
    def __init__(self, q0, w0, P0, Q, R_meas, dt):
        """
        ESKF with quaternion and angular velocity
        :param q0: initial quaternion [w, x, y, z]
        :param w0: initial angular velocity [wx, wy, wz]
        :param P0: initial covariance (6x6)
        :param Q: process noise (6x6)
        :param R_meas: measurement noise (3x3) for quaternion residual
        :param dt: timestep
        """
        self.q = self._normalize(q0)
        self.w = np.array(w0, dtype=float)
        self.P = P0
        self.Q = Q
        self.R = R_meas
        self.dt = dt

    def _normalize(self, q):
        return q / np.linalg.norm(q)

    def _quat_from_omega(self, omega, dt):
        theta = np.linalg.norm(omega) * dt
        if theta < 1e-8:
            return np.array([1, 0, 0, 0])
        axis = omega / np.linalg.norm(omega)
        qw = np.cos(theta / 2)
        qx, qy, qz = axis * np.sin(theta / 2)
        return self._normalize(np.array([qw, qx, qy, qz]))

    def _quat_mult(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def _quat_conj(self, q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def _delta_theta_from_quat(self, q_meas, q_est):
        q_err = self._quat_mult(q_meas, self._quat_conj(q_est))
        if q_err[0] < 0:
            q_err = -q_err
        return 2.0 * q_err[1:4]  # small angle approximation

    def predict(self):
        dq = self._quat_from_omega(self.w, self.dt)
        self.q = self._quat_mult(dq, self.q)
        self.q = self._normalize(self.q)
        F = np.eye(6)
        F[:3, 3:] = self.dt * np.eye(3)
        self.P = F @ self.P @ F.T + self.Q

    def predict_future_quat(self, time_ahead):
        dq = self._quat_from_omega(self.w, time_ahead)
        q_future = self._quat_mult(dq, self.q)
        q_future = self._normalize(q_future)
        return q_future

    def update(self, q_meas):
        delta_theta = self._delta_theta_from_quat(q_meas, self.q)
        y = delta_theta
        H = np.hstack((np.eye(3), np.zeros((3, 3))))
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y
        delta_q = self._quat_from_omega(dx[:3], 1.0)
        self.q = self._quat_mult(delta_q, self.q)
        self.q = self._normalize(self.q)
        self.w += dx[3:]
        self.P = (np.eye(6) - K @ H) @ self.P

    def step(self, q_meas):
        self.predict()
        self.update(q_meas)
        return self.q


class TwoArmPosePredictor:
    def __init__(self, eef_pos_left, eef_pos_right, eef_quat_left, eef_quat_right, dt, process_noise_scale=0.1, measurement_noise_scale=0.01):
        # 位置滤波
        initial_state = np.array([0]*6, dtype=np.float64)  # 初始状态 [x, y, z, vx, vy, vz]
        initial_covariance = np.eye(6, dtype=np.float64)  # 初始协方差矩阵
        process_noise = np.eye(6, dtype=np.float64) * process_noise_scale  # 过程噪声协方差矩阵
        measurement_noise = np.eye(3, dtype=np.float64) * measurement_noise_scale  # 测量噪声协方差矩阵
        initial_state[0:3] = eef_pos_left
        self.pkf_left = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise, dt)
        initial_state[0:3] = eef_pos_right
        self.pkf_right = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise, dt)
        
        # 四元数滤波
        initial_covariance = np.eye(6) * process_noise_scale
        process_noise = np.eye(6) * process_noise_scale
        measurement_noise = np.eye(3) * measurement_noise_scale
        initial_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.qkf_left = ESKFQuaternion(eef_quat_left, initial_angular_velocity, 
                initial_covariance, process_noise, measurement_noise, dt)
        self.qkf_right = ESKFQuaternion(eef_quat_right, initial_angular_velocity, 
                initial_covariance, process_noise, measurement_noise, dt)
        
    def filter(self, eef_pos_left, eef_pos_right, eef_quat_left, eef_quat_right):
        pos_left = self.pkf_left.filter(eef_pos_left)
        pos_right = self.pkf_right.filter(eef_pos_right)
        quat_left = self.qkf_left.step(eef_quat_left)
        quat_right = self.qkf_right.step(eef_quat_right)
        return pos_left, pos_right, quat_left, quat_right
    
    def predict_next_pose(self, time_ahead=0.1):
        pos_left = self.pkf_left.predict_future_pos(time_ahead)
        pos_right = self.pkf_right.predict_future_pos(time_ahead)
        quat_left = self.qkf_left.predict_future_quat(time_ahead)
        quat_right = self.qkf_right.predict_future_quat(time_ahead)
        return pos_left, pos_right, quat_left, quat_right


def quaternion_multiply(q1, q2):
    """四元数乘法"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def test_quaternion_filter_with_plotting():
    """
    完整的四元数卡尔曼滤波测试，包含真值生成、噪声添加、滤波处理和绘图比较
    """
    import matplotlib.pyplot as plt
    import math
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 参数设置
    dt = 0.01  # 时间步长
    t_end = 10.0  # 总时间
    t = np.arange(0, t_end, dt)
    n_steps = len(t)
    
    # 生成真值四元数轨迹（复合旋转）
    true_quaternions = []
    for i, time in enumerate(t):
        # 绕X轴旋转（俯仰）
        pitch = 0.5 * math.sin(0.5 * time)
        q_pitch = np.array([math.cos(pitch/2), math.sin(pitch/2), 0, 0])
        
        # 绕Y轴旋转（偏航）
        yaw = 0.3 * math.sin(0.3 * time)
        q_yaw = np.array([math.cos(yaw/2), 0, math.sin(yaw/2), 0])
        
        # 绕Z轴旋转（翻滚）
        roll = 0.2 * math.sin(0.4 * time)
        q_roll = np.array([math.cos(roll/2), 0, 0, math.sin(roll/2)])
        
        # 复合旋转：q = q_pitch * q_yaw * q_roll
        q_temp = quaternion_multiply(q_pitch, q_yaw)
        q_true = quaternion_multiply(q_temp, q_roll)
        q_true = q_true / np.linalg.norm(q_true)  # 归一化
        true_quaternions.append(q_true)
    
    true_quaternions = np.array(true_quaternions)
    
    # 添加噪声生成测量值
    noise_std = 0.015  # 噪声标准差
    noisy_quaternions = []
    for q_true in true_quaternions:
        # 添加高斯噪声
        noise = np.random.normal(0, noise_std, 4)
        q_noisy = q_true + noise
        # 归一化
        q_noisy = q_noisy / np.linalg.norm(q_noisy)
        noisy_quaternions.append(q_noisy)
    
    noisy_quaternions = np.array(noisy_quaternions)
    
    # 初始化卡尔曼滤波器
    initial_quaternion = true_quaternions[0]
    initial_angular_velocity = np.array([0.0, 0.0, 0.0])
    initial_covariance = np.eye(6) * 0.1
    process_noise = np.eye(6) * 0.01
    # measurement_noise = np.eye(3) * (noise_std ** 2)
    measurement_noise = np.eye(3) * 1.0
    dt = 0.01
    
    kf = ESKFQuaternion(initial_quaternion, initial_angular_velocity, 
                    initial_covariance, process_noise, measurement_noise, dt)
    
    # 滤波处理
    filtered_quaternions = []
    predicted_quaternions = []
    for q_meas in noisy_quaternions:
        q_filtered = kf.step(q_meas)
        # print(f"q_filtered: {q_filtered}")
        # print(f"w_filtered: {w_filtered}")
        filtered_quaternions.append(q_filtered)
        predicted_quaternions.append(kf.predict_future_quat(0.5))
    
    filtered_quaternions = np.array(filtered_quaternions)
    predicted_quaternions = np.array(predicted_quaternions)
    # 计算欧拉角
    def quaternion_to_euler(q):
        w, x, y, z = q
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return np.array([roll, pitch, yaw])
    
    true_euler = np.array([quaternion_to_euler(q) for q in true_quaternions])
    noisy_euler = np.array([quaternion_to_euler(q) for q in noisy_quaternions])
    filtered_euler = np.array([quaternion_to_euler(q) for q in filtered_quaternions])
    predicted_euler = np.array([quaternion_to_euler(q) for q in predicted_quaternions])
    # 绘图
    fig, axes = plt.subplots(4, 2, figsize=(15, 16))
    fig.suptitle('kalman filter test', fontsize=16)
    
    # 四元数分量对比
    quat_labels = ['w', 'x', 'y', 'z']
    for i in range(4):
        ax = axes[i, 0]
        ax.plot(t, true_quaternions[:, i], 'g-', label='true', linewidth=2)
        ax.plot(t, noisy_quaternions[:, i], 'r.', label='noisy', markersize=2, alpha=0.6)
        ax.plot(t, filtered_quaternions[:, i], 'b-', label='filtered', linewidth=1.5)
        ax.plot(t, predicted_quaternions[:, i], 'y--', label='predicted', linewidth=1.5)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'quaternion {quat_labels[i]}')
        ax.set_title(f'quaternion {quat_labels[i]}')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # 欧拉角对比
    euler_labels = ['Roll', 'Pitch', 'Yaw']
    for i in range(3):
        ax = axes[i, 1]
        ax.plot(t, np.degrees(true_euler[:, i]), 'g-', label='true', linewidth=2)
        ax.plot(t, np.degrees(noisy_euler[:, i]), 'r.', label='noisy', markersize=2, alpha=0.6)
        ax.plot(t, np.degrees(filtered_euler[:, i]), 'b-', label='filtered', linewidth=1.5)
        ax.plot(t, np.degrees(predicted_euler[:, i]), 'y--', label='predicted', linewidth=1.5)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'{euler_labels[i]} (deg)')
        ax.set_title(f'{euler_labels[i]}')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # 隐藏最后一个子图
    axes[3, 1].set_visible(False)
    
    plt.tight_layout()
    plt.show()
    
    # 计算误差统计
    def quaternion_error(q1, q2):
        """计算四元数误差（角度）"""
        dot_product = np.clip(np.abs(np.dot(q1, q2)), -1, 1)
        return 2 * np.arccos(dot_product) * 180 / np.pi  # 转换为度
    
    # 计算平均误差
    noisy_errors = [quaternion_error(q_true, q_noisy) for q_true, q_noisy in zip(true_quaternions, noisy_quaternions)]
    filtered_errors = [quaternion_error(q_true, q_filtered) for q_true, q_filtered in zip(true_quaternions, filtered_quaternions)]
    
    print("="*60)
    print("四元数卡尔曼滤波测试结果统计")
    print("="*60)
    print(f"测试时长: {t_end} 秒")
    print(f"时间步长: {dt} 秒")
    print(f"噪声标准差: {noise_std}")
    print(f"带噪声测量平均误差: {np.mean(noisy_errors):.2f} 度")
    print(f"滤波后平均误差: {np.mean(filtered_errors):.2f} 度")
    print(f"误差改善: {np.mean(noisy_errors) - np.mean(filtered_errors):.2f} 度")
    print(f"改善比例: {((np.mean(noisy_errors) - np.mean(filtered_errors)) / np.mean(noisy_errors) * 100):.1f}%")
    print("="*60)

def test_position_filter_with_plotting():
    """
    完整的位置卡尔曼滤波测试，包含真值生成、噪声添加、滤波处理和绘图比较
    """
    import matplotlib.pyplot as plt
    import math
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 参数设置
    dt = 0.01  # 时间步长
    t_end = 10.0  # 总时间
    t = np.arange(0, t_end, dt)
    n_steps = len(t)
    
    # 生成真值位置轨迹（复合运动）
    true_positions = []
    true_velocities = []
    
    for i, time in enumerate(t):
        # X方向：正弦运动
        x = 2.0 * math.sin(0.5 * time)
        vx = 1.0 * math.cos(0.5 * time)
        
        # Y方向：余弦运动
        y = 1.5 * math.cos(0.3 * time)
        vy = -0.45 * math.sin(0.3 * time)
        
        # Z方向：线性运动
        z = 0.5 * time
        vz = 0.5
        
        true_positions.append(np.array([x, y, z]))
        true_velocities.append(np.array([vx, vy, vz]))
    
    true_positions = np.array(true_positions)
    true_velocities = np.array(true_velocities)
    
    # 添加噪声生成测量值
    noise_std = 0.1  # 位置噪声标准差
    noisy_positions = []
    for pos_true in true_positions:
        # 添加高斯噪声
        noise = np.random.normal(0, noise_std, 3)
        pos_noisy = pos_true + noise
        noisy_positions.append(pos_noisy)
    
    noisy_positions = np.array(noisy_positions)
    
    # 初始化卡尔曼滤波器
    initial_position = true_positions[0]
    initial_velocity = true_velocities[0]
    initial_state = np.concatenate([initial_position, initial_velocity])
    initial_covariance = np.eye(6) * 0.1
    process_noise = np.eye(6) * 0.01
    # measurement_noise = np.eye(3) * (noise_std ** 2)
    measurement_noise = np.eye(3) * 1.0
    
    kf = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise, dt)
    
    # 滤波处理
    filtered_positions = []
    filtered_velocities = []
    predicted_positions = []
    
    for i, pos_meas in enumerate(noisy_positions):
        # 滤波
        filtered_pos = kf.filter(pos_meas)
        filtered_vel = kf.get_velocity()
        
        filtered_positions.append(filtered_pos)
        filtered_velocities.append(filtered_vel)
        
        # 预测未来0.5秒的位置
        predicted_pos = kf.predict_future_pos(0.5)
        predicted_positions.append(predicted_pos)
        
        if i < 5:  # 只打印前几次的调试信息
            print(f"Step {i}: 测量={pos_meas}, 滤波位置={filtered_pos}, 滤波速度={filtered_vel}")
    
    filtered_positions = np.array(filtered_positions)
    filtered_velocities = np.array(filtered_velocities)
    predicted_positions = np.array(predicted_positions)
    
    # 绘图
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('position kalman filter test', fontsize=16)
    
    # 位置对比
    pos_labels = ['X', 'Y', 'Z']
    for i in range(3):
        ax = axes[i, 0]
        ax.plot(t, true_positions[:, i], 'g-', label='true', linewidth=2)
        ax.plot(t, noisy_positions[:, i], 'r.', label='noisy', markersize=2, alpha=0.6)
        ax.plot(t, filtered_positions[:, i], 'b-', label='filtered', linewidth=1.5)
        ax.plot(t, predicted_positions[:, i], 'y--', label='predicted(0.5s)', linewidth=1.5)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'{pos_labels[i]} position (m)')
        ax.set_title(f'{pos_labels[i]} position')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # 速度对比
    vel_labels = ['Vx', 'Vy', 'Vz']
    for i in range(3):
        ax = axes[i, 1]
        ax.plot(t, true_velocities[:, i], 'g-', label='true', linewidth=2)
        ax.plot(t, filtered_velocities[:, i], 'b-', label='filtered', linewidth=1.5)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'{vel_labels[i]} velocity (m/s)')
        ax.set_title(f'{vel_labels[i]} velocity')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # 计算误差统计
    def position_error(pos1, pos2):
        """计算位置误差（欧几里得距离）"""
        return np.linalg.norm(pos1 - pos2)
    
    def velocity_error(vel1, vel2):
        """计算速度误差（欧几里得距离）"""
        return np.linalg.norm(vel1 - vel2)
    
    # 计算平均误差
    noisy_pos_errors = [position_error(pos_true, pos_noisy) 
                       for pos_true, pos_noisy in zip(true_positions, noisy_positions)]
    filtered_pos_errors = [position_error(pos_true, pos_filtered) 
                          for pos_true, pos_filtered in zip(true_positions, filtered_positions)]
    
    noisy_vel_errors = [velocity_error(vel_true, vel_filtered) 
                       for vel_true, vel_filtered in zip(true_velocities, filtered_velocities)]
    
    print("="*60)
    print("位置卡尔曼滤波测试结果统计")
    print("="*60)
    print(f"测试时长: {t_end} 秒")
    print(f"时间步长: {dt} 秒")
    print(f"位置噪声标准差: {noise_std} m")
    print(f"带噪声测量平均位置误差: {np.mean(noisy_pos_errors):.4f} m")
    print(f"滤波后平均位置误差: {np.mean(filtered_pos_errors):.4f} m")
    print(f"位置误差改善: {np.mean(noisy_pos_errors) - np.mean(filtered_pos_errors):.4f} m")
    print(f"位置改善比例: {((np.mean(noisy_pos_errors) - np.mean(filtered_pos_errors)) / np.mean(noisy_pos_errors) * 100):.1f}%")
    print(f"速度估计平均误差: {np.mean(noisy_vel_errors):.4f} m/s")
    print("="*60)


def test_next_pose_predictor():
    """
    完整的nextPosePredictor测试，包含双臂末端执行器的位置和姿态滤波测试
    """
    import matplotlib.pyplot as plt
    import math
    
    # 参数设置
    dt = 0.01  # 时间步长
    t_end = 8.0  # 总时间
    t = np.arange(0, t_end, dt)
    n_steps = len(t)
    
    # 生成真值轨迹（双臂末端执行器）
    true_pos_left = []
    true_pos_right = []
    true_quat_left = []
    true_quat_right = []
    
    for i, time in enumerate(t):
        # 左臂位置：圆形轨迹
        radius = 0.3
        center = np.array([0.5, 0.0, 0.8])
        angle = 0.5 * time
        x_left = center[0] + radius * math.cos(angle)
        y_left = center[1] + radius * math.sin(angle)
        z_left = center[2] + 0.1 * math.sin(2 * angle)
        true_pos_left.append(np.array([x_left, y_left, z_left]))
        
        # 右臂位置：8字形轨迹
        x_right = 0.3 * math.sin(0.8 * time)
        y_right = 0.2 * math.sin(1.6 * time)
        z_right = 0.7 + 0.1 * math.cos(0.4 * time)
        true_pos_right.append(np.array([x_right, y_right, z_right]))
        
        # 左臂姿态：绕Z轴旋转
        yaw_left = 0.3 * math.sin(0.6 * time)
        q_left = np.array([
            math.cos(yaw_left/2),
            0,
            0,
            math.sin(yaw_left/2)
        ])
        true_quat_left.append(q_left)
        
        # 右臂姿态：复合旋转
        roll_right = 0.2 * math.sin(0.4 * time)
        pitch_right = 0.15 * math.cos(0.3 * time)
        yaw_right = 0.25 * math.sin(0.5 * time)
        
        # 复合旋转：q = q_roll * q_pitch * q_yaw
        q_roll = np.array([math.cos(roll_right/2), math.sin(roll_right/2), 0, 0])
        q_pitch = np.array([math.cos(pitch_right/2), 0, math.sin(pitch_right/2), 0])
        q_yaw = np.array([math.cos(yaw_right/2), 0, 0, math.sin(yaw_right/2)])
        
        q_temp = quaternion_multiply(q_roll, q_pitch)
        q_right = quaternion_multiply(q_temp, q_yaw)
        q_right = q_right / np.linalg.norm(q_right)
        true_quat_right.append(q_right)
    
    true_pos_left = np.array(true_pos_left)
    true_pos_right = np.array(true_pos_right)
    true_quat_left = np.array(true_quat_left)
    true_quat_right = np.array(true_quat_right)
    
    # 添加噪声生成测量值
    pos_noise_std = 0.02  # 位置噪声标准差
    quat_noise_std = 0.03  # 四元数噪声标准差
    
    noisy_pos_left = []
    noisy_pos_right = []
    noisy_quat_left = []
    noisy_quat_right = []
    
    for i in range(n_steps):
        # 位置噪声
        pos_noise_left = np.random.normal(0, pos_noise_std, 3)
        pos_noise_right = np.random.normal(0, pos_noise_std, 3)
        
        noisy_pos_left.append(true_pos_left[i] + pos_noise_left)
        noisy_pos_right.append(true_pos_right[i] + pos_noise_right)
        
        # 四元数噪声
        quat_noise_left = np.random.normal(0, quat_noise_std, 4)
        quat_noise_right = np.random.normal(0, quat_noise_std, 4)
        
        q_noisy_left = true_quat_left[i] + quat_noise_left
        q_noisy_right = true_quat_right[i] + quat_noise_right
        
        # 归一化
        q_noisy_left = q_noisy_left / np.linalg.norm(q_noisy_left)
        q_noisy_right = q_noisy_right / np.linalg.norm(q_noisy_right)
        
        noisy_quat_left.append(q_noisy_left)
        noisy_quat_right.append(q_noisy_right)
    
    noisy_pos_left = np.array(noisy_pos_left)
    noisy_pos_right = np.array(noisy_pos_right)
    noisy_quat_left = np.array(noisy_quat_left)
    noisy_quat_right = np.array(noisy_quat_right)
    
    # 初始化nextPosePredictor
    predictor = TwoArmPosePredictor(
        noisy_pos_left[0], noisy_pos_right[0],
        noisy_quat_left[0], noisy_quat_right[0],
        dt,
        process_noise_scale=0.001,
        measurement_noise_scale=0.1
    )
    
    # 滤波处理
    filtered_pos_left = []
    filtered_pos_right = []
    filtered_quat_left = []
    filtered_quat_right = []
    predicted_pos_left = []
    predicted_pos_right = []
    predicted_quat_left = []
    predicted_quat_right = []
    
    for i in range(n_steps):
        # 滤波
        pos_l, pos_r, quat_l, quat_r = predictor.filter(
            noisy_pos_left[i], noisy_pos_right[i],
            noisy_quat_left[i], noisy_quat_right[i]
        )
        
        filtered_pos_left.append(pos_l)
        filtered_pos_right.append(pos_r)
        filtered_quat_left.append(quat_l)
        filtered_quat_right.append(quat_r)
        
        # 预测未来
        predict_time = 0.3
        pred_pos_l, pred_pos_r, pred_quat_l, pred_quat_r = predictor.predict_next_pose(predict_time)
        predicted_pos_left.append(pred_pos_l)
        predicted_pos_right.append(pred_pos_r)
        predicted_quat_left.append(pred_quat_l)
        predicted_quat_right.append(pred_quat_r)
        
        if i < 5:  # 只打印前几次的调试信息
            print(f"Step {i}:")
            print(f"  左臂 - 测量位置: {noisy_pos_left[i]}, 滤波位置: {pos_l}")
            print(f"  右臂 - 测量位置: {noisy_pos_right[i]}, 滤波位置: {pos_r}")
    
    filtered_pos_left = np.array(filtered_pos_left)
    filtered_pos_right = np.array(filtered_pos_right)
    filtered_quat_left = np.array(filtered_quat_left)
    filtered_quat_right = np.array(filtered_quat_right)
    predicted_pos_left = np.array(predicted_pos_left)
    predicted_pos_right = np.array(predicted_pos_right)
    predicted_quat_left = np.array(predicted_quat_left)
    predicted_quat_right = np.array(predicted_quat_right)
    
    # 计算欧拉角
    def quaternion_to_euler(q):
        w, x, y, z = q
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return np.array([roll, pitch, yaw])
    
    true_euler_left = np.array([quaternion_to_euler(q) for q in true_quat_left])
    true_euler_right = np.array([quaternion_to_euler(q) for q in true_quat_right])
    noisy_euler_left = np.array([quaternion_to_euler(q) for q in noisy_quat_left])
    noisy_euler_right = np.array([quaternion_to_euler(q) for q in noisy_quat_right])
    filtered_euler_left = np.array([quaternion_to_euler(q) for q in filtered_quat_left])
    filtered_euler_right = np.array([quaternion_to_euler(q) for q in filtered_quat_right])
    predicted_euler_left = np.array([quaternion_to_euler(q) for q in predicted_quat_left])
    predicted_euler_right = np.array([quaternion_to_euler(q) for q in predicted_quat_right])
    
    # 绘图
    fig, axes = plt.subplots(4, 3, figsize=(18, 16))
    fig.suptitle('NextPosePredictor two arm pose filter test', fontsize=16)
    
    # 左臂位置
    pos_labels = ['X', 'Y', 'Z']
    for i in range(3):
        ax = axes[0, i]
        ax.plot(t, true_pos_left[:, i], 'g-', label='true', linewidth=2)
        ax.plot(t, noisy_pos_left[:, i], 'r.', label='noisy', markersize=2, alpha=0.6)
        ax.plot(t, filtered_pos_left[:, i], 'b-', label='filtered', linewidth=1.5)
        ax.plot(t, predicted_pos_left[:, i], 'y--', label=f'predicted({predict_time}s)', linewidth=1.5)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'left arm {pos_labels[i]} position (m)')
        ax.set_title(f'left arm {pos_labels[i]} position')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # 右臂位置
    for i in range(3):
        ax = axes[1, i]
        ax.plot(t, true_pos_right[:, i], 'g-', label='true', linewidth=2)
        ax.plot(t, noisy_pos_right[:, i], 'r.', label='noisy', markersize=2, alpha=0.6)
        ax.plot(t, filtered_pos_right[:, i], 'b-', label='filtered', linewidth=1.5)
        ax.plot(t, predicted_pos_right[:, i], 'y--', label=f'predicted({predict_time}s)', linewidth=1.5)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'right arm {pos_labels[i]} position (m)')
        ax.set_title(f'right arm {pos_labels[i]} position')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # 左臂姿态
    euler_labels = ['Roll', 'Pitch', 'Yaw']
    for i in range(3):
        ax = axes[2, i]
        ax.plot(t, np.degrees(true_euler_left[:, i]), 'g-', label='true', linewidth=2)
        ax.plot(t, np.degrees(noisy_euler_left[:, i]), 'r.', label='noisy', markersize=2, alpha=0.6)
        ax.plot(t, np.degrees(filtered_euler_left[:, i]), 'b-', label='filtered', linewidth=1.5)
        ax.plot(t, np.degrees(predicted_euler_left[:, i]), 'y--', label=f'predicted({predict_time}s)', linewidth=1.5)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'left arm {euler_labels[i]} (deg)')
        ax.set_title(f'left arm {euler_labels[i]}')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # 右臂姿态
    for i in range(3):
        ax = axes[3, i]
        ax.plot(t, np.degrees(true_euler_right[:, i]), 'g-', label='true', linewidth=2)
        ax.plot(t, np.degrees(noisy_euler_right[:, i]), 'r.', label='noisy', markersize=2, alpha=0.6)
        ax.plot(t, np.degrees(filtered_euler_right[:, i]), 'b-', label='filtered', linewidth=1.5)
        ax.plot(t, np.degrees(predicted_euler_right[:, i]), 'y--', label=f'predicted({predict_time}s)', linewidth=1.5)
        ax.set_xlabel('time (s)')
        ax.set_ylabel(f'right arm {euler_labels[i]} (deg)')
        ax.set_title(f'right arm {euler_labels[i]}')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # 计算误差统计
    def position_error(pos1, pos2):
        """计算位置误差（欧几里得距离）"""
        return np.linalg.norm(pos1 - pos2)
    
    def quaternion_error(q1, q2):
        """计算四元数误差（角度）"""
        dot_product = np.clip(np.abs(np.dot(q1, q2)), -1, 1)
        return 2 * np.arccos(dot_product) * 180 / np.pi  # 转换为度
    
    # 计算平均误差
    # 左臂误差
    noisy_pos_errors_left = [position_error(pos_true, pos_noisy) 
                            for pos_true, pos_noisy in zip(true_pos_left, noisy_pos_left)]
    filtered_pos_errors_left = [position_error(pos_true, pos_filtered) 
                               for pos_true, pos_filtered in zip(true_pos_left, filtered_pos_left)]
    
    noisy_quat_errors_left = [quaternion_error(q_true, q_noisy) 
                             for q_true, q_noisy in zip(true_quat_left, noisy_quat_left)]
    filtered_quat_errors_left = [quaternion_error(q_true, q_filtered) 
                                for q_true, q_filtered in zip(true_quat_left, filtered_quat_left)]
    
    # 右臂误差
    noisy_pos_errors_right = [position_error(pos_true, pos_noisy) 
                             for pos_true, pos_noisy in zip(true_pos_right, noisy_pos_right)]
    filtered_pos_errors_right = [position_error(pos_true, pos_filtered) 
                                for pos_true, pos_filtered in zip(true_pos_right, filtered_pos_right)]
    
    noisy_quat_errors_right = [quaternion_error(q_true, q_noisy) 
                              for q_true, q_noisy in zip(true_quat_right, noisy_quat_right)]
    filtered_quat_errors_right = [quaternion_error(q_true, q_filtered) 
                                 for q_true, q_filtered in zip(true_quat_right, filtered_quat_right)]
    
    print("="*80)
    print("NextPosePredictor 双臂末端执行器滤波测试结果统计")
    print("="*80)
    print(f"测试时长: {t_end} 秒")
    print(f"时间步长: {dt} 秒")
    print(f"位置噪声标准差: {pos_noise_std} m")
    print(f"四元数噪声标准差: {quat_noise_std}")
    print()
    
    print("左臂统计:")
    print(f"  带噪声测量平均位置误差: {np.mean(noisy_pos_errors_left):.4f} m")
    print(f"  滤波后平均位置误差: {np.mean(filtered_pos_errors_left):.4f} m")
    print(f"  位置误差改善: {np.mean(noisy_pos_errors_left) - np.mean(filtered_pos_errors_left):.4f} m")
    print(f"  位置改善比例: {((np.mean(noisy_pos_errors_left) - np.mean(filtered_pos_errors_left)) / np.mean(noisy_pos_errors_left) * 100):.1f}%")
    print(f"  带噪声测量平均姿态误差: {np.mean(noisy_quat_errors_left):.2f} 度")
    print(f"  滤波后平均姿态误差: {np.mean(filtered_quat_errors_left):.2f} 度")
    print(f"  姿态误差改善: {np.mean(noisy_quat_errors_left) - np.mean(filtered_quat_errors_left):.2f} 度")
    print(f"  姿态改善比例: {((np.mean(noisy_quat_errors_left) - np.mean(filtered_quat_errors_left)) / np.mean(noisy_quat_errors_left) * 100):.1f}%")
    print()
    
    print("右臂统计:")
    print(f"  带噪声测量平均位置误差: {np.mean(noisy_pos_errors_right):.4f} m")
    print(f"  滤波后平均位置误差: {np.mean(filtered_pos_errors_right):.4f} m")
    print(f"  位置误差改善: {np.mean(noisy_pos_errors_right) - np.mean(filtered_pos_errors_right):.4f} m")
    print(f"  位置改善比例: {((np.mean(noisy_pos_errors_right) - np.mean(filtered_pos_errors_right)) / np.mean(noisy_pos_errors_right) * 100):.1f}%")
    print(f"  带噪声测量平均姿态误差: {np.mean(noisy_quat_errors_right):.2f} 度")
    print(f"  滤波后平均姿态误差: {np.mean(filtered_quat_errors_right):.2f} 度")
    print(f"  姿态误差改善: {np.mean(noisy_quat_errors_right) - np.mean(filtered_quat_errors_right):.2f} 度")
    print(f"  姿态改善比例: {((np.mean(noisy_quat_errors_right) - np.mean(filtered_quat_errors_right)) / np.mean(noisy_quat_errors_right) * 100):.1f}%")
    print("="*80)


# 示例使用
if __name__ == "__main__":
    # 3D位置卡尔曼滤波示例
    initial_state = np.array([0, 0, 0, 0, 0, 0])  # 初始状态 [x, y, z, vx, vy, vz]
    initial_covariance = np.eye(6)  # 初始协方差矩阵
    process_noise = np.eye(6) * 0.01  # 过程噪声协方差矩阵
    measurement_noise = np.eye(3) * 0.1  # 测量噪声协方差矩阵

    kf = KalmanFilter3D(initial_state, initial_covariance, process_noise, measurement_noise)

    measurements = [np.array([1.1, 0.9, 1.0]), np.array([2.1, 1.9, 2.0]), np.array([3.1, 3.0, 3.1])]

    for measurement in measurements:
        filtered_state = kf.filter(measurement)
        print("Filtered State:", filtered_state)

    # 运行nextPosePredictor测试
    test_next_pose_predictor()

    # # 运行四元数滤波测试
    # test_quaternion_filter_with_plotting()
    
    # # 运行位置滤波测试
    # test_position_filter_with_plotting()

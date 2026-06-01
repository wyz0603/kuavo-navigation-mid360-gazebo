# -*- coding: utf-8 -*-
import os
import sys
import rospy
import rosbag
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
import argparse
from rich.console import Console
import questionary
from kuavo_msgs.msg import sensorsData
from std_msgs.msg import Bool
import threading
import subprocess
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
console = Console()

# 关节误差阈值
JOINT_ERROR_THRESHOLD = 0.015  
# 逆解轨迹误差阈值
IK_ERROR_THRESHOLD = 9.0  

class TrajectoryValidator:
    def __init__(self):
        self.reference_joint_states = []
        self.actual_joint_states = []
        self.joint_names = []
        self.time_reference = []
        self.time_actual = []
        self.is_recording = False
        self.start_time = None
        self.recording_sub = rospy.Subscriber('/recording_status', Bool, self.recording_callback)
        # 增加逆解轨迹相关变量
        self.reference_ik_trajectory = []
        self.actual_ik_trajectory = []
        self.time_reference_ik = []
        self.time_actual_ik = []
        
        # 存储多次运行的实际轨迹数据
        self.multiple_runs_trajectories = []  # 存储每次运行的实际关节状态
        self.multiple_runs_times = []         # 存储每次运行的时间戳
        self.multiple_runs_ik_trajectories = []  # 存储每次运行的逆解轨迹
        self.multiple_runs_ik_times = []         # 存储每次运行的逆解轨迹时间戳
        
    def load_reference_bag(self, reference_bag_path):
        """加载参考轨迹的rosbag文件"""
        # console.print(f"[blue]加载参考轨迹: {reference_bag_path}[/blue]")
        try:
            bag = rosbag.Bag(reference_bag_path)
            for topic, msg, t in tqdm(bag.read_messages(topics=['/sensors_data_raw', '/kuavo_arm_traj']), desc="加载参考轨迹"):
                if topic == '/sensors_data_raw':
                    if not self.joint_names and hasattr(msg, 'joint_data') and hasattr(msg.joint_data, 'joint_name'):
                        self.joint_names = msg.joint_data.joint_name
                        console.print(f"[green]成功设置joint_names，共 {len(self.joint_names)} 个关节[/green]")
                    
                    if hasattr(msg, 'joint_data') and hasattr(msg.joint_data, 'joint_q'):
                        # 只关心第12~25个数据（手臂电机数据）
                        arm_joint_data = msg.joint_data.joint_q[:]
                        self.reference_joint_states.append(arm_joint_data)
                        self.time_reference.append(t.to_sec())
                elif topic == '/kuavo_arm_traj':
                    # 记录逆解轨迹数据
                    self.reference_ik_trajectory.append(msg)
                    self.time_reference_ik.append(t.to_sec())
            bag.close()
            # console.print(f"[green]成功加载参考轨迹，共 {len(self.reference_joint_states)} 个关节数据点[/green]")
            # console.print(f"[green]成功加载参考逆解轨迹，共 {len(self.reference_ik_trajectory)} 个数据点[/green]")
            return True
        except Exception as e:
            console.print(f"[red]加载参考轨迹失败: {str(e)}[/red]")
            return False
    
    def recording_callback(self, msg):
        """接收录制状态的回调函数"""
        console.print("[green]获取到录制状态为True...[/green]")
        self.is_recording = msg.data
        if self.is_recording and not self.start_time:
            self.start_time = rospy.Time.now()
            console.print("[green]开始记录实际关节状态...[/green]")
        elif not self.is_recording:
            console.print("[green]停止记录实际关节状态[/green]")
    
    def joint_state_callback(self, msg):
        """接收实际关节状态的回调函数"""
        if self.is_recording:
            if not self.start_time:
                self.start_time = rospy.Time.now()
            
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()
            
            # 只关心第12~25个数据（手臂电机数据）
            arm_joint_data = msg.joint_data.joint_q[:]
            self.actual_joint_states.append(arm_joint_data)
            self.time_actual.append(elapsed_time)
    
    def ik_trajectory_callback(self, msg):
        """接收实际逆解轨迹的回调函数"""
        if self.is_recording:
            if not self.start_time:
                self.start_time = rospy.Time.now()
            
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()
            
            self.actual_ik_trajectory.append(msg)
            self.time_actual_ik.append(elapsed_time)

    
    def play_trajectory_bag(self, trajectory_bag_path=None):
        """播放轨迹指令的rosbag文件"""
        
        # 取消之前的订阅
        if hasattr(self, 'joint_state_sub'):
            self.joint_state_sub.unregister()
        if hasattr(self, 'ik_trajectory_sub'):
            self.ik_trajectory_sub.unregister()
            
        # 清空数据
        self.actual_joint_states = []
        self.time_actual = []
        self.actual_ik_trajectory = []
        self.time_actual_ik = []
        
        while not self.is_recording:
            # console.print("[green]等待录制状态为True...[/green]")
            rospy.sleep(0.1)
            
        # 重新订阅
        self.joint_state_sub = rospy.Subscriber('/sensors_data_raw', sensorsData, self.joint_state_callback)
        self.ik_trajectory_sub = rospy.Subscriber('/kuavo_arm_traj', JointState, self.ik_trajectory_callback)
        
        while self.is_recording:
            rospy.sleep(0.1)
            
        # console.print(f"[green]轨迹播放完成，记录了 {len(self.actual_joint_states)} 个实际关节状态[/green]")
        # console.print(f"[green]轨迹播放完成，记录了 {len(self.actual_ik_trajectory)} 个实际逆解轨迹点[/green]")
        try:
            # 保存当前运行的轨迹数据
            if self.actual_joint_states and self.time_actual:
                self.multiple_runs_trajectories.append(self.actual_joint_states.copy())
                self.multiple_runs_times.append(self.time_actual.copy())
                # console.print(f"[green]已保存第 {len(self.multiple_runs_trajectories)} 次运行的关节状态数据[/green]")
            
            if self.actual_ik_trajectory and self.time_actual_ik:
                self.multiple_runs_ik_trajectories.append(self.actual_ik_trajectory.copy())
                self.multiple_runs_ik_times.append(self.time_actual_ik.copy())
                # console.print(f"[green]已保存第 {len(self.multiple_runs_ik_trajectories)} 次运行的逆解轨迹数据[/green]")
            
            return True
        except Exception as e:
            console.print(f"[red]播放轨迹指令失败: {str(e)}[/red]")
            return False
    
    def calculate_error(self):
        """计算参考轨迹和实际轨迹之间的误差"""
        # 检查是否有数据
        if not self.reference_joint_states:
            console.print("[red]参考轨迹数据为空[/red]")
            return None
        if not self.actual_joint_states:
            console.print("[red]实际轨迹数据为空[/red]")
            return None
        
        reference_states = self.reference_joint_states
        actual_states = self.actual_joint_states
    
        # 如果数据点数量不一致，自动对齐
        if len(reference_states) != len(actual_states):
            if len(reference_states) < len(actual_states):
                actual_states = actual_states[:len(reference_states)]
            else:
                reference_states = reference_states[:len(actual_states)]
        
        # 检查数据点数量是否仍然有效
        if len(reference_states) == 0 or len(actual_states) == 0:
            console.print("[red]对齐后的数据点数量为0[/red]")
            return None
        
        # 检查关节数量是否一致
        if len(reference_states[0]) != len(actual_states[0]):
            console.print(f"[red]关节数量不一致，参考轨迹: {len(reference_states[0])}, 实际轨迹: {len(actual_states[0])}[/red]")
            return None
        
        # 计算每个位置的平方差误差
        errors = []
        num_joints = len(reference_states[0])
        
        for joint_idx in range(num_joints):
            try:
                ref_positions = [state[joint_idx] for state in reference_states]
                act_positions = [state[joint_idx] for state in actual_states]
                
                # 计算平方差误差
                mse = np.mean([(r - a)**2 for r, a in zip(ref_positions, act_positions)])
                errors.append(mse)
                # console.print(f"[green]位置 {joint_idx} 的平方差误差: {mse}[/green]")
            except Exception as e:
                # console.print(f"[red]计算位置 {joint_idx} 的误差时出错: {str(e)}[/red]")
                return None
        
        # 清除实际轨迹数据
        self.actual_joint_states = []
        self.time_actual = []
        
        return errors
    
    def calculate_ik_error(self):
        """计算参考逆解轨迹和实际逆解轨迹之间的误差"""
        # 检查是否有数据
        if not self.reference_ik_trajectory:
            console.print("[red]参考逆解轨迹数据为空[/red]")
            return None
        if not self.actual_ik_trajectory:
            console.print("[red]实际逆解轨迹数据为空[/red]")
            return None
        
        reference_ik = self.reference_ik_trajectory
        actual_ik = self.actual_ik_trajectory
        
        # 如果数据点数量不一致，自动对齐
        if len(reference_ik) != len(actual_ik):
            if len(reference_ik) < len(actual_ik):
                actual_ik = actual_ik[:len(reference_ik)]
            else:
                reference_ik = reference_ik[:len(actual_ik)]
        
        # 检查数据点数量是否仍然有效
        if len(reference_ik) == 0 or len(actual_ik) == 0:
            console.print("[red]对齐后的逆解数据点数量为0[/red]")
            return None
        
        # 计算每个位置的平方差误差
        errors = []
        
        # 假设JointState消息有position字段
        try:
            for i in range(len(reference_ik[0].position)):
                ref_positions = [msg.position[i] for msg in reference_ik]
                act_positions = [msg.position[i] for msg in actual_ik]
                
                # 计算平方差误差
                mse = np.mean([(r - a)**2 for r, a in zip(ref_positions, act_positions)])
                errors.append(mse)
                # console.print(f"[green]逆解轨迹位置 {i} 的平方差误差: {mse}[/green]")
        except Exception as e:
            console.print(f"[red]计算逆解轨迹误差时出错: {str(e)}[/red]")
            return None
        
        self.actual_ik_trajectory = []
        self.time_actual_ik = []
        
        return errors
    
    def visualize_comparison(self, errors=None):
        """可视化参考轨迹和多次运行的实际轨迹的比较"""
        if not self.reference_joint_states or not self.multiple_runs_trajectories:
            console.print("[red]没有足够的数据进行可视化[/red]")
            return
        
        # 使用Agg后端，避免Tkinter问题
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from scipy.interpolate import interp1d
        
        # 获取关节数量
        num_joints = len(self.reference_joint_states[0])
        if num_joints == 0:
            console.print("[red]没有关节数据可用于可视化[/red]")
            return
        
            
        time_reference_filtered = self.time_reference
        reference_states_filtered = self.reference_joint_states
        
        # 为每个关节创建一个子图
        fig, axes = plt.subplots(num_joints, 1, figsize=(12, 3*num_joints), sharex=True)
        
        if num_joints == 1:
            axes = [axes]
        
        # 颜色列表，用于区分不同的运行
        colors = ['r', 'g', 'c', 'm', 'y', 'k', 'orange', 'purple', 'brown', 'pink']
        
        for joint_idx in range(num_joints):
            ax = axes[joint_idx]
            
            # 提取该关节的参考位置
            ref_positions = [state[joint_idx] for state in reference_states_filtered]
            
            # 绘制参考轨迹
            ax.plot(time_reference_filtered, ref_positions, 'b-', linewidth=2, label='Reference trajectory')
            
            # 绘制每次运行的实际轨迹
            for run_idx, (run_trajectory, run_times) in enumerate(zip(self.multiple_runs_trajectories, self.multiple_runs_times)):
                if len(run_times) > 1:
                    # 创建插值函数
                    joint_positions = [state[joint_idx] for state in run_trajectory]
                    interp_func = interp1d(run_times, joint_positions, bounds_error=False, fill_value="extrapolate")
                    
                    # 在参考轨迹的时间点上重采样实际轨迹
                    reference_duration = time_reference_filtered[-1] - time_reference_filtered[0]
                    actual_duration = run_times[-1] - run_times[0]
                    
                    # 时间缩放因子
                    scale_factor = actual_duration / reference_duration if reference_duration > 0 else 1.0
                    
                    resampled_positions = []
                    for t in time_reference_filtered:
                        scaled_t = (t - time_reference_filtered[0]) * scale_factor + run_times[0]
                        resampled_positions.append(interp_func(scaled_t))
                    
                    # 绘制实际轨迹
                    color_idx = run_idx % len(colors)
                    ax.plot(time_reference_filtered, resampled_positions, 
                            color=colors[color_idx], linestyle='-', alpha=0.7,
                            label=f'Run {run_idx+1}')
            
            ax.set_title(f'Position {joint_idx}')
            ax.set_ylabel('Position')
            ax.grid(True)
            ax.legend()
        
        if num_joints > 0:
            axes[-1].set_xlabel('Time (s)')
            plt.tight_layout()
            
            # 保存图像
            output_dir = os.path.expanduser("./")
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            
            timestamp = rospy.Time.now().to_sec()
            output_path = os.path.join(output_dir, f"multiple_runs_comparison_{int(timestamp)}.png")
            plt.savefig(output_path)
            console.print(f"[green]多次运行轨迹比较图已保存至: {output_path}[/green]")
            
            # 关闭图形，避免内存泄漏
            plt.close(fig)
 
    def visualize_multiple_runs_comparison(self):
        """可视化多次运行的逆解轨迹与参考轨迹的比较"""
        if not self.reference_ik_trajectory or not self.multiple_runs_ik_trajectories:
            console.print("[red]没有足够的逆解轨迹数据进行多次运行比较可视化[/red]")
            return
        
        # 使用Agg后端，避免Tkinter问题
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from scipy.interpolate import interp1d
        
        # 获取关节数量
        num_joints = len(self.reference_ik_trajectory[0].position)
        if num_joints == 0:
            console.print("[red]没有逆解轨迹数据可用于可视化[/red]")
            return
        
        # 忽略前50个数据点
        start_idx = 0
        if len(self.time_reference_ik) <= start_idx:
            console.print("[yellow]数据点不足50个，将使用所有可用数据[/yellow]")
            start_idx = 0
            
        time_reference_ik_filtered = self.time_reference_ik
        reference_ik_trajectory_filtered = self.reference_ik_trajectory
        
        # 为每个关节创建一个子图
        fig, axes = plt.subplots(num_joints, 1, figsize=(12, 3*num_joints), sharex=True)
        
        if num_joints == 1:
            axes = [axes]
        
        # 颜色列表，用于区分不同的运行
        colors = ['r', 'g', 'c', 'm', 'y', 'k', 'orange', 'purple', 'brown', 'pink']
        
        for joint_idx in range(num_joints):
            ax = axes[joint_idx]
            
            # 提取该关节的参考位置
            ref_positions = [msg.position[joint_idx] for msg in reference_ik_trajectory_filtered]
            
            # 绘制参考轨迹
            ax.plot(time_reference_ik_filtered, ref_positions, 'b-', linewidth=2, label='Reference trajectory')
            
            # 绘制每次运行的实际轨迹
            for run_idx, (run_trajectory, run_times) in enumerate(zip(self.multiple_runs_ik_trajectories, self.multiple_runs_ik_times)):
                if len(run_times) > 1:
                    # 创建插值函数
                    joint_positions = [msg.position[joint_idx] for msg in run_trajectory]
                    interp_func = interp1d(run_times, joint_positions, bounds_error=False, fill_value="extrapolate")
                    
                    # 在参考轨迹的时间点上重采样实际轨迹
                    reference_duration = time_reference_ik_filtered[-1] - time_reference_ik_filtered[0]
                    actual_duration = run_times[-1] - run_times[0]
                    
                    # 时间缩放因子
                    scale_factor = actual_duration / reference_duration if reference_duration > 0 else 1.0
                    
                    resampled_positions = []
                    for t in time_reference_ik_filtered:
                        scaled_t = (t - time_reference_ik_filtered[0]) * scale_factor + run_times[0]
                        resampled_positions.append(interp_func(scaled_t))
                    
                    # 绘制实际轨迹
                    color_idx = run_idx % len(colors)
                    ax.plot(time_reference_ik_filtered, resampled_positions, 
                            color=colors[color_idx], linestyle='-', alpha=0.7,
                            label=f'Run {run_idx+1}')
            
            ax.set_title(f'IK Position {joint_idx}')
            ax.set_ylabel('Position')
            ax.grid(True)
            ax.legend()
        
        if num_joints > 0:
            axes[-1].set_xlabel('Time (s)')
            plt.tight_layout()
            
            # 保存图像
            output_dir = os.path.expanduser("./")
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            
            timestamp = rospy.Time.now().to_sec()
            output_path = os.path.join(output_dir, f"multiple_runs_ik_comparison_{int(timestamp)}.png")
            plt.savefig(output_path)
            console.print(f"[green]多次运行逆解轨迹比较图已保存至: {output_path}[/green]")
            
            # 显示图像
            plt.show()

    def visualize_element_aligned_comparison(self, errors=None):
        """直接对轨迹元素进行对齐比较，不考虑时间因素"""
        if not self.reference_joint_states or not self.multiple_runs_trajectories:
            console.print("[red]没有足够的数据进行可视化[/red]")
            return
        
        # 使用Agg后端，避免Tkinter问题
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        
        # 获取关节数量
        num_joints = len(self.reference_joint_states[0])
        if num_joints == 0:
            console.print("[red]没有关节数据可用于可视化[/red]")
            return
        
        # 为每个关节创建一个子图
        fig, axes = plt.subplots(num_joints, 1, figsize=(12, 3*num_joints), sharex=True)
        
        if num_joints == 1:
            axes = [axes]
        
        # 颜色列表，用于区分不同的运行
        colors = ['r', 'g', 'c', 'm', 'y', 'k', 'orange', 'purple', 'brown', 'pink']
        
        for joint_idx in range(num_joints):
            ax = axes[joint_idx]
            
            # 提取该关节的参考位置
            ref_positions = [state[joint_idx] for state in self.reference_joint_states]
            
            # 绘制参考轨迹
            ax.plot(range(len(ref_positions)), ref_positions, 'b-', linewidth=2, label='Reference trajectory')
            
            # 绘制每次运行的实际轨迹
            for run_idx, run_trajectory in enumerate(self.multiple_runs_trajectories):
                # 直接提取该关节的位置数据
                joint_positions = [state[joint_idx] for state in run_trajectory]
                
                # 绘制实际轨迹
                color_idx = run_idx % len(colors)
                ax.plot(range(len(joint_positions)), joint_positions, 
                        color=colors[color_idx], linestyle='-', alpha=0.7,
                        label=f'Run {run_idx+1}')
            
            ax.set_title(f'Position {joint_idx}')
            ax.set_ylabel('Position')
            ax.set_xlabel('Sample Index')
            ax.grid(True)
            ax.legend()
        
        if num_joints > 0:
            plt.tight_layout()
            
            # 保存图像
            output_dir = os.path.expanduser("./")
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            
            timestamp = rospy.Time.now().to_sec()
            output_path = os.path.join(output_dir, f"element_aligned_comparison_{int(timestamp)}.png")
            plt.savefig(output_path)
            console.print(f"[green]元素对齐比较图已保存至: {output_path}[/green]")
            
            # 关闭图形，避免内存泄漏
            plt.close(fig)

def select_rosbag_file(base_dir, message):
    """选择rosbag文件"""
    if not os.path.exists(base_dir):
        console.print(f"[red]目录不存在: {base_dir}[/red]")
        return None
    
    # 直接返回base_dir，不再进行文件选择
    # console.print(f"[green]直接使用rosbag文件: {base_dir}[/green]")
    return base_dir

def main():
    rospy.init_node('trajectory_validator', anonymous=True)
    
    validator = TrajectoryValidator()
    
    # 从命令行参数获取rosbag文件路径，如果没有提供则使用当前目录
    if len(sys.argv) > 1:
        rosbag_log_save_path = sys.argv[1]
    else:
        rosbag_log_save_path = os.path.expanduser("./")
    console.print(f"[blue]使用rosbag路径: {rosbag_log_save_path}[/blue]")
    
    # 选择参考轨迹rosbag
    reference_bag_path = select_rosbag_file(rosbag_log_save_path, "请选择包含标准电机位置的参考rosbag文件")
    if not reference_bag_path:
        return
    
    # 加载参考轨迹
    if not validator.load_reference_bag(reference_bag_path):
        return
    
    # 运行5次并记录数据
    # console.print("[blue]将进行5次轨迹运行并记录数据...[/blue]")
    for run in range(1):
        # console.print(f"[blue]开始第 {run+1} 次运行...[/blue]")
        
        if not validator.play_trajectory_bag():
            console.print(f"[red]第 {run+1} 次运行失败，跳过[/red]")
            continue
        console.print(f"[green]第 {run+1} 次运行完成[/green]")
        
        # 每次运行后立即处理数据
        console.print(f"[blue]开始处理第 {run+1} 次运行的数据...[/blue]")
        
        # 计算关节误差
        errors = validator.calculate_error()
        if errors:
            # console.print(f"[green]第 {run+1} 次运行的关节轨迹误差计算完成[/green]")
            
            # 显示每个位置的误差
            # console.print(f"[blue]第 {run+1} 次运行各关节位置的平方差误差 (MSE):[/blue]")
            total_error = 0.0
            for idx, error in enumerate(errors):
                # console.print(f"  位置 {idx}: {error:.4f}")
                total_error += error
            
            # 计算平均误差
            avg_error = total_error / len(errors)
            
            # 根据误差判断动作是否标准
            if avg_error < JOINT_ERROR_THRESHOLD:
                console.print(f"[green]运行平均关节误差: {avg_error:.4f}[/green]")
                console.print(f"[green]运行关节动作执行标准！误差在可接受范围内。[/green]")
            else:
                console.print(f"[red]运行平均关节误差: {avg_error:.4f}[/red]")
                console.print(f"[red]运行关节动作执行不够标准，误差超过阈值。[/red]")
            
            # 可视化比较
            # validator.visualize_comparison(errors)
        else:
            console.print(f"[red]无法计算第 {run+1} 次运行的关节轨迹误差[/red]")
        
        # 计算逆解轨迹误差
        ik_errors = validator.calculate_ik_error()
        if ik_errors:
            # console.print(f"[green]第 {run+1} 次运行的逆解轨迹误差计算完成[/green]")
            
            # 显示每个位置的误差
            # console.print(f"[blue]第 {run+1} 次运行各逆解位置的平方差误差 (MSE):[/blue]")
            total_error = 0.0
            for idx, error in enumerate(ik_errors):
                # console.print(f"  逆解位置 {idx}: {error:.4f}")
                total_error += error
            
            # 计算平均平方差误差
            avg_error = total_error / len(ik_errors)
            
            
            # 根据平方差误差判断逆解是否标准（单位：度）
            if avg_error < IK_ERROR_THRESHOLD:
                console.print(f"[green]运行平均逆解平方差误差: {avg_error:.4f}[/green]")
                console.print(f"[green]运行逆解轨迹执行标准！平方差误差在可接受范围内。[/green]")
            else:
                console.print(f"[red]运行平均逆解平方差误差: {avg_error:.4f}[/red]")
                console.print(f"[red]运行逆解轨迹执行不够标准，平方差误差超过阈值。[/red]")

        else:
            console.print(f"[red]无法计算第 {run+1} 次运行的逆解轨迹误差[/red]")
        
        # 等待一段时间再进行下一次运行
        rospy.sleep(1.0)
    
    # 所有运行完成后，生成多次运行的综合比较图
    # console.print("[blue]生成多次运行的综合比较图...[/blue]")
    # validator.visualize_multiple_runs_comparison()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        console.print("[yellow]程序被用户中断[/yellow]")

#!/usr/bin/env python3

import rospy
import time
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories  # 导入自定义消息类型
from kuavo_msgs.msg import gaitTimeName
from geometry_msgs.msg import Twist
import numpy as np
from utils.sat import RotatingRectangle

def publish_vel_command(x, y, yaw):
        """
        根据给定的vel发布机器人命令。
        """
        twist = Twist()
        twist.linear.x = x  # 线速度，沿x轴
        twist.linear.y = y  # 线速度，沿y轴
        twist.linear.z = 0  # 线速度，沿z轴
        twist.angular.z = yaw
        return twist

def get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj):
    num = len(time_traj)

    # 创建消息实例
    msg = footPoseTargetTrajectories()
    msg.timeTrajectory = time_traj  # 设置时间轨迹
    msg.footIndexTrajectory = foot_idx_traj         # 设置脚索引
    msg.footPoseTrajectory = []  # 初始化脚姿态轨迹

    for i in range(num):
        # 创建脚姿态信息
        foot_pose_msg = footPose()
        foot_pose_msg.footPose = foot_traj[i]      # 设置脚姿态
        foot_pose_msg.torsoPose = torso_traj[i]    # 设置躯干姿态

        # 将脚姿态添加到消息中
        msg.footPoseTrajectory.append(foot_pose_msg)

    return msg

def generate_steps(torso_pos, torso_yaw, foot_bias):
    l_foot_bias = np.array([0, foot_bias, -torso_pos[2]])
    r_foot_bias = np.array([0, -foot_bias, -torso_pos[2]])
    R_z = np.array([
        [np.cos(torso_yaw), -np.sin(torso_yaw), 0],
        [np.sin(torso_yaw), np.cos(torso_yaw), 0],
        [0, 0, 1]
    ])
    l_foot = torso_pos + R_z.dot(l_foot_bias)
    r_foot = torso_pos + R_z.dot(r_foot_bias)
    return l_foot, r_foot   

def get_multiple_steps_msg(body_poses, dt, is_left_first=True, collision_check=True):
    num_steps = 2*len(body_poses)
    time_traj = []
    foot_idx_traj = []
    foot_traj = []
    torso_traj = []
    l_foot_rect_last = RotatingRectangle(center=(0, 0.1), width=0.24, height=0.1, angle=0)
    r_foot_rect_last = RotatingRectangle(center=(0,-0.1), width=0.24, height=0.1, angle=0)
    for i in range(num_steps):
        time_traj.append(dt * (i+1))
        body_pose = body_poses[i//2]
        torso_pos = np.asarray(body_pose[:3])
        torso_yaw = np.radians(body_pose[3])
        # body_pose[3] = torso_yaw    
        l_foot, r_foot = generate_steps(torso_pos, torso_yaw, 0.1)
        l_foot = [*l_foot[:3], torso_yaw]
        r_foot = [*r_foot[:3], torso_yaw]

        if(collision_check and i%2 == 0):
            l_foot_rect_next = RotatingRectangle(center=(l_foot[0],l_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            r_foot_rect_next = RotatingRectangle(center=(r_foot[0],r_foot[1]), width=0.24, height=0.1, angle=torso_yaw)
            l_collision = l_foot_rect_next.is_collision(r_foot_rect_last)
            r_collision = r_foot_rect_next.is_collision(l_foot_rect_last)
            if l_collision and r_collision:
                print("\033[91m[Error] Detect collision, Please adjust your body_poses input!!!\033[0m")
                break
            elif l_collision:
                print("\033[92m[Info] Left foot is in collision, switch to right foot\033[0m")
                is_left_first = False
            else:
                is_left_first = True
            l_foot_rect_last = l_foot_rect_next
            r_foot_rect_last = r_foot_rect_next
        if(i%2 == 0):
            if is_left_first:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)
            else:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
        else:
            if is_left_first:
                foot_idx_traj.append(1)
                foot_traj.append(r_foot)
            else:
                foot_idx_traj.append(0)
                foot_traj.append(l_foot)
        torso_traj.append([*body_pose[:3], torso_yaw])
    print("time_traj:", time_traj)
    print("foot_idx_traj:", foot_idx_traj)
    print("foot_traj:", foot_traj)
    print("torso_traj:", torso_traj)
    return get_foot_pose_traj_msg(time_traj, foot_idx_traj, foot_traj, torso_traj)

# 全局变量
step_time = None  # 保存时间差
last_custom_gait_time = None  # 记录上一次 custom_gait 的时间

def gait_time_name_callback(msg):
    """
    话题回调函数，处理接收到的消息
    """
    global step_time, last_custom_gait_time

    # 捕获 custom_gait
    if msg.gait_name == "custom_gait":
        last_custom_gait_time = msg.start_time
        # rospy.loginfo(f"Captured custom_gait at time: {last_custom_gait_time}")

    # 捕获 stance 并计算时间差
    if msg.gait_name == "stance" and last_custom_gait_time is not None:
        stance_time = msg.start_time
        step_time = stance_time - last_custom_gait_time
        # rospy.loginfo(f"Captured stance at time: {stance_time}")
        rospy.loginfo(f"Step time: {step_time}")

        # 重置 last_custom_gait_time，以便捕获下一组数据
        last_custom_gait_time = None

if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node('foot_pose_publisher', anonymous=True)
    # 创建发布者，话题为 /humanoid_mpc_foot_pose_target_trajectories
    pub_step = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/humanoid_mpc_gait_time_name', gaitTimeName, gait_time_name_callback)

    # 等待一定时间以确保订阅者已经准备好
    rospy.sleep(1)

    is_left_first_default = True # 缺省左脚先行
    # 缺省开启碰撞检测，如果默认规划的步态顺序会导致碰撞，则会自动切换到另一侧的步态,如果设置为False则不会切换步态顺序
    # 注意：碰撞检测开启后，并且可能导致规划失败
    collision_check = True 
    # body_poses基于局部坐标系给定，每一个身体姿态对应两步到达
    dt = 0.4 #迈一步的时间间隔，腾空相和支撑相时间占比各dt/2
    # 一次完整的步态Mode序列为:[SS FS SS SF SS]或者[SS SF SS FS SS]
    # body_pose： [x(m), y(m), z(m), yaw(deg)]
    body_poses = [
        [0.1, 0.1, 0, -45],
        [0.1, 0.1, 0, -90],
        [0.1, 0.0, 0, -180],
        [0.1, 0.1, 0, -180],
    ]
    # 循环执行单步和速度指令
    while not rospy.is_shutdown(): 
        step_msg = get_multiple_steps_msg(body_poses, dt, is_left_first_default, collision_check)
        pub_step.publish(step_msg)

        # 等待 step_time 不为 None
        while step_time is None and not rospy.is_shutdown():
            time.sleep(0.05)
        if step_time is not None:
            time.sleep(step_time)
            # 使用完后，将 step_time 重新置为 None
            step_time = None


        # 发送速度指令并保持 4 秒
        start_time = time.time()
        while time.time() - start_time < 4:  # 保持 4 秒
            vel_msg = publish_vel_command(0.4, 0, 0)
            pub_cmd_vel.publish(vel_msg)
            time.sleep(0.1)
        # 等待5秒，让速度指令执行后机器人恢复stance
        vel_msg = publish_vel_command(0, 0, 0)
        pub_cmd_vel.publish(vel_msg)
        time.sleep(2)  
    
    
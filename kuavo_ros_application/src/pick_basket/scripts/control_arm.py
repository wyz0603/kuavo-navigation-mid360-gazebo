#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
import json
import time
import copy
import math
import numpy as np
import moveit_msgs.msg
from typing import Tuple
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from rospy_message_converter import json_message_converter
from pydrake.all import StartMeshcat
from kuavoRobotSDK import kuavo
from arm_ik import ArmIk
from bezier_curve_planner import BezierCurvePlanner
from bezier_curve_planner import (load_action_frames_file)

# 逆解函数
def _inverse_kinematics(arm_ik:ArmIk, q0:list, target_pose)->list:
    if len(q0) != 7:
        raise ValueError("q0 长度必须为 7")
    ik_joint_q = None
    arm_joint_state = [0.0]*7 + q0
    arm_ik.set_arm_joint_state(arm_joint_state)
    curr_q = arm_ik.curr_q()
    arm_q = arm_ik.computeArmIK(curr_q, None, target_pose)
    if arm_q is not None:
        ik_joint_q = arm_q[-7:].tolist()
    return  ik_joint_q

def generate_smooth_trajectory(q0, q_target, duration=2.0, num_points=200):
    """
    生成从q0到q_target的平滑关节空间轨迹。
    
    参数:
        q0 (list of float): 起始关节角度列表。
        q_target (list of float): 目标关节角度列表。
        duration (float): 轨迹总持续时间（秒）。
        num_points (int): 轨迹中的点数。
        
    返回:
        RobotTrajectory: 包含平滑轨迹的RobotTrajectory对象。
    """
    
    # 确保输入是numpy数组
    q0 = np.array(q0)
    q_target = np.array(q_target)
    
    # 计算每个点的时间间隔
    time_step = duration / (num_points - 1)
    
    # 生成轨迹点
    t = np.linspace(0, 1, num_points)
    traj_points = [q0 + (q_target - q0) * ti for ti in t]
    
    # 创建JointTrajectory消息
    joint_trajectory = JointTrajectory()
    joint_trajectory.header.stamp = rospy.Time.now()
    
    joint_trajectory.joint_names = ["r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
    
    # 填充轨迹点
    for i, point in enumerate(traj_points):
        traj_point =JointTrajectoryPoint()
        traj_point.positions = point.tolist()
        traj_point.time_from_start = rospy.Duration(i * time_step)
        joint_trajectory.points.append(traj_point)
    
    # 创建RobotTrajectory消息
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory = joint_trajectory
    
    return robot_trajectory

class ControlArm(object):
    def __init__(self, model_file_path):
        self._kuavo = kuavo("control_arm_traj")
        end_frames_name = ['torso', 'l_hand_end_virtual', 'r_hand_end_virtual']
        meshcat = StartMeshcat()
        self._arm_ik = ArmIk(model_file_path, end_frames_name, meshcat)
        self._arm_ik.init_state(0.0, 0.0, [0] * 14)
        self._bezier_curve_planner = BezierCurvePlanner()

    def IK(self, q0:list, target_pose:Pose)->list:
        return _inverse_kinematics(self._arm_ik, q0, target_pose)
    
    def change_arm_ctrl_mode(self, control_mode)->bool:
        """
        - 修改手臂控制模式，control_mode 有三种模式
        - 0: keep pose 保持姿势 
        - 1: auto_swing_arm 行走时自动摆手，切换到该模式会自动运动到摆手姿态
        - 2: external_control 外部控制，手臂的运动由外部控制
        """
        return self._kuavo.set_robot_arm_ctl_mode(control_mode)    
    
    def load_traj(self, path: str) -> moveit_msgs.msg.RobotTrajectory:
        """加载轨迹
        """
        with open(path, "r") as f:
            traj = json.load(f)
        traj = json_message_converter.convert_json_to_ros_message("moveit_msgs/RobotTrajectory", traj)
        # print(f'轨迹已从{path}中加载')
        return traj

    def merge_traj(self, traj1, traj2) -> moveit_msgs.msg.RobotTrajectory:
        # 合并轨迹时忽略 traj2 的第一个点
        traj = traj1
        for point in traj2.joint_trajectory.points[1:]:
            traj.joint_trajectory.points.append(point)
        return traj  
    
    def get_interpolate_trajectory(self, start_q, start_pose, end_pose, duration=2.0, num_points=5):
        # 提取位置信息
        start_position = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        end_position = np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z])

        # 固定使用的四元数
        fixed_orientation = copy.deepcopy(end_pose.orientation)

        total_dist = 0.0
        waypoints = []
        joint_values = [0.0]*7
        prev_position = start_position
        
        if num_points == 1:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = end_position
            pose.orientation = copy.deepcopy(fixed_orientation)
            joint_values = self.IK(start_q, pose)
            if joint_values is not None:
                waypoints.append([duration, joint_values])  # 时间戳为0.0
                return True, waypoints
            else:
                print("No IK solution found for the single point")
                return False, []
    
        for i in range(num_points):
            t = float(i) / (num_points - 1)
            position = (1 - t) * start_position + t * end_position
            
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation = copy.deepcopy(fixed_orientation)
            
            # 调用逆解函数
            joint_values = self.IK(start_q, pose)
            if joint_values is not None:  # 确保有解
                dist = np.linalg.norm(position - prev_position)
                total_dist += dist
                prev_position = position
                waypoints.append([dist, joint_values])
            else:
                print("No IK solution found for point", i)
                pass

        # can't reach end_pose       
        if self.IK(start_q, end_pose) is None:
            return False, waypoints
        else:
            for i in range(len(waypoints)):
                dist = waypoints[i][0]
                time_gap = (dist / total_dist) * duration # 根据距离比例计算时间间隔
                waypoints[i][0] = time_gap
            return True, waypoints  

    def _execute_kuavo_arm_target_poses(self, traj:moveit_msgs.msg.RobotTrajectory, start_time_point) \
        -> Tuple[bool, float]:
        """
         按照 kuavo 的 kuavo_arm_target_poses 接口执行轨迹
         返回执行成功与否，以及预计执行时间
        """
        if len(traj.joint_trajectory.points) == 0:
            return False, 0.0
        
        print("_execute_kuavo_arm_target_poses, traj:", len(traj.joint_trajectory.points))

        times = []
        values = []

        time_point = start_time_point  #  从当前值执行到轨迹的第一个点间隔时间， 尽量宽裕些
        time_cost = start_time_point
        index = 0
        step = 1
        dt = 0.01
        for point in traj.joint_trajectory.points:
            index += 1
            if index % step != 0:
                continue
            time_point += dt  # 轨迹点间隔时间
            time_cost += dt*1.5
            times.append(time_point)
            values.append([0.0]*7 + point.positions)
        
        # 至少保证有一个点
        if len(times) == 0:
            times.append(1.5)
            values.append([0.0]*7 + traj.joint_trajectory.points[0].positions)
        
        # 保证轨迹终点被执行
        if len(traj.joint_trajectory.points) % step != 0:
            times.append(time_point+dt)
            values.append([0.0]*7 + traj.joint_trajectory.points[-1].positions)

        # print("execute_kuavo_arm_target_poses, times:", times)
        # print("execute_kuavo_arm_target_poses, values:", values)
        
        # 调用旧版 Kuavo 手臂轨迹执行接口
        self._kuavo.set_kuavo_arm_target_poses(times, values)
        return True, time_cost
    
    def execute_action_frames(self, frames, timeout=40)-> bool:
        """
        execute joint frames
        frames: list of list, each frame is a list of joint angles
        timeout: float, the time point to start the first frame
        return: bool
        """
        # plan arm traj by bezier curve    
        if self._bezier_curve_planner.plan(frames, timeout) == True:
            print("execute_action_frames: plan arm traj by bezier curve success")
            return True   
        return False
    
    def execute_traj(self, traj:moveit_msgs.msg.RobotTrajectory, start_time_point=1.8) -> Tuple[bool, float]:
        """
         执行手臂轨迹
         返回执行成功与否，以及预计执行时间
        """
        return self._execute_kuavo_arm_target_poses(traj, start_time_point)

    def arm_reset(self):    
        self._kuavo.set_mpc_arm_traj([1.5], [[0,0,0,-0,0,0,0,0,0,0,-0,0,0,0]])
        time.sleep(1.5)
     
def test_pre_defined_traj(arm_controller:ControlArm):
    positions =[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [-3, -10, -15, -15, 20, -10, 10],
                [-6, -20, -25, -25, 26, -14, 16],
                [-11, -40, -24, -25, 26, -14, 16],
                [-35, -40, -12, -30, 4, -14, 16],
                [-35, -19, -14, -47, 4, -24, 16],
                [-35, -20, -5, -55, 50, -30, 10]]
    
    traj = RobotTrajectory()
    points = []
    for pos in positions:
        point = JointTrajectoryPoint()
        point.positions = pos
        points.append(point)

    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ["r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
    joint_trajectory.points = points
    traj.joint_trajectory = joint_trajectory

    arm_controller.change_arm_ctrl_mode(2)
    time.sleep(2.5)
    _, time_cost = arm_controller.execute_traj(traj)

if __name__ == "__main__":
    rospy.init_node("test_arm_control_node")
    model_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                       '../../ros_robotModel/biped_s3/urdf/biped_s3_arm.urdf')
    arm_controller = ControlArm(model_file_path)

    tact_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/action_files/pick1.tact")

    action_frames = load_action_frames_file(tact_path)
    time.sleep(0.5)
    arm_controller.execute_action_frames(action_frames)

    exit(0)
#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import json
import rospy
import numpy as np

import moveit_msgs.msg
import trajectory_msgs.msg
import rospy_message_converter.json_message_converter
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Quaternion

def load_json(path: str) -> dict:
    """ 加载json文件 """
    with open(path, "r") as f:
        json_data = json.load(f)
    rospy.loginfo("json已从{}中加载".format(path))
    
    return json_data


def rad_to_angle(rad_list: list) -> list:
    """ 弧度转变为角度 """
    return (np.array(rad_list)/np.pi*180).tolist()


def angle_to_rad(angle_list: list) -> list:
    """ 角度转变为弧度 """
    return (np.array(angle_list)/180*np.pi).tolist()


def check_num(num: float) -> bool:
    """ 检查输入数字 """
    if str(num).isdigit() and num > 0:
        return True
    else:
        return False

def l_to_r(l_traj: moveit_msgs.msg.RobotTrajectory) -> moveit_msgs.msg.RobotTrajectory:
    """左手到右手轨迹对称映射
    """
    r_traj = moveit_msgs.msg.RobotTrajectory()
    r_traj.joint_trajectory.header = l_traj.joint_trajectory.header
    r_traj.joint_trajectory.joint_names = [
        "r_arm_pitch",
        "r_arm_roll",
        "r_arm_yaw",
        "r_forearm_pitch",
        "r_forearm_yaw",
        "r_hand_roll",
        "r_hand_pitch"
    ]
    
    for l_point in l_traj.joint_trajectory.points:
        r_point = trajectory_msgs.msg.JointTrajectoryPoint()
        r_point.time_from_start = l_point.time_from_start
        r_point.positions = [
             l_point.positions[0],
            -l_point.positions[1],
            -l_point.positions[2],
             l_point.positions[3],
             l_point.positions[4],
            -l_point.positions[5],
             l_point.positions[6]
        ]
        r_point.velocities = l_point.velocities
        r_point.accelerations = l_point.accelerations
        r_traj.joint_trajectory.points.append(r_point)
    
    return r_traj

def load_traj(path: str) -> moveit_msgs.msg.RobotTrajectory:
    """加载轨迹
    """
    with open(path, "r") as f:
        traj = json.load(f)
    traj = rospy_message_converter.json_message_converter.convert_json_to_ros_message("moveit_msgs/RobotTrajectory", traj)
    rospy.loginfo("轨迹已从{}中加载".format(path))
    return traj

def orientation_to_rpy(ori:Quaternion):
    quat = [ori.x, ori.y, ori.z, ori.w]
    return Rotation.from_quat(quat).as_euler('xyz', degrees=False)

def rpy_to_orientation(rpy:list):
    quat = Rotation.from_euler('xyz', rpy).as_quat()
    return Quaternion(quat[0], quat[1], quat[2], quat[3])
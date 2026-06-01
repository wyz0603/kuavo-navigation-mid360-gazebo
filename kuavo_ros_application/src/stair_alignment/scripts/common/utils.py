#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import moveit_msgs.msg
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion
    
def Quaternion_to_quat(q:Quaternion):
    quat = [q.x, q.y, q.z, q.w]
    return R.from_quat(quat).as_quat()

def normalize_rpy(rpy):
    """
    规范化欧拉角（RPY），使其在 [-π, π] 范围内
    """
    roll, pitch, yaw = rpy
    roll = (roll + np.pi) % (2 * np.pi) - np.pi
    pitch = (pitch + np.pi) % (2 * np.pi) - np.pi
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
    return np.array([roll, pitch, yaw])

def rpy_degree(rpy:list)->list:    
    return [r * 180.0 / np.pi for r in rpy]    

def rpy_to_orientation(rpy:list, seq='xyz')->Quaternion:
    quat = R.from_euler(seq, rpy).as_quat()
    quat = R.from_quat(quat).as_quat()
    return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

def rpy_from_orientation(orientation:Quaternion, seq='xyz')->list:
    return normalize_rpy(R.from_quat(Quaternion_to_quat(orientation)).as_euler(seq))

def get_traj_last_joint_q(traj: moveit_msgs.msg.RobotTrajectory) -> list:
    """
        获取轨迹最后一帧的关节角度
    """
    if traj is None or len(traj.joint_trajectory.points) == 0:
            return None
    return traj.joint_trajectory.points[-1].positions


if __name__ == "__main__":
    # ori  = Quaternion(-0.30014, -0.60205, 0.629348,0.389272)
    ori  = Quaternion(-0.24088, -0.55538, 0.602329, 0.250311)

    print("ori:", ori.x, ori.y, ori.z, ori.w)
    rpy = rpy_from_orientation(ori)
    print()
    ori = rpy_to_orientation(rpy)
    print(f'rpy: {rpy}, ori: {ori.x}, {ori.y}, {ori.z}, {ori.w}')
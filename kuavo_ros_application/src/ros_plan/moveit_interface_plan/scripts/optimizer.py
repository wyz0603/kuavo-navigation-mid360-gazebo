#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import moveit_msgs.msg
import trajectory_msgs.msg

from base import Base


class Optimizer(Base):
    """ 轨迹优化器 """
    
    _instance = None
    def __new__(cls, *args, **kwargs):
        """ 单实例模式 """
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance
    
    def __init__(self) -> None:
        pass
        
        
    def retime_traj(self, traj: moveit_msgs.msg.RobotTrajectory) -> moveit_msgs.msg.RobotTrajectory:
        """重新计时轨迹优化
        
        :param traj: 待优化轨迹
        :return: 重新计时轨迹优化
        """
        traj = self._move_group.retime_trajectory(
            self._robot.get_current_state(),
            traj,
            velocity_scaling_factor=self._max_vel_scaling_factor,
            acceleration_scaling_factor=self._max_acc_scaling_factor,
            algorithm="time_optimal_trajectory_generation",
        )
        
        return traj
    
        
    def min_variance_traj(self, trajs: list) -> moveit_msgs.msg.RobotTrajectory:
        """计算最小方差轨迹
        
        :param trajs: 轨迹数组
        :return: 具有最小方差的轨迹
        """
        if not trajs:
            rospy.logwarn("未检测到轨迹")
            return None
        
        variances = np.zeros(len(trajs)).tolist()
        position_matrix = []
        for i, traj in enumerate(trajs):
            points = traj.joint_trajectory.points
            for point in points:
                position_matrix.append(point.positions)
            position_matrix_T = np.array(position_matrix).T
            for positions in position_matrix_T:
                variances[i] += np.var(positions)
            rospy.loginfo("第{}条轨迹方差：{}".format(i+1, variances[i]))
        
        rospy.loginfo("选中第{}条轨迹, 方差值:{}".format(variances.index(min(variances))+1, min(variances)))
        return trajs[variances.index(min(variances))]
    
    def interpolate_traj(self, traj: trajectory_msgs.msg.JointTrajectory, scale: int=10) -> trajectory_msgs.msg.JointTrajectory:
        """轨迹插值优化
        
        :param traj: 待插值轨迹
        :param scale: 放缩比例
        :return: 已插值轨迹
        """
        new_traj = trajectory_msgs.msg.JointTrajectory()
        new_traj.header = traj.header
        new_traj.joint_names = traj.joint_names
        new_traj.points = []
        for i in range(len(traj.points) - 1):
            point = traj.points[i]
            next_point = traj.points[i + 1]
            new_traj.points.append(point)
            t = 0
            while t < 1:
                new_point = trajectory_msgs.JointTrajectoryPoint()
                new_point.time_from_start = (
                    point.time_from_start
                    + (next_point.time_from_start - point.time_from_start) * t
                )
                new_point.positions = []
                for j in range(len(point.positions)):
                    new_point.positions.append(
                        point.positions[j]
                        + (next_point.positions[j] - point.positions[j]) * t
                    )
                rospy.loginfo("vel: {}".format(point.velocities))
                new_point.velocities = point.velocities
                new_traj.points.append(new_point)
                t += 1 / scale
        new_traj.points.append(traj.points[-1])
        return new_traj
    
    
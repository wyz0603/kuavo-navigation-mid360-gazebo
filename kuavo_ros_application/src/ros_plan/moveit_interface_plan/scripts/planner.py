#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from typing import overload

from base import Base
from optimizer import Optimizer
from utils import check_num


class Planner(Base):
    """ 规划器 """
    
    _instance = None
    def __new__(cls, *args, **kwargs):
        """ 单实例模式 """
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance


    def __init__(self) -> None:
        self._optimizer = Optimizer()
    
    
    @classmethod
    def set_planning_time(cls, planning_time: float) -> None:
        if check_num(planning_time):
            cls._move_group.set_planning_time(planning_time)
        
        
    @classmethod
    def set_num_planning(cls, num_planning: float) -> None:
        if check_num(num_planning):
            cls._move_group.set_num_planning_attempts(num_planning)
    
    
    @classmethod
    def set_joint_tolerance(cls, joint_tolerance: float) -> None:
        if check_num(joint_tolerance):
            cls._move_group.set_goal_joint_tolerance(joint_tolerance)
        
        
    @classmethod
    def set_position_tolerance(cls, position_tolerance: float) -> None:
        if check_num(position_tolerance):
            cls._move_group.set_goal_position_tolerance(position_tolerance)
        
        
    @classmethod
    def set_orientation_tolerance(cls, orientation_tolerance: float) -> None:
        if check_num(orientation_tolerance):
            cls._move_group.set_goal_orientation_tolerance(orientation_tolerance)
    
    @classmethod
    def get_move_group_name(cls) -> str:
        return cls._move_group_name
    
    @classmethod
    def get_joint_name(cls) -> list:
        return cls._joint_name    
    def _init_grasp(self) -> None:
        """ 初始化抓取 """
        self._grasp = moveit_msgs.msg.Grasp()
        self._grasp.grasp_pose.header.frame_id = self._planning_frame
        self._grasp.grasp_posture.joint_names = self._gripper_name
        self._grasp.grasp_pose.header.stamp = rospy.Time.now()
    
    
    def _init_place(self) -> None:
        """ 初始化放置 """
        self._place = moveit_msgs.msg.PlaceLocation()
        self._place.place_pose.header.frame_id = self._planning_frame
        self._place.post_place_posture.joint_names = self._gripper_name
        self._place.place_pose.header.stamp = rospy.Time.now()
    
    
    def _init_gripper(self) -> None:
        """ 初始化手抓 """
        self._gripper_pose = trajectory_msgs.msg.JointTrajectoryPoint()
        self._gripper_pose.time_from_start = rospy.Duration(self._GRIPPER_MOTION_TIME)
    
    
    def init_arm(self) -> None:
        """ 初始化手臂位置 """
        self._move_group.go(np.zeros(7).tolist(), wait=True)
    
    
    def get_current_joints(self) -> list:
        """ 获取当前关节位姿信息 """
        return self._move_group.get_joints()
    
    def get_current_joints_values(self) -> list:
        """ 获取当前关节位姿的各个关节的信息 """
        return self._move_group.get_current_joint_values()

    def get_current_pose(self) -> geometry_msgs.msg.PoseStamped:
        """ 获取当前笛卡尔位姿信息 """
        return self._move_group.get_current_pose()
    
    
    def set_start_state_to_current_state(self):
        """ 设定起始关节状态为当前状态 """
        self._move_group.set_start_state_to_current_state()
    
    
    def set_start_state(self, joints: list) -> None:
        """ 设定起始关节状态 """
        start_state = moveit_msgs.msg.RobotState()
        start_state.joint_state.header.frame_id = self._planning_frame
        start_state.joint_state.header.stamp = rospy.Time.now()
        start_state.joint_state.name = self._joint_name
        start_state.joint_state.position = joints
        self._move_group.set_start_state(start_state)
    
    
    def plan_to_target_joints(
        self,
        joints: list,
        optimize=True,
        add_queue: bool=True
    ) -> moveit_msgs.msg.RobotTrajectory:
        """规划关节目标位置
        
        :param joints: 目标关节位置
        :param optimize: 是否开启优化
        :return: 规划的轨迹，失败返回None
        """

        self._move_group.set_joint_value_target(joints)
        trajs = []
        for i in range(self._num_planning):
            rospy.loginfo("第{}次关节空间规划".format(i+1))
            (success, traj, *_) = self._move_group.plan()
            if success:
                rospy.loginfo("第{}次规划成功".format(i+1))
                trajs.append(traj)
            else:
                rospy.logwarn("第{}次规划失败".format(i+1))
        
        if not trajs:
            rospy.logerr("规划失败")
            return None
        
        min_variance_traj = self._optimizer.min_variance_traj(trajs)
        if optimize:
            traj = self._optimizer.retime_traj(min_variance_traj)
        if add_queue:
            self._traj_queue.put(traj)
        
        return traj
    @overload
    def plan_to_target_pose(
        self,
        pose: geometry_msgs.msg.Pose
    ) -> moveit_msgs.msg.RobotTrajectory:
        ...
        
        
    @overload
    def plan_to_target_pose(
        self,
        pose: geometry_msgs.msg.PoseStamped
    ) -> moveit_msgs.msg.RobotTrajectory:
        ...
    

    def plan_to_target_pose(
        self,
        pose: ...,
        optimize=True
    ) -> moveit_msgs.msg.RobotTrajectory:
        """规划笛卡尔目标位置
        
        :param pose: 目标笛卡尔位置
        :param wait: 是否等待仿真执行
        :param optimize: 是否开启优化
        :return: 已优化轨迹，失败返回None
        """

        self._move_group.set_pose_target(pose)
        trajs = []
        for i in range(self._num_planning):
            rospy.loginfo("第{}次笛卡尔空间规划".format(i+1))
            (success, traj, *_) = self._move_group.plan()
            if success:
                rospy.loginfo("第{}次规划成功".format(i+1))
                trajs.append(traj)
            else:
                rospy.logwarn("第{}次规划失败".format(i+1))

        if not trajs:
            rospy.logerr("规划失败")
            return None
        
        min_variance_traj = self._optimizer.min_variance_traj(trajs)
        if optimize:
            traj = self._optimizer.retime_traj(min_variance_traj)
        self._traj_queue.put(traj)
        
        return traj
    
    
    @overload
    def set_grasp_pose(
        self,
        pose: geometry_msgs.msg.Pose
    ) -> None:
        ...
    
    
    @overload
    def set_grasp_pose(
        self,
        pose: geometry_msgs.msg.PoseStamped
    ) -> None:
        ...
    
    
    def set_grasp_pose(
        self,
        pose: ...,
        approach_direction=[1, 0, 0],
        approach_min_dis=0.04,
        approach_desired_dis=0.05,
        retreat_direction=[0, 0, 1],
        retreat_min_dis=0.04,
        retreat_desired_dis=0.05
    ) -> None:
        """设置抓取配置
        
        :param pose: 抓取位姿
        :param approach_direction: 预抓取方向
        :param approach_min_dis: 预抓取最小前进距离
        :param approach_desired_dis: 预抓取期望前进距离
        :param retreat_direction: 撤退方向
        :param retreat_min_dis: 撤退最小退后距离
        :param retreat_desired_dis: 撤退期望退后距离
        """
        self._init_grasp()
        self._grasp.grasp_pose.pose = pose
        
        self._grasp.pre_grasp_approach.direction.vector = geometry_msgs.msg.Vector3(*approach_direction)
        self._grasp.pre_grasp_approach.min_distance = approach_min_dis
        self._grasp.pre_grasp_approach.desired_distance = approach_desired_dis
        
        self._grasp.post_grasp_retreat.direction.vector = geometry_msgs.msg.Vector3(*retreat_direction)
        self._grasp.post_grasp_retreat.min_distance = retreat_min_dis
        self._grasp.post_grasp_retreat.desired_distance = retreat_desired_dis
    
    
    @overload
    def set_place_pose(
        self,
        pose: geometry_msgs.msg.Pose
    ) -> None:
        ...
        
        
    @overload
    def set_place_pose(
        self,
        pose: geometry_msgs.msg.PoseStamped
    ) -> None:
        ...
    
    
    def set_place_pose(
        self,
        pose: geometry_msgs.msg.Pose,
        approach_direction=[0, 0, -1], 
        approach_min_dis=0.08, 
        approach_desired_dis=0.1,
        retreat_direction=[-1, 0, 0],
        retreat_min_dis=0.04,
        retreat_desired_dis=0.05
    ) -> None:
        """设置放置配置
        
        :param pose: 放置位姿
        :param approach_direction: 预放置方向
        :param approach_min_dis: 预放置最小前进距离
        :param approach_desired_dis: 预放置期望前进距离
        :param retreat_direction: 撤退方向
        :param retreat_min_dis: 撤退最小退后距离
        :param retreat_desired_dis: 撤退期望退后距离
        """
        self._init_place()
        self._place.place_pose.pose = pose
        
        self._place.pre_place_approach.direction.vector = geometry_msgs.msg.Vector3(*approach_direction)
        self._place.pre_place_approach.min_distance = approach_min_dis
        self._place.pre_place_approach.desired_distance = approach_desired_dis
        
        self._place.post_place_retreat.direction.vector = geometry_msgs.msg.Vector3(*retreat_direction)
        self._place.post_place_retreat.min_distance = retreat_min_dis
        self._place.post_place_retreat.desired_distance = retreat_desired_dis


    def set_grasp_gripper_pose(
        self,
        pre_position: list=[0.01, 0.01],
        position: list=[0, 0]
    ) -> None:
        """设置抓取手抓位置
        
        :param pre_position: 预抓取时手抓开合
        :param position: 抓取时手抓开合
        """
        self._init_gripper()
        self._gripper_pose.positions = pre_position
        self._grasp.pre_grasp_posture.points.append(self._gripper_pose)
        
        self._init_gripper()
        self._gripper_pose.positions = position
        self._grasp.grasp_posture.points.append(self._gripper_pose)
    
    
    def set_place_gripper_pose(
        self,
        position: list=[0.01, 0.01]
    ) -> None:
        """设置放置手抓位置
        
        :param position: 放置时手抓开合
        """
        self._init_gripper()
        self._gripper_pose.positions = position
        self._place.post_place_posture.points.append(self._gripper_pose)


    def plan_to_grasp(
        self,
        target_name: str,
        optimize=True
    ) -> moveit_msgs.msg.RobotTrajectory:
        """规划抓取目标
        
        :param target_name: 目标名
        :param optimize: 是否开启优化
        """
        
        for i in range(self._num_planning):
            rospy.loginfo("第{}次笛卡尔空间抓取".format(i+1))
            result = self._move_group.pick(target_name, self._grasp)
            if result == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                try:
                    traj_msg = rospy.wait_for_message(
                        "/move_group/display_planned_path",
                        moveit_msgs.msg.DisplayTrajectory,
                        timeout=3)
                    traj = traj_msg.trajectory
                    rospy.loginfo("抓取成功")
                    break
                except rospy.ROSException as e:
                    rospy.logerr("未收到轨迹消息：{}".format(str(e)))
                    return None
        else:
            rospy.logerr("抓取失败")
            return None
        
        if optimize:
            traj = self._optimizer.retime_traj(traj)
        self._traj_queue.put(traj)
        
        return traj


    def plan_to_place(
        self,
        target_name: str,
        optimize=True
    ) -> moveit_msgs.msg.RobotTrajectory:
        """规划放置目标
        
        :param target_name: 目标名
        :param optimize: 是否开启优化
        """

        for i in range(self._num_planning):
            rospy.loginfo("第{}次笛卡尔空间放置".format(i+1))
            result = self._move_group.place(target_name, self._place)
            if result == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                try:
                    traj_msg = rospy.wait_for_message(
                        "/move_group/display_planned_path",
                        moveit_msgs.msg.DisplayTrajectory,
                        timeout=3)
                    traj = traj_msg.trajectory
                    rospy.loginfo("放置成功")
                    break
                except rospy.ROSException as e:
                    rospy.logerr("未收到轨迹消息：{}".format(str(e)))
                    return None
        else:
            rospy.logerr("放置失败")
            return None
        
        if optimize:
            traj = self._optimizer.retime_traj(traj)
        self._traj_queue.put(traj)
        
        return traj

    def retime_trajectory(self, traj_in):
        return self._optimizer.retime_traj(traj_in)
        # return self._move_group.retime_trajectory(self._move_group.get_current_state(), traj_in)
    def push_trajectory(self, traj:moveit_msgs.msg.RobotTrajectory):
        self._traj_queue.put(traj)
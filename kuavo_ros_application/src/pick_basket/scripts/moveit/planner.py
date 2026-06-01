#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import rospy
import json
import time
import numpy as np
import rospy_message_converter.json_message_converter
from geometry_msgs.msg import Pose,  Point, Quaternion, PoseStamped
from moveit_msgs.msg import RobotTrajectory,RobotState
from moveit_wrap import MoveitWrapBase
from nav_msgs.msg import Path
from std_msgs.msg import Header

from moveit_ifk_wrap import MoveItArmIFKWrap
def dump_traj(
        traj1: RobotTrajectory,
        file_name: str = None
    ) -> None:
        """ 存入轨迹 """
        traj = rospy_message_converter.json_message_converter.convert_ros_message_to_json(traj1)
        file_name = file_name + ".json"
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
        with open(path, "w") as f:
            json.dump(traj, f)
        rospy.loginfo("轨迹保存到{}".format(path))


class MoveitPlanner(MoveitWrapBase):
    """ 规划器 """
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(MoveitPlanner, cls).__new__(cls)
        return cls._instance

    def __init__(self, config_file):
        # 确保只初始化一次
        if not hasattr(self, 'initialized'):
            print("Planner init")
            super().__init__(config_file)
            self.initialized = True  # 标记已经初始化
    
    def _min_variance_traj(self, trajs: list) -> RobotTrajectory:
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
    
    def set_start_state(self, joints: list) -> None:
        """ 设定起始关节状态 """
        start_state = RobotState()
        start_state.joint_state.header.frame_id = self.config.planning_frame
        start_state.joint_state.header.stamp = rospy.Time.now()
        start_state.joint_state.name = self.config.joint_name
        start_state.joint_state.position = joints
        self._move_group.set_start_state(start_state)

    def plan_to_target_joints(
        self,
        joints: list,
        optimize=True
    ) -> RobotTrajectory:
        """规划关节目标位置
        
        :param joints: 目标关节位置
        :param optimize: 是否开启优化
        :return: 规划的轨迹，失败返回None
        """

        self._move_group.set_joint_value_target(joints)
        trajs = []
        for i in range(self.config.num_planning):
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
        
        min_variance_traj = self._min_variance_traj(trajs)
        if optimize:
            pass #TODO
        
        return traj
    
    def plan_to_target_pose(
        self,
        pose: Pose,
        optimize=True
    ) -> RobotTrajectory:
        """规划笛卡尔目标位置
        
        :param pose: 目标笛卡尔位置
        :param wait: 是否等待仿真执行
        :param optimize: 是否开启优化
        :return: 已优化轨迹，失败返回None
        """

        self._move_group.set_pose_target(pose)
        trajs = []
        for i in range(self.config.num_planning):
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
        
        min_variance_traj = self._min_variance_traj(trajs)
        if optimize:
            pass #TODO
        
        return traj
    
    def compute_cartesian_path(self, waypoints, eef_step=0.01):
        """
        计算给定的pose列表的笛卡尔路径
        """
        return self.move_group.compute_cartesian_path(waypoints, eef_step)


class EEPosePublisher:
    def __init__(self, path_topic='/ee_path'):
        # 创建 Path 发布者
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=10)

        # 设置路径消息的头信息
        self.path = Path()
        self.path.header = Header()

    def add_pose_to_path(self, pose):
        # 创建一个 PoseStamped 消息
        pose_stamped = PoseStamped()
        pose_stamped.header = self.path.header
        pose_stamped.header.frame_id = 'torso'
        pose_stamped.pose = pose

        # 将 PoseStamped 添加到路径中
        self.path.poses.append(pose_stamped)

    def publish_path(self):
        self.path_pub.publish(self.path)

def angle_to_rad(angle_list: list) -> list:
    """ 角度转变为弧度 """
    return (np.array(angle_list)/180*np.pi).tolist()

def gen_pre_defined_traj(planner: MoveitPlanner, positions):
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names =  ["r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
    for index in range(0, len(positions)-1):
        planner.set_start_state(angle_to_rad(positions[index]))
        traj1 = planner.plan_to_target_joints(angle_to_rad(positions[index+1]))
        if traj1 is not None:
            for j in traj1.joint_trajectory.points:
                traj.joint_trajectory.points.append(j)
        index += 1   
    return traj

if __name__ == "__main__":
    rospy.init_node("moveit_planner_node")
    planner = MoveitPlanner('/root/dev/kuavo_ros_application/src/pick_basket/config/moveit_config.json')
    
    joint_names = ["r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
    arm_ifk = MoveItArmIFKWrap(joint_names)

    rospy.set_param('~cartesian', True)
    # 定义所有给定的 ee_pose 为 Pose 对象
    positions = [
        [0, 0, 0, 0, 0, 0, 0],
        [0, -80, 0, 0, 0, 0, 0],
        [-70, -80, -7.5, -2.5, 0, 0, 0],
        [-80, -80, -15, -5, 0, -10, 0],
        [-65, -0, 0, -20, 85, 0, -5]
    ]
    ee_poses = []
    for position in positions:
        q = angle_to_rad(position)
        pose_stamped = arm_ifk.compute_fk(q)
        ee_poses.append(pose_stamped.pose)
        print(pose_stamped.pose)

    # 生成预设轨迹
    pre_defined_traj  = gen_pre_defined_traj(planner, positions)
    dump_traj(pre_defined_traj, file_name='pre_defined_traj')
    exit(0)
    
    planner.set_start_state([0.0]*7)
    fraction = 0.0
    attempts =  1
    while fraction < 1.0 and attempts < 100:
        traj,fraction = planner.compute_cartesian_path(ee_poses, 0.01)
        if fraction < 1.0:
            print("path fraction:", fraction)
        attempts += 1
        print("attempts:", attempts)
    dump_traj(traj, file_name='ready')
    
    
    # print("path:", traj)
    print("path len:", len(traj.joint_trajectory.points))
    planner.move_group.execute(traj, wait=True)

    ref_ee_pose_pub = EEPosePublisher(path_topic='ee_pose/refrence_path')
    real_ee_pose_pub = EEPosePublisher(path_topic='ee_pose/real_path')

    for pose in ee_poses:
        ref_ee_pose_pub.add_pose_to_path(pose)

    for p in traj.joint_trajectory.points:
        pose_stamped = arm_ifk.compute_fk(p.positions) 
        if pose_stamped is not None:
            real_ee_pose_pub.add_pose_to_path(pose_stamped.pose)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ref_ee_pose_pub.publish_path()
        real_ee_pose_pub.publish_path()
        rate.sleep()
          
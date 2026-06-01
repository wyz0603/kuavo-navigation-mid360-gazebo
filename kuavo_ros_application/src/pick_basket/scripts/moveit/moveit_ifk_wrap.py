#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest,GetPositionFK, GetPositionFKRequest

class MoveItArmIFKWrap:
    def __init__(self, joint_names, gourp_name='r_arm_group', ee_name='r_hand_end_virtual'):
        self._base_link  = 'torso'
        self._ee_name = ee_name
        self._joint_names = joint_names
        self._group_name = gourp_name
    def compute_ik(self, curr_joint_positions, target_ee_pose:Pose, timeout='0.5')->list:
        """
            Compute inverse kinematics for the given pose by MoveIt!.
            Input:
                curr_joint_positions: current joint positions
                target_ee_pose: target end effector pose
            Return the joint positions.
        """
        ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = self._group_name  # set move group name
        ik_request.ik_request.robot_state.joint_state.name = self._joint_names
        ik_request.ik_request.robot_state.joint_state.position = curr_joint_positions

        target_pose_stamped = PoseStamped()
        target_pose_stamped.pose = target_ee_pose
        ik_request.ik_request.pose_stamped = target_pose_stamped  # set target pose
        ik_request.ik_request.timeout = rospy.Duration(timeout)   # timeout

        try:
            ik_response = ik_service(ik_request)
            if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                left_arm_angles_rad   = ik_response.solution.joint_state.position[0:7]    # left arm angles
                right_arm_angles_rad  = ik_response.solution.joint_state.position[15:22]  # right arm angles
                if self._move_group_name == "r_arm_group":
                    return right_arm_angles_rad
                else:
                    return left_arm_angles_rad
            else:
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return None
    def compute_fk(self, joint_positions:list) -> Pose:
        """
            Compute forward kinematics for the given joint positions by MoveIt!.
            joint_positions: list of joint positions in radians
            Returns the pose of the end effector.
        """
        fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = self._base_link
        fk_request.fk_link_names = [self._ee_name]
        fk_request.robot_state = RobotState()
        fk_request.robot_state.joint_state.name = self._joint_names
        fk_request.robot_state.joint_state.position = joint_positions
        try:
            fk_response = fk_service(fk_request)
            if fk_response.error_code.val == fk_response.error_code.SUCCESS:
                return fk_response.pose_stamped[0]
            else:
                rospy.logerr("Forward kinematics failed with error code: %d", fk_response.error_code.val)
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return None

def compute_pose_distance(pose1, pose2):
    # 提取位置信息
    pos1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
    pos2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
    
    # 计算位置之间的欧几里得距离
    position_distance = np.linalg.norm(pos1 - pos2)
    # print("position_distance:", position_distance)
    return position_distance

def compute_distances_and_ratios(arm_ifk, traj_positions):
    distance = []
    pose0 = arm_ifk.compute_fk(traj_positions[0])
    for i in range(1, len(traj_positions)):
        current_pose = arm_ifk.compute_fk(traj_positions[i])
        # 计算与初始姿态的位置距离
        position_distance = compute_pose_distance(pose0.pose, current_pose.pose)
        pose0 = current_pose
        distance.append(position_distance)

    # 计算总距离
    total_distance = sum(distance)

    # 计算每段距离的占比
    distance_ratios = [round(d / total_distance, 4) for d in distance]

    return distance, distance_ratios

if __name__ == '__main__':
    rospy.init_node("moveit_ik_fk_node")
    q = [-1.0965052836790312, 0.1716298149421535, 1.7004252525707, 0.9599395069488317, -0.805659151005594, -1.1251157643834353, 0.1452965816143362]

    ready_q = np.radians([-35, -20, -5, -55, 50, -30, 10])
    joint_names = ["r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]

    arm_ifk = MoveItArmIFKWrap(joint_names)

    # print('q pose:', arm_ifk.compute_fk(q))

    # print('ready_q:', arm_ifk.compute_fk(ready_q))

    # handup q
    # [-0.9647150229452542, 0.0842447856642047, 0.40937596755597433, -1.1009105281380716, 1.1274042277565082, -0.04712274486708432, 0.5499051548839302],
    # side_q
    #  [-0.8893876161232286, -0.3700797135763496, 0.15009016787954954, -1.1728725175105728, 0.8181667016941533, -0.44551267914555653, 0.6698731900919114]

    traj1_q = [
        [np.radians(angle) for angle in [0,0,0,0,0,0,0]],
        [np.radians(angle) for angle in [-3,-10,-15,-15,20,-10,10]],
        [np.radians(angle) for angle in [-6,-20,-25,-25,25,-15,15]],
        [np.radians(angle) for angle in [-10,-40,-25,-25,25,-15,15]],
        [np.radians(angle) for angle in [-35,-40,-12,-30,5,-15,15]],
        [np.radians(angle) for angle in [-35,-20,-5,-55,50,-30,10]],
        # pick_q
        [-0.7897491828373483, 0.07511163721542863, 0.5625410963029764, -0.6986121491968701, 1.1532172151061697, -0.14139761073226245, -0.02667603626832312]
    ]

    traj2 = [
        # pick_q
        [-0.7897491828373483, 0.07511163721542863, 0.5625410963029764, -0.6986121491968701, 1.1532172151061697, -0.14139761073226245, -0.02667603626832312],
        # handup q
        [-0.9647150229452542, 0.0842447856642047, 0.40937596755597433, -1.1009105281380716, 1.1274042277565082, -0.04712274486708432, 0.5499051548839302],
        # side_q
        [-0.8893876161232286, -0.3700797135763496, 0.15009016787954954, -1.1728725175105728, 0.8181667016941533, -0.44551267914555653, 0.6698731900919114],
        [np.radians(angle) for angle in [-35,-20,-5,-55,50,-30,10]],
    ]

    putdown_traj_q = [
        [np.radians(angle) for angle in [-35,-20,-5,-55,50,-30,10]],
        # handup q
        [-0.8926199063238873, 0.08817960414874874, 0.4122775007384312, -1.0968617969134586, 1.161435969835282, -0.06470371743495543, 0.35274205277398535],
        # putdown q
        [-0.7800130620683714, 0.06455793704186484, 0.5232756253756788, -0.8197322973754142, 1.164584009664916, -0.12630311714873008, 0.08485045029075573],
    ]

    goback_q = [
         # handup q
        [-0.7800130620683714, 0.06455793704186484, 0.5232756253756788, -0.8197322973754142, 1.164584009664916, -0.12630311714873008, 0.08485045029075573], 
        [-0.8926199063238873, 0.08817960414874874, 0.4122775007384312, -1.0968617969134586, 1.161435969835282, -0.06470371743495543, 0.35274205277398535],
        [np.radians(angle) for angle in [-35,-20,-5,-55,50,-30,10]],
        [np.radians(angle) for angle in [-35,-40,-12,-30,5,-15,15]],
        [np.radians(angle) for angle in [-10,-40,-25,-25,25,-15,15]],
        [np.radians(angle) for angle in [-6,-20,-25,-25,25,-15,15]],
        [np.radians(angle) for angle in [-3,-10,-15,-15,20,-10,10]],
        [np.radians(angle) for angle in [0,0,0,0,0,0,0]],
    ]

    all_q = [
        [np.radians(angle) for angle in [0,0,0,0,0,0,0]],
        [np.radians(angle) for angle in [-3,-10,-15,-15,20,-10,10]],
        [np.radians(angle) for angle in [-6,-20,-25,-25,25,-15,15]],
        [np.radians(angle) for angle in [-10,-40,-25,-25,25,-15,15]],
        [np.radians(angle) for angle in [-35,-40,-12,-30,5,-15,15]],
        [np.radians(angle) for angle in [-35,-20,-5,-55,50,-30,10]],
        # pick_q
        [-0.7897491828373483, 0.07511163721542863, 0.5625410963029764, -0.6986121491968701, 1.1532172151061697, -0.14139761073226245, -0.02667603626832312]
        #.------------------------------------
        # handup q
        [-0.9647150229452542, 0.0842447856642047, 0.40937596755597433, -1.1009105281380716, 1.1274042277565082, -0.04712274486708432, 0.5499051548839302],
        # side_q
        [-0.8893876161232286, -0.3700797135763496, 0.15009016787954954, -1.1728725175105728, 0.8181667016941533, -0.44551267914555653, 0.6698731900919114],
        # ----------------------------------------
        [np.radians(angle) for angle in [-35,-20,-5,-55,50,-30,10]],
        # handup q
        [-0.8926199063238873, 0.08817960414874874, 0.4122775007384312, -1.0968617969134586, 1.161435969835282, -0.06470371743495543, 0.35274205277398535],
        # putdown q
        [-0.7800130620683714, 0.06455793704186484, 0.5232756253756788, -0.8197322973754142, 1.164584009664916, -0.12630311714873008, 0.08485045029075573],
        # ------------------------------------
         # handup q
        [-0.8926199063238873, 0.08817960414874874, 0.4122775007384312, -1.0968617969134586, 1.161435969835282, -0.06470371743495543, 0.35274205277398535],
        [np.radians(angle) for angle in [-35,-20,-5,-55,50,-30,10]],
        [np.radians(angle) for angle in [-35,-40,-12,-30,5,-15,15]],
        [np.radians(angle) for angle in [-10,-40,-25,-25,25,-15,15]],
        [np.radians(angle) for angle in [-6,-20,-25,-25,25,-15,15]],
        [np.radians(angle) for angle in [-3,-10,-15,-15,20,-10,10]],
        [np.radians(angle) for angle in [0,0,0,0,0,0,0]],
    ]
    distance, distance_ratios = compute_distances_and_ratios(arm_ifk, traj1_q)
    # print(f'traj1 distance: {distance}')
    print(f'traj1 distance ratios: {distance_ratios}')

    distance, distance_ratios = compute_distances_and_ratios(arm_ifk, traj2)
    # print(f'traj2 distance: {distance}')
    print(f'traj2 distance ratios: {distance_ratios}')

    distance, distance_ratios = compute_distances_and_ratios(arm_ifk, putdown_traj_q)
    # print(f'putdown_traj_q distance: {distance}')
    print(f'putdown_traj_q distance ratios: {distance_ratios}')

    distance, distance_ratios = compute_distances_and_ratios(arm_ifk, goback_q)
    # print(f'goback_q distance: {distance}')
    print(f'goback_q distance ratios: {distance_ratios}')

    distance, distance_ratios = compute_distances_and_ratios(arm_ifk, all_q)
    # print(f'all_q distance: {distance}')
    print(f'all_q distance ratios: {distance_ratios}')

"""
biped_s3 ready_q pose: 
  position: 
    x: 0.41283254624009835
    y: -0.3152970590934898
    z: 0.22502325827796238
  orientation: 
    x: -0.3277447211016064
    y: -0.6675616824694373
    z: 0.4495501530037256
    w: 0.49482265289993677

biped_s4 ready_q pose: 
  position: 
    x: 0.4010543565652259
    y: -0.29067401205405186
    z: 0.23674732892827302
  orientation: 
    x: -0.3277447211016064
    y: -0.6675616824694373
    z: 0.4495501530037256
    w: 0.49482265289993677    
"""
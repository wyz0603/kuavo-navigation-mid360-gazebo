#!/usr/bin/env python3
# -*- coding: UTF-8 -*-


import sys
import moveit_commander
import rospy
import geometry_msgs.msg

from planner import Planner
from logger import Logger
from publisher import Publisher

from utils import angle_to_rad
import tf
import numpy as np
import math

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# 获取目标姿态的四元数
def robot_euler_from_quaternion(orientation_quaternion):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_quaternion)

    roll_degrees = np.degrees(roll)
    pitch_degrees = np.degrees(pitch)
    yaw_degrees = np.degrees(yaw)

    print("Roll (degrees):", roll_degrees, "Pitch (degrees):", pitch_degrees, "Yaw (degrees):", yaw_degrees)

    return roll_degrees, pitch_degrees, yaw_degrees


Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])

def calculate_inverse_kinematics(target_pose_stamped):
    # 调用逆解服务
    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = 'l_arm_group'  # 设置运动规划组名称
    ik_request.ik_request.pose_stamped = target_pose_stamped  # 设置目标末端位姿
    ik_request.ik_request.timeout = rospy.Duration(0.2)  # 设置逆解计算时间限制

    try:
        # 发送逆解请求并等待响应
        ik_response = ik_service(ik_request)
        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            # 获取逆解结果
            return ik_response.solution.joint_state
        else:
            rospy.logerr("Inverse kinematics failed with error code: %d", ik_response.error_code.val)
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return None

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    planner = Planner()
    logger = Logger()
    publisher = Publisher()
    
    publisher.start_auto_publish()
    logger.make_traj_dir()

    print("=====================================================")

    # 做动作/正解末端的位姿 随手截取
    target_pose_stamped = PoseStamped()
    target_pose_stamped.header.frame_id = "torso"
    target_pose_stamped.pose.position.x = 0.12595975554344743
    target_pose_stamped.pose.position.y = 0.2546565360634217
    target_pose_stamped.pose.position.z = 0.03443182480649596

    target_pose_stamped.pose.orientation.x = -0.07733529679630219
    target_pose_stamped.pose.orientation.y = -0.4109196668427433
    target_pose_stamped.pose.orientation.z = -0.03449601648776949
    target_pose_stamped.pose.orientation.w = 0.9077303036242385

    # target_pose_stamped.pose.orientation.x = 0.0
    # target_pose_stamped.pose.orientation.y = -0.707
    # target_pose_stamped.pose.orientation.z = 0.0
    # target_pose_stamped.pose.orientation.w = 0.707
    
    print(" target_pose_stamped : ", target_pose_stamped)

    # 逆解出关节角度
    joint_angles = calculate_inverse_kinematics(target_pose_stamped)

    left_arm_angles_rad   = joint_angles.position[2:9]  # 左手的关节角度
    right_arm_angles_rad  = joint_angles.position[19:26]  # 右手的关节角度

    left_arm_angles_deg = [math.degrees(angle) for angle in left_arm_angles_rad]
    right_arm_angles_deg = [math.degrees(angle) for angle in right_arm_angles_rad]

    print("left_arm_angles_deg : ", left_arm_angles_deg)
    print("right_arm_angles_deg : ", right_arm_angles_deg)

    # 查看欧拉角
    orientation_quaternion = [
    target_pose_stamped.pose.orientation.x,
    target_pose_stamped.pose.orientation.y,
    target_pose_stamped.pose.orientation.z,
    target_pose_stamped.pose.orientation.w
    ]
    robot_euler_from_quaternion(orientation_quaternion)

    # 设置并且开启规划
    planner.set_start_state(Point_zero)
    traj = planner.plan_to_target_joints(left_arm_angles_rad)
    logger.dump_traj(traj, file_name="test1_moveit_point")

    # 结束自动发布
    publisher.stop_auto_publish()
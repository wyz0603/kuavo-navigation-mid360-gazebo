#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
    目标检测的结果
    当作pose
    然后逆解为joint
    然后用joint调取规划
"""
import sys
import rospy
import moveit_commander
import math
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import JointState
from planner import Planner
from logger import Logger
from publisher import Publisher

from utils import angle_to_rad

Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])

# 获取目标姿态的四元数
def robot_euler_from_quaternion(orientation_quaternion):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_quaternion)

    roll_degrees = np.degrees(roll)
    pitch_degrees = np.degrees(pitch)
    yaw_degrees = np.degrees(yaw)

    print("Roll (degrees):", roll_degrees, "Pitch (degrees):", pitch_degrees, "Yaw (degrees):", yaw_degrees)

    return roll_degrees, pitch_degrees, yaw_degrees

# 调用逆解服务
def calculate_inverse_kinematics(target_pose_stamped):
    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = 'l_arm_group'  # 设置运动规划组名称
    ik_request.ik_request.pose_stamped = target_pose_stamped  # 设置目标末端位姿
    ik_request.ik_request.timeout = rospy.Duration(0.2)  # 设置逆解计算时间限制

    try:
        ik_response = ik_service(ik_request)
        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            return ik_response.solution.joint_state
        else:
            rospy.logerr("Inverse kinematics failed with error code: %d", ik_response.error_code.val)
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return None

def detection_callback(msg):
    if not msg.detections:
        rospy.logwarn("No detections in message.")
        return

    # 提取目标检测信息（假设只处理第一个检测结果）
    detection = msg.detections[0]
    x = detection.results[0].pose.pose.position.x
    y = detection.results[0].pose.pose.position.y
    z = detection.results[0].pose.pose.position.z  

    target_pose_stamped = PoseStamped()
    target_pose_stamped.header.frame_id = "torso"
    target_pose_stamped.pose.position.x = x
    target_pose_stamped.pose.position.y = y
    target_pose_stamped.pose.position.z = z

    # 修改成斜着抓
    target_pose_stamped.pose.orientation.x = 0.0
    target_pose_stamped.pose.orientation.y = -0.707
    target_pose_stamped.pose.orientation.z = 0.0
    target_pose_stamped.pose.orientation.w = 0.707
    # print("target_pose_stamped : ", target_pose_stamped)
    
    # 逆解出关节角度
    joint_angles = calculate_inverse_kinematics(target_pose_stamped)
    if joint_angles is None:
        rospy.logerr("Failed to calculate inverse kinematics")
        return

    left_arm_angles_rad   = joint_angles.position[2:9]  # 左手的关节角度
    right_arm_angles_rad  = joint_angles.position[19:26]  # 右手的关节角度

    left_arm_angles_deg = [math.degrees(angle) for angle in left_arm_angles_rad]
    right_arm_angles_deg = [math.degrees(angle) for angle in right_arm_angles_rad]

    print("left_arm_angles_deg : ", left_arm_angles_deg)
    print("right_arm_angles_deg : ", right_arm_angles_deg)

    # 设置并且开启规划
    planner.set_start_state(Point_zero)
    traj = planner.plan_to_target_joints(left_arm_angles_rad)
    if traj:
        logger.dump_traj(traj, file_name="test1_moveit_point")
    else:
        rospy.logerr("Failed to plan trajectory")

if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    planner = Planner()
    logger = Logger()
    publisher = Publisher()

    # publisher.start_auto_publish()
    logger.make_traj_dir()

    print("=====================================================")

    # 订阅 /object_yolo_tf2_torso_result 话题
    rospy.Subscriber("/object_yolo_tf2_torso_result", Detection2DArray, detection_callback)

    # 设置频率为 10 Hz
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        # 在循环中处理回调
        rate.sleep()

    # 结束自动发布
    # publisher.stop_auto_publish()

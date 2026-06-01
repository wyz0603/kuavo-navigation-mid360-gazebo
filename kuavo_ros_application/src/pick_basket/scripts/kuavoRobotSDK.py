#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
import time
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamic_biped.msg import robotHandPosition
from dynamic_biped.msg import armTargetPoses
from pick_basket.srv import changeArmCtrlModeOCS2, changeArmCtrlModeOCS2Request
from pick_basket.srv import planArmTrajectoryCubicSpline, planArmTrajectoryCubicSplineRequest

class kuavo:
    def __init__(self, name):
        self.name = name
        self.arm_num = 14
        """ ROS """
        # 末端灵巧手控制
        self._pub_hand_pose = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
        # 手臂关节轨迹        
        self._pub_kuavo_arm_target = rospy.Publisher("/kuavo_arm_target_poses",armTargetPoses, queue_size=10)
        self._pub_arm_traj = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

        # 切换手臂模式
        self._arm_ctrl_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlModeOCS2)

    def _srv_changeArmCtrlMode(self, control_mode)->bool:
        try:
            request = changeArmCtrlModeOCS2Request()
            request.control_mode = control_mode

            response = self._arm_ctrl_mode_client(request)

            return response.result

        except rospy.ServiceException as e:
            rospy.logerr(f"changeArmCtrlMode Service call failed: {e}")
            return False
    def _pub_hand_position(self, left_hand_pos:list, right_hand_pos:list):
        msg = robotHandPosition()
        msg.left_hand_position = left_hand_pos
        msg.right_hand_position = right_hand_pos
        self._pub_hand_pose.publish(msg)

    def _pub_kuavo_arm_traj(self, traj_jointstate):
        arm_traj_msg = JointState()

        arm_traj_msg.position = traj_jointstate.position
        self._pub_arm_traj.publish(arm_traj_msg)
    def set_robot_arm_ctl_mode(self, control_mode)->bool:
        """ 切换手臂规划模式 
        :param control_mode: uint8, # 0: keep pose, 1: auto_swing_arm, 2: external_control 
        :return: bool, 服务调用结果
        """
        result = self._srv_changeArmCtrlMode(control_mode)
        rospy.loginfo(f"Service call /arm_traj_change_mode call mode:{control_mode}, result: {result}")
        return result

    def set_end_hand(self, left_hand_position, right_hand_position):
        """ 设置机器人的灵巧手
        :param left_hand_position: [uint8]*6
        :param right_hand_position:[uint8]*6 
        """
        self._pub_hand_position(left_hand_position, right_hand_position)

    def set_arm_traj_position(self, joint_positions:list):
        """ 发布手臂控制命令
        :param joint_positions: list 最终关节的位置
        """
        if len(joint_positions) == self.arm_num:
            arm_traj_msg = JointState()
            arm_traj_msg.header.stamp = rospy.Time.now()
            arm_traj_msg.position = joint_positions

            self._pub_kuavo_arm_traj(arm_traj_msg)
        else:
            rospy.logerr("Invalid number of joint positions provided.")

    def set_kuavo_arm_target_poses(self, times:list, values:list):
        arm_target_poses_msg = armTargetPoses()
        arm_target_poses_msg.times = times
        DEG_TO_RAD = 180 / math.pi
        for i in range(len(values)):
            rads = [v * DEG_TO_RAD for v in values[i]]
            arm_target_poses_msg.values.extend(rads)
        # print("times:", len(arm_target_poses_msg.times))
        # print("values:", len(arm_target_poses_msg.values))
        self._pub_kuavo_arm_target.publish(arm_target_poses_msg)

if __name__ == "__main__":
    rospy.init_node('kuavo_sdk_demo_node', anonymous=True)
    kuavo_robot = kuavo("kuavo_robot")
# ["r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]

    positions = [
        [0, 0, 0, 0, 0, 0, 0],
        [0, -80, 0, 0, 0, 0, 0],
        [-70, -80, -7.5, -2.5, 0, 0, 0],
        [-80, -80, -15, -5, 0, -10, 0],
        [-65, -0, 0, -20, 85, 0, -5]
    ]
    values = []
    times = []
    start_time = 2.0
    dt = 1.2
    for i in range(len(positions)):
        times.append(start_time + dt*i)
        q = (np.array(positions[i])/180*np.pi).tolist()
        values.append([0.0]*7 + q)

    kuavo_robot.set_robot_arm_ctl_mode(2)
    time.sleep(2.0)
    kuavo_robot.set_kuavo_arm_target_poses(times, values)
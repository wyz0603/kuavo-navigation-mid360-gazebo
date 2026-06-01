#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rospy
import copy
from common.utils import rpy_to_orientation
from common.config import Config
from control_hand import ControlHand
from control_arm import ControlArm
from bezier_curve_planner import (
    load_action_frames_file, last_q_from_action_frames, 
    pushback_action_frames_rad, dump_action_data_to_tact_file,
    pushfront_action_frames_rad, first_q_from_action_frames
)
from typing import Tuple
from geometry_msgs.msg import Pose, Point, Quaternion

class PickBasketService(object):
    def __init__(self):
        self._config = Config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                           '../config/config.json'))
        model_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                       self._config.model_file_path)
        
        self._hand_controller = ControlHand()
        self._arm_controller = ControlArm(model_file_path)
        
        """ Config Variables """
        self._holdon_pose = Pose(
            position=Point(x=0.411445, y=-0.299152, z=0.20872299999999996),
            orientation=Quaternion(x=-0.53, y=-0.49, z=0.468, w=0.506))
        
    def _calculate_pick_basket_arm_traj(self, ee_pose:Pose) \
        -> Tuple[bool, any, any]:
        """
            计算抓取篮子动作轨迹
            -----------
            pose: 抓取 Basket 的目标位置
            return: [bool, action_frames1, action_frames2]
        """
        result = [False, None, None]
        
        if ee_pose is None:
            return result

        ############################# traj1 pick #############################
        # Get the ready pose
        action_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self._config.ready_action_path)
        traj1_action_frames = load_action_frames_file(action_path)

        # 抓取轨迹: 从 ready 到 pick 
        ready_q = last_q_from_action_frames(traj1_action_frames)
        ready_q = ready_q[-7:]
        
        start_pose = Pose()
        start_pose.position = Point(0.40, -0.48, 1.11)
        start_pose.orientation = Quaternion(-0.3355, -0.76072, 0.15, 0.53)
        pick_pose = copy.deepcopy(ee_pose)
        success, pick_waypoints = self._arm_controller.get_interpolate_trajectory(ready_q, start_pose, pick_pose, 4.5, 5)
        if not success: # 逆解失败，返回None
            rospy.loginfo("calculate_pick_basket_arm_traj, get_interpolate_trajectory failed!")
            return result
        
        ############################# traj2 handup to side #############################
        # Get the ready pose
        action_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self._config.handdown_action_path)
        traj2_action_frames = load_action_frames_file(action_path)

        pick_q = pick_waypoints[-1][1]
        handup_pose = copy.deepcopy(ee_pose)
        handup_pose.position.z += self._config.handup_z_offset # 抬高（单位m）
        handup_q = self._arm_controller.IK(pick_q, handup_pose)
        if handup_q is None:
            rospy.loginfo("calculate_pushdown_basket_arm_traj, ik handup pose failed!")
            return result

        print("------------ calculate_pick_basket_arm_traj, handup_q: \n", handup_q)

        ############## Return
        # traj1:
        for i, (time_gap, q) in enumerate(pick_waypoints, start=1):
            pushback_action_frames_rad(action_frames=traj1_action_frames, time_gap=time_gap, q_target_rad= [0.0]*7 + q)
        
        # traj2:
        pushfront_action_frames_rad(action_frames=traj2_action_frames, time_gap=2.0, q_target_rad=[0.0]*7 + handup_q)
        pushfront_action_frames_rad(action_frames=traj2_action_frames, time_gap=2.0, q_target_rad=[0.0]*7 + pick_q)

        return True, traj1_action_frames, traj2_action_frames
    
    def pick_basket(self, pick_position:Point = Point(0.411445, -0.079152, 0.168723))->bool:
        """
            抓取桌上的篮子
            -----------    
        """
        pick_pose = Pose()
        pick_pose.position = pick_position
        pick_pose.orientation = rpy_to_orientation(self._config.pick_pose)
        
        print("------------ pick basket, pick_pose: \n", pick_pose)
     
        success, traj1_action_frames, traj2_action_frames = self._calculate_pick_basket_arm_traj(pick_pose)
        if success is False:
            rospy.logwarn("pick basket fail, calculate_arm_traj failed!")
            return False

        # 设置手臂控制模式：切换为外部控制
        self._arm_controller.change_arm_ctrl_mode(2)
        time.sleep(2.0)

        dump_action_data_to_tact_file(traj1_action_frames, 'pick1.tact')
        dump_action_data_to_tact_file(traj2_action_frames, 'pick2.tact')

        # 1. 执行轨迹1
        print("------------ pick basket: execute traj1 action frames")
        if self._arm_controller.execute_action_frames(traj1_action_frames) == False:
            rospy.logwarn("pick basket fail, execute traj1 action frame failed!")
            return False

        self._hand_controller.open(hand=ControlHand.Hand.Right)
        # 抓取
        time.sleep(0.5)
        self._hand_controller.pick(hand=ControlHand.Hand.Right)

        # 1. 执行轨迹2
        if self._arm_controller.execute_action_frames(traj2_action_frames) == False:
            rospy.logwarn("pick basket fail, execute traj2 action frame failed!")
            return False

        return True
    
    def _calculate_pushdown_basket_arm_traj(self, pose:Pose) \
        -> Tuple[bool, any, any]:
        pass
        result = [False, None, None]

        print("------------ calculate_pushdown_basket_arm_traj, pose: \n", pose)

        ############################# traj1  #############################
        action_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self._config.handup_action_path)
        traj1_action_frames = load_action_frames_file(action_path)

        # Get ready q 
        last_q = last_q_from_action_frames(traj1_action_frames)
        last_q = last_q[-7:]

        # handup 
        handup_pose = copy.deepcopy(pose)
        handup_pose.position.z += self._config.handup_z_offset - 0.00 # 抬高（单位m）
        handup_pose.orientation = rpy_to_orientation(self._config.handup_pose)
        print("------------ calculate_pushdown_basket_arm_traj, handup_pose: \n", handup_pose)
        handup_q = self._arm_controller.IK(last_q, handup_pose)
        if handup_q is None:
            rospy.loginfo("calculate_pushdown_basket_arm_traj, ik handup pose failed!")
            return result
        
        print("------------ calculate_pushdown_basket_arm_traj, handup_q: \n", handup_q)
        
        # pushdown
        pushdown_q = self._arm_controller.IK(handup_q, pose)
        if pushdown_q is None:
            rospy.loginfo("calculate_pushdown_basket_arm_traj, ik pushdown pose failed!")
            return result
        
        print("------------ calculate_pushdown_basket_arm_traj, pushdown_q: \n", pushdown_q)
        
        ############################# traj2  #############################
        action_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self._config.goback_action_path)
        traj2_action_frames = load_action_frames_file(action_path)

        # traj1
        pushback_action_frames_rad(action_frames=traj1_action_frames, time_gap=1.5, q_target_rad= [0.0]*7 + handup_q)
        pushback_action_frames_rad(action_frames=traj1_action_frames, time_gap=1.5, q_target_rad= [0.0]*7 + pushdown_q)

        # traj2 
        pushfront_action_frames_rad(action_frames=traj2_action_frames, time_gap=2.0, q_target_rad= [0.0]*7 + handup_q)
        pushfront_action_frames_rad(action_frames=traj2_action_frames, time_gap=1.3, q_target_rad= [0.0]*7 + pushdown_q)
        pushfront_action_frames_rad(action_frames=traj2_action_frames, time_gap=0.1, q_target_rad= [0.0]*7 + pushdown_q)

        # 返回放置轨迹和抬手归位轨迹
        return [True, traj1_action_frames, traj2_action_frames]
    
    def putdown_basket(self, putdown_position:Point = Point(0.431445, -0.079152, 0.188723))->bool:
        """
            放置篮子到桌子上
            pushdown 然后松开然后手臂归位
        """
        pushdown_pose = Pose() 
        pushdown_pose.position = copy.deepcopy(putdown_position)
        pushdown_pose.orientation = rpy_to_orientation(self._config.pick_pose)

        success, traj1_action_frames, traj2_action_frames = self._calculate_pushdown_basket_arm_traj(pushdown_pose)
        if success is False:
            rospy.logwarn("putdown basket fail, calculate_arm_traj failed!")
            return False
        
        # 设置手臂外部控制
        self._arm_controller.change_arm_ctrl_mode(2)
        time.sleep(1.0)

        dump_action_data_to_tact_file(traj1_action_frames, 'putdown1.tact')
        dump_action_data_to_tact_file(traj2_action_frames, 'putdown2.tact')

        # 1. 执行轨迹1
        print("------------ putdown basket: execute traj1 action frames")
        if self._arm_controller.execute_action_frames(traj1_action_frames) == False:
            rospy.logwarn("pick basket fail, execute traj1 action frames failed!")
            return False

        # 松开手
        time.sleep(0.5)
        self._hand_controller.release()

        # 2. 执行轨迹2
        print("------------ putdown basket: execute traj2 action frames")
        if self._arm_controller.execute_action_frames(traj2_action_frames) == False:
            rospy.logwarn("pick basket fail, execute traj2 action frames failed!")
            return False
        
        return True
    
    def is_graspable_position(self, q:Point):
        """
            是否可以抓取到该位置的篮子
            q ：末端执行器的位置
        """
        ee_pose = Pose()
        ee_pose.position.x = q.x
        ee_pose.position.y = q.y
        ee_pose.position.z = q.z
        ee_pose.orientation = rpy_to_orientation(self._config.pick_pose)
        # print("ee_pose: ", ee_pose)
        success, _, _ = self._calculate_pick_basket_arm_traj(ee_pose=ee_pose)
        return success

    def is_putdown_position(self, q:Point):
        """
            是否可以放置篮子到该位置
            q ：末端执行器的位置
        """
        ee_pose = Pose()
        ee_pose.position.x = q.x
        ee_pose.position.y = q.y
        ee_pose.position.z = q.z
        ee_pose.orientation = rpy_to_orientation(self._config.pick_pose)
        success, _, _ = self._calculate_pushdown_basket_arm_traj(pose=ee_pose)
        return success

    def arm_homing(self):
        """
            将手臂复位到初始位置
        """
        self._arm_controller.arm_reset()

    def set_arm_fixed(self):
        """
            固定手臂保持当前位置
        """
        # 手臂模式切换保持姿势
        self._arm_controller.change_arm_ctrl_mode(0) 

    def set_arm_flexible(self) -> bool:
        """
            手臂模式切换为灵活模式，行走时自动摆手
        """
        self._arm_controller.change_arm_ctrl_mode(1) 

if __name__ == "__main__":
    rospy.init_node("pick_basket_service_demo")
    pick_basket_service = PickBasketService()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 0.421445, -0.059152, 0.208723
        if pick_basket_service.pick_basket(pick_position = Point(0.421445, -0.059152, 0.178723)):
        # if pick_basket_service.putdown_basket(putdown_position = Point(0.431445, -0.079152, 0.20723)):
            time.sleep(1.5)
            # 放置回去
            pick_basket_service.putdown_basket(putdown_position = Point(0.431445, -0.059152, 0.17723)) 
            rate.sleep()
            rospy.signal_shutdown("Pick basket finished!")
            break
        rate.sleep()
    rospy.logwarn("Exiting...")
    time.sleep(3)    
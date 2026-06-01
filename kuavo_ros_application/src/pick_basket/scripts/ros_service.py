#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import rospy
import time
import copy
import numpy as np
from typing import Tuple
from enum import Enum
from pick_basket_service import PickBasketService
from close_to_tag import CloseToTagNode
from common.config import Config
from geometry_msgs.msg import Pose, Point, Quaternion
from pick_basket.srv import pickBasketSrv, pickBasketSrvResponse
from apriltag_ros.msg import AprilTagDetectionArray
from dynamic_biped.msg import robotHeadMotionData

class StateMachine:
    class State(Enum):
        """定义动作状态枚举"""
        Free = 0
        Picking = 1
        PuttingDown = 2
        
    def __init__(self):
        self._state = StateMachine.State.Free

    def is_free_state(self)->bool:
        return self._state == StateMachine.State.Free

    def reset_state(self):
        self._state = StateMachine.State.Free

    def get_state(self)->State:
        return self._state
    
    def set_state(self, state:State):
        self._state = state
        
class PickBasketRosService:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pick_basket_service_node')
        rospy.loginfo("Pick basket service is ready to handle requests.")

        self._pick_basket_service = PickBasketService()
        self._config = Config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                    '../config/config.json'))
        
        # TODO: 调试站立抓取期望位置
        expected_offset= self._config.stand_params["expected_offset"] # xyz
        threshold = [self._config.stand_params["x_threshold"], 
                     self._config.stand_params["y_threshold"], 
                     np.radians(self._config.stand_params["yaw_deg_threshold"])]
        
        self._robot_control = CloseToTagNode(tag_id=7, expected_offset=expected_offset, threshold=threshold)

        """ Config Variables """
        self._tag_pose = Pose()
        self._tag_pose.position = Point(0.431445, -0.079152, 0.218723)
        self._tag_pose.orientation = Quaternion(0, 0, 0, 1)
        self._has_apriltag = False
        self._need_detection_tag = False
        self._state_machine = StateMachine()

        """ ROS """
        # 订阅 apriltag 信息 NOTE: 每次都抓取同样的位置，不需要根据 tag 位置抓取
        self._sub_apriltag = rospy.Subscriber("/robot_tag_info", AprilTagDetectionArray, self._detection_callback)
        self.service = rospy.Service('pick_basket', pickBasketSrv, self.handle_pick_basket)
        self._head_traj_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)

    def control_head(self, yaw, pitch):
        """
        yaw: desired yaw angle in degrees
        pitch: desired pitch angle in degrees
        """    

        # Print the values
        print(f"[Tag] Head pitch: {pitch}")
        print(f"[Tag] Head yaw: {yaw}")

        # Clamp yaw to the range [-30, 30]
        yaw = max(-30.0, min(30.0, yaw))

        # Clamp pitch to the range [-25, 25]
        pitch = max(-25.0, min(25.0, pitch))

        # Create and populate the robotHeadMotionData message
        head_cmd = robotHeadMotionData()
        head_cmd.joint_data = [0.0, 0.0]  # Initialize with zeros
        head_cmd.joint_data[0] = yaw  # yaw in degrees
        head_cmd.joint_data[1] = pitch  # pitch in degrees

        # Publish the command
        self._head_traj_pub.publish(head_cmd)

    def robot_lookdown(self):
        """
            调整头部位置 低头
        """

        print("ROS service: Adjusting head position to low")
        self.control_head(self._config.head_orientation["yaw_deg"], self._config.head_orientation["pitch_deg"])

    def robot_head_reset(self):
        """
            调整头部位置 低头
        """

        print("ROS service: Adjusting head position to normal")
        self.control_head(0, 0)

    def _detection_callback(self, msg:AprilTagDetectionArray):
        """
            apriltag 检测回调函数
        """
        if not msg.detections:
            return
        
        target_detection = None

        for detection in msg.detections:
            # print("Detected valid object with ID:", detection.id[0])
            if detection.id[0] == self._config.tag_id:
                # rospy.loginfo("Detected object with ID: %d", detection.id[0])
                target_detection = detection
                target_detection.pose.pose.pose.position.x += self._config.xyz_offset[0]
                target_detection.pose.pose.pose.position.y += self._config.xyz_offset[1]
                target_detection.pose.pose.pose.position.z += self._config.xyz_offset[2]
                # 保存 apriltag 信息
                self._tag_pose = target_detection.pose.pose.pose
                self._has_apriltag = True
                break

        if target_detection is None:
            rospy.logwarn("未识别到 ID 为 %d 的 AprilTag 标签", self._config.tag_id)
            return
        
    def handle_pick_basket(self, req):
        res = pickBasketSrvResponse()
        res.result = True
        res.message = "success"
        # 根据请求执行动作
        if req.action == 0:    # 抓取
            rospy.loginfo(f"Attempting to pick basket with tag ID: {req.tag_id}")
            res.result, res.message = self.pick(tag_id=req.tag_id)
        elif req.action == 1:  # 放置
            rospy.loginfo(f"Attempting to place basket with tag ID: {req.tag_id}")
            res.result, res.message = self.pushdown(tag_id=req.tag_id)
        else:
            rospy.logerr("Unknown action requested.")
            res.result = False
            res.message = "Invalid action"
            
        return res

    def pick(self, tag_id:int) -> Tuple[bool, str]:
        if not self._state_machine.is_free_state():
            return False, "Robot is busy!"
        
        # 调整头部位置
        self.robot_lookdown()

        # 更新状态
        self._state_machine.set_state(StateMachine.State.Picking)
        
        # 靠近桌子， 到达可抓取范围
        self._robot_control.close_to_tag()
        rospy.loginfo(f"Attempting to pick basket, close to desktop finish!")
        
        # 抓取前判断是否可以抓取到
        # TODO: 实物调试得到最佳抓取位置
        pick_position = Point(0.411445, -0.079152, 0.16872) # 固定位置
        # pick_position = copy.deepcopy(self._tag_pose.position) # 实时根据标签位置抓取
        if not self._pick_basket_service.is_graspable_position(pick_position):
            self._state_machine.reset_state() #更新状态
            print("tag is not graspable, pick_pose:", pick_position)
            rospy.loginfo(f"Invalid position, can't pick it!")
            return False, "Invalid position, can't pick it!"

        # 抓取篮子
        if not self._pick_basket_service.pick_basket(pick_position):
            self._state_machine.reset_state() #更新状态
            self._robot_control.backward_specify_distance(-0.5) # 往后退！
            rospy.loginfo(f"Invalid position, can't pick it!")
            return False, "pick basket failed!"
        
        # 固定手臂
        rospy.loginfo(f"set arm fixed ,keep it!")    
        self._pick_basket_service.set_arm_fixed()
        time.sleep(0.5)

        self.robot_head_reset() # 回正

        rospy.loginfo(f"Attempting to pick basket, pick finish, backward!")
        self._robot_control.backward_specify_distance(-0.5) # 往后退！
        self._state_machine.reset_state() #更新状态
        return True, "success"
        
    def pushdown(self, tag_id:int) -> Tuple[bool, str]:
        if not self._state_machine.is_free_state():
            return False, "Robot is busy!"
        
        # 调整头部位置
        self.robot_lookdown()

        # 更新状态
        self._state_machine.set_state(StateMachine.State.PuttingDown)

        # 靠近桌子， 到达可抓取范围
        self._robot_control.close_to_tag()
        rospy.loginfo(f"Attempting to pushdown basket, close to desktop finish!")
        
        # TODO: 实物调试得到最佳放下位置
        putdown_position = Point(0.431445, -0.079152, 0.20723) # 固定位置
        if not self._pick_basket_service.is_putdown_position(putdown_position):
            self._state_machine.set_state(StateMachine.State.Free) #更新状态
            return False, "Invalid position, can't pushdown it!"

        # 放置篮子
        if self._pick_basket_service.putdown_basket(putdown_position=putdown_position) == False:
            self._robot_control.backward_specify_distance(-0.5) # 往后退！
            self._state_machine.reset_state() #更新状态
            return False, "pushdown basket failed!"

        rospy.loginfo(f"Attempting to pushdown basket, pushdown finish, backward!")
        self._robot_control.backward_specify_distance(-0.5) # 往后退！
        self._state_machine.reset_state() #更新状态

        # 手臂控制恢复摆手
        self._pick_basket_service.set_arm_flexible()

        self.robot_head_reset() # 回正

        return True, "success"

if __name__ == "__main__":
    try:
        node = PickBasketRosService()
        time.sleep(2.0)
        node.robot_lookdown()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
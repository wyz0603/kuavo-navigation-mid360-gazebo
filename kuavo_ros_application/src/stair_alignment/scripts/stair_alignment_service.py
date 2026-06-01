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
from common.config import Config
from stair_close_to_tag import StairCloseToTagNode
from geometry_msgs.msg import Pose, Point, Quaternion
from stair_alignment.srv import stairAlignmentSrv, stairAlignmentSrvResponse
from stair_alignment.msg import StairAlignmentStatus
from apriltag_ros.msg import AprilTagDetectionArray
from dynamic_biped.msg import robotHeadMotionData

class StateMachine:
    class State(Enum):
        """定义动作状态枚举"""
        Free = 0
        Aligning = 1
        
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
        
class StairAlignmentRosService:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('stair_alignment_service_node')
        rospy.loginfo("Stair alignment service is ready to handle requests.")

        self._config = Config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                    '../config/config.json'))
        
        # 初始化机器人控制节点
        expected_offset = self._config.stand_params["expected_offset"] # xyz
        threshold = [self._config.stand_params["x_threshold"], 
                     self._config.stand_params["y_threshold"], 
                     np.radians(self._config.stand_params["yaw_deg_threshold"])]
        
        self._robot_control = StairCloseToTagNode(
            tag_id=self._config.tag_id, 
            expected_offset=expected_offset, 
            threshold=threshold
        )

        """ Config Variables """
        self._tag_pose = Pose()
        self._has_apriltag = False
        self._need_detection_tag = False
        self._state_machine = StateMachine()
        
        # 日志时间控制
        self._last_detection_log_time = 0
        self._detection_log_interval = 3.0  # 3秒打印一次

        """ ROS """
        # 订阅 apriltag 信息
        self._sub_apriltag = rospy.Subscriber("/robot_tag_info", AprilTagDetectionArray, self._detection_callback)
        self.service = rospy.Service('stair_alignment', stairAlignmentSrv, self.handle_stair_alignment)
        self._head_traj_pub = rospy.Publisher("/robot_head_motion_data", robotHeadMotionData, queue_size=10)
        self._status_pub = rospy.Publisher("/stair_alignment_status", StairAlignmentStatus, queue_size=10)
        
        # 获取launch文件参数
        self._launch_tag_id = rospy.get_param('~tag_id', self._config.tag_id)
        self._launch_offset_x = rospy.get_param('~offset_x', self._config.stand_params["expected_offset"][0])
        self._launch_offset_y = rospy.get_param('~offset_y', self._config.stand_params["expected_offset"][1])
        self._launch_offset_yaw = rospy.get_param('~offset_yaw', self._config.stand_params["expected_offset"][2])
        
        rospy.loginfo(f"Launch parameters - tag_id: {self._launch_tag_id}, offsets: [{self._launch_offset_x}, {self._launch_offset_y}, {self._launch_offset_yaw}]")

    def publish_status(self, tag_id, current_state, current_x=0.0, current_y=0.0, current_yaw=0.0, 
                      target_x=0.0, target_y=0.0, target_yaw=0.0, step_count=0, total_steps=0, 
                      message="", is_aligned=False):
        """发布楼梯对齐状态信息"""
        status_msg = StairAlignmentStatus()
        status_msg.tag_id = tag_id
        status_msg.current_state = current_state
        status_msg.current_x = current_x
        status_msg.current_y = current_y
        status_msg.current_yaw = current_yaw
        status_msg.target_x = target_x
        status_msg.target_y = target_y
        status_msg.target_yaw = target_yaw
        status_msg.step_count = step_count
        status_msg.total_steps = total_steps
        status_msg.message = message
        status_msg.is_aligned = is_aligned
        
        self._status_pub.publish(status_msg)
        rospy.loginfo(f"Status published: {current_state} - {message}")

    def control_head(self, yaw, pitch):
        """
        yaw: desired yaw angle in degrees
        pitch: desired pitch angle in degrees
        """   

        # Print the values
        rospy.loginfo(f"[StairAlignment] Head pitch: {pitch}")
        rospy.loginfo(f"[StairAlignment] Head yaw: {yaw}")

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
            调整头部位置 低头以便识别楼梯上的AprilTag
        """
        rospy.loginfo("StairAlignment: Adjusting head position for stair tag detection")
        self.control_head(self._config.head_orientation["yaw_deg"], self._config.head_orientation["pitch_deg"])

    def robot_head_reset(self):
        """
            调整头部位置 回正
        """
        rospy.loginfo("StairAlignment: Adjusting head position to normal")
        self.control_head(0, 0)

    def _detection_callback(self, msg:AprilTagDetectionArray):
        """
            apriltag 检测回调函数
        """
        if not msg.detections:
            return
        
        target_detection = None
        current_time = time.time()

        for detection in msg.detections:
            if detection.id[0] == self._config.tag_id:
                # 控制日志打印频率，每3秒打印一次
                if current_time - self._last_detection_log_time >= self._detection_log_interval:
                    rospy.loginfo("StairAlignment: Detected stair tag with ID: %d", detection.id[0])
                    self._last_detection_log_time = current_time
                
                target_detection = detection
                target_detection.pose.pose.pose.position.x += self._config.xyz_offset[0]
                target_detection.pose.pose.pose.position.y += self._config.xyz_offset[1]
                target_detection.pose.pose.pose.position.z += self._config.xyz_offset[2]
                # 保存 apriltag 信息
                self._tag_pose = target_detection.pose.pose.pose
                self._has_apriltag = True
                break

        if target_detection is None:
            # 控制警告日志打印频率，每3秒打印一次
            if current_time - self._last_detection_log_time >= self._detection_log_interval:
                rospy.logwarn("StairAlignment: 未识别到 ID 为 %d 的楼梯 AprilTag 标签", self._robot_control._tag_id)
                self._last_detection_log_time = current_time
            return
        
    def handle_stair_alignment(self, req):
        # 使用请求中的参数，如果没有则使用launch文件参数
        tag_id = req.tag_id if req.tag_id != 0 else self._launch_tag_id
        offset_x = req.offset_x if req.offset_x != 0 else self._launch_offset_x
        offset_y = req.offset_y if req.offset_y != 0 else self._launch_offset_y
        offset_yaw = req.offset_yaw if req.offset_yaw != 0 else self._launch_offset_yaw
        
        rospy.loginfo(f"StairAlignment: Request received - tag_id: {tag_id}, offsets: [{offset_x}, {offset_y}, {offset_yaw}]")
        
        # 执行楼梯对齐动作
        result, message = self.align_to_tag(tag_id=tag_id, offset_x=offset_x, offset_y=offset_y, offset_yaw=offset_yaw)
        
        # 创建响应对象并设置结果
        res = stairAlignmentSrvResponse()
        res.result = result
        res.message = message
            
        return res

    def align_to_tag(self, tag_id:int, offset_x:float, offset_y:float, offset_yaw:float) -> Tuple[bool, str]:
        if not self._state_machine.is_free_state():
            self.publish_status(tag_id, "failed", message="Robot is busy!")
            return False, "Robot is busy!"
        
        # 发布开始状态
        self.publish_status(tag_id, "detecting", message="Starting stair alignment process")
        
        # 调整头部位置以便识别楼梯上的标签
        self.robot_lookdown()
        time.sleep(1.0)  # 等待头部调整完成

        # 更新状态
        self._state_machine.set_state(StateMachine.State.Aligning)
        
        # 更新目标标签ID和偏移参数
        self._robot_control._tag_id = tag_id
        self._robot_control._expected_offset = [offset_x, offset_y, offset_yaw]
        self._config.tag_id = tag_id
        
        try:
            # 发布开始对齐状态
            self.publish_status(tag_id, "aligning", target_x=offset_x, target_y=offset_y, target_yaw=offset_yaw, 
                              message="Starting alignment to stair tag")
            
            # 使用单步控制靠近目标标签
            success = self._robot_control.close_to_tag_with_status(self._status_pub, tag_id, offset_x, offset_y, offset_yaw)
            
            if success:
                # 发布完成状态
                self.publish_status(tag_id, "completed", target_x=offset_x, target_y=offset_y, target_yaw=offset_yaw,
                                  message="Successfully aligned to stair tag", is_aligned=True)
                rospy.loginfo("StairAlignment: Successfully aligned to stair tag!")
                
                # 保持对齐状态一段时间
                time.sleep(2.0)
            else:
                # 发布失败状态
                self.publish_status(tag_id, "failed", target_x=offset_x, target_y=offset_y, target_yaw=offset_yaw,
                                  message="Failed to align to stair tag")
                rospy.logwarn("StairAlignment: Failed to align to stair tag")
            
            # 回正头部
            self.robot_head_reset()
            
            # 重置状态
            self._state_machine.reset_state()
            
            return success, "Successfully aligned to stair tag" if success else "Failed to align to stair tag"
            
        except Exception as e:
            rospy.logerr(f"StairAlignment: Error during alignment: {str(e)}")
            self.publish_status(tag_id, "failed", message=f"Alignment failed: {str(e)}")
            self._state_machine.reset_state()
            self.robot_head_reset()
            return False, f"Alignment failed: {str(e)}"

if __name__ == "__main__":
    try:
        node = StairAlignmentRosService()
        time.sleep(2.0)
        node.robot_lookdown()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#! /usr/bin/env python3
# coding: utf-8

import copy
import time
import numpy as np
from collections import deque
from typing import Tuple, Optional
from transforms3d import quaternions, euler

from kuavo_humanoid_sdk.kuavo.core.ros.param import make_robot_param, EndEffectorType
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.interfaces.data_types import (AprilTagData, AprilTagDetection)
import roslibpy
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK

class KuavoRobotVisionCoreWebsocket:
    """Handles vision-related data processing for Kuavo humanoid robot.
    
    Attributes:
        tf_client (roslibpy.Topic): TF client for coordinate transformations
        tf_publisher (roslibpy.Topic): TF publisher for broadcasting transforms
    """
    
    def __init__(self):
        """Initializes vision system components including TF and AprilTag subscribers."""
        if not hasattr(self, '_initialized'):
            # Initialize WebSocket connection
            self.websocket = WebSocketKuavoSDK()
            
            # Initialize AprilTag subscribers
            self._apriltag_data_camera_sub = roslibpy.Topic(
                self.websocket.client,
                '/tag_detections',
                'apriltag_ros/AprilTagDetectionArray'
            )
            self._apriltag_data_camera_sub.subscribe(self._apriltag_data_callback_camera)
            
            self._apriltag_data_base_sub = roslibpy.Topic(
                self.websocket.client,
                '/robot_tag_info',
                'apriltag_ros/AprilTagDetectionArray'
            )
            self._apriltag_data_base_sub.subscribe(self._apriltag_data_callback_base)

            self._apriltag_data_odom_sub = roslibpy.Topic(
                self.websocket.client,
                '/robot_tag_info_odom',
                'apriltag_ros/AprilTagDetectionArray'
            )
            self._apriltag_data_odom_sub.subscribe(self._apriltag_data_callback_odom)

            # Initialize data structures
            self._apriltag_data_from_camera = AprilTagData(
                id = [],
                size = [],
                pose = []
            )

            self._apriltag_data_from_base = AprilTagData(
                id = [],
                size = [],
                pose = []
            )

            self._apriltag_data_from_odom = AprilTagData(
                id = [],
                size = [],
                pose = []
            )
            
            self._initialized = True

    def _tf_callback(self, msg):
        """Callback for TF messages.
        
        Args:
            msg: TF message containing transforms
        """
        for transform in msg['transforms']:
            key = (transform['header']['frame_id'], transform['child_frame_id'])
            self._transforms[key] = transform

    def _apriltag_data_callback_camera(self, data):
        """Callback for processing AprilTag detections from camera.
        
        Args:
            data (dict): Raw detection data from camera
        """
        # 清空之前的数据
        self._apriltag_data_from_camera.id = []
        self._apriltag_data_from_camera.size = []
        self._apriltag_data_from_camera.pose = []
        
        # 处理每个标签检测
        for detection in data['detections']:
            # 添加ID
            for id in detection['id']:
                self._apriltag_data_from_camera.id.append(id)
            
            # 添加尺寸
            if detection.get('size') and len(detection['size']) > 0:
                self._apriltag_data_from_camera.size.append(detection['size'][0])
            else:
                self._apriltag_data_from_camera.size.append(0.0)
            
            
            # Convert pose dict to AprilTagDetection
            pose_dict = detection['pose']['pose']['pose']

            tag_detection = AprilTagDetection(
                position=AprilTagDetection.Point(
                    x=float(pose_dict['position']['x']), 
                    y=float(pose_dict['position']['y']), 
                    z=float(pose_dict['position']['z'])
                    ), 
                orientation=AprilTagDetection.Quaternion(
                    x=float(pose_dict['orientation']['x']), 
                    y=float(pose_dict['orientation']['y']), 
                    z=float(pose_dict['orientation']['z']), 
                    w=float(pose_dict['orientation']['w'])
                ))
            # 添加姿态
            self._apriltag_data_from_camera.pose.append(tag_detection)
            
    def _apriltag_data_callback_base(self, data):
        """Callback for processing AprilTag detections from base link.
        
        Args:
            data (dict): Raw detection data from base frame
        """
        # 清空之前的数据
        self._apriltag_data_from_base.id = []
        self._apriltag_data_from_base.size = []
        self._apriltag_data_from_base.pose = []
        
        # 处理每个标签检测
        for detection in data['detections']:
            # 添加ID
            for id in detection['id']:
                self._apriltag_data_from_base.id.append(id)
            
            # 添加尺寸
            if detection.get('size') and len(detection['size']) > 0:
                self._apriltag_data_from_base.size.append(detection['size'][0])
            else:
                self._apriltag_data_from_base.size.append(0.0)
            
                        # Convert pose dict to AprilTagDetection
            pose_dict = detection['pose']['pose']['pose']
            
            tag_detection = AprilTagDetection(
                position=AprilTagDetection.Point(
                    x=float(pose_dict['position']['x']), 
                    y=float(pose_dict['position']['y']), 
                    z=float(pose_dict['position']['z'])
                    ), 
                orientation=AprilTagDetection.Quaternion(
                    x=float(pose_dict['orientation']['x']), 
                    y=float(pose_dict['orientation']['y']), 
                    z=float(pose_dict['orientation']['z']), 
                    w=float(pose_dict['orientation']['w'])
                ))
            
            # 添加姿态
            self._apriltag_data_from_base.pose.append(tag_detection)
 
    def _apriltag_data_callback_odom(self, data):
        """Callback for processing AprilTag detections from odom frame.
        
        Args:
            data (dict): Raw detection data from odom frame
        """
        # 清空之前的数据
        self._apriltag_data_from_odom.id = []
        self._apriltag_data_from_odom.size = []
        self._apriltag_data_from_odom.pose = []
        
        # 处理每个标签检测
        for detection in data['detections']:
            # 添加ID
            for id in detection['id']:
                self._apriltag_data_from_odom.id.append(id)
            
            # 添加尺寸
            if detection.get('size') and len(detection['size']) > 0:
                self._apriltag_data_from_odom.size.append(detection['size'][0])
            else:
                self._apriltag_data_from_odom.size.append(0.0)
            
            # Convert pose dict to AprilTagDetection
            pose_dict = detection['pose']['pose']['pose']
            
            tag_detection = AprilTagDetection(
                position=AprilTagDetection.Point(
                    x=float(pose_dict['position']['x']), 
                    y=float(pose_dict['position']['y']), 
                    z=float(pose_dict['position']['z'])
                    ), 
                orientation=AprilTagDetection.Quaternion(
                    x=float(pose_dict['orientation']['x']), 
                    y=float(pose_dict['orientation']['y']), 
                    z=float(pose_dict['orientation']['z']), 
                    w=float(pose_dict['orientation']['w'])
                ))
            
            # 添加姿态
            self._apriltag_data_from_odom.pose.append(tag_detection)

    @property 
    def apriltag_data_from_camera(self) -> AprilTagData:
        """AprilTag detection data in camera coordinate frame.
        
        Returns:
            AprilTagData: Contains lists of tag IDs, sizes and poses
        """
        return self._apriltag_data_from_camera
    
    @property
    def apriltag_data_from_base(self) -> AprilTagData:
        """AprilTag detection data in base_link coordinate frame.
        
        Returns:
            AprilTagData: Contains lists of tag IDs, sizes and poses
        """
        return self._apriltag_data_from_base
    
    @property
    def apriltag_data_from_odom(self) -> AprilTagData:
        """AprilTag detection data in odom coordinate frame.
        
        Returns:
            AprilTagData: Contains lists of tag IDs, sizes and transformed poses
        """
        return self._apriltag_data_from_odom

    def _get_data_by_id(self, target_id: int, data_source: str = "base") -> Optional[dict]:
        """Retrieves AprilTag data by specific ID from selected source.
        
        Args:
            target_id (int): AprilTag ID to search for
            data_source (str): Data source selector, valid options: 
                "camera", "base", "odom"
        
        Returns:
            Optional[dict]: Dictionary containing 'sizes' and 'poses' lists if found,
                None if no matching data
        """
        data_map = {
            "camera": self._apriltag_data_from_camera,
            "base": self._apriltag_data_from_base,
            "odom": self._apriltag_data_from_odom
        }
        
        if data_source not in data_map:
            raise ValueError(f"Invalid data source: {data_source}")
            
        data = data_map[data_source]
        
        # 查找匹配的ID
        try:
            idx = data.id.index(target_id)
            return {
                'sizes': [data.size[idx]],
                'poses': [data.pose[idx]]
            }
        except ValueError:
            return None

# if __name__ == "__main__":

#     kuavo_robot_vision_core = KuavoRobotVisionCoreWebsocket()
#     time.sleep(5)
#     print("apriltag_data_from_camera:")
#     print(kuavo_robot_vision_core.apriltag_data_from_camera)
#     print("apriltag_data_from_base:")
#     print(kuavo_robot_vision_core.apriltag_data_from_base)
#     print("apriltag_data_from_odom:")
#     print(kuavo_robot_vision_core.apriltag_data_from_odom)
#     rospy.spin()
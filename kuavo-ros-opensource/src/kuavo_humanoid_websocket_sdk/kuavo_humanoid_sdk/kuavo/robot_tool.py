#!/usr/bin/env python3
# coding: utf-8
import math
from kuavo_humanoid_sdk.common.logger import SDKLogger
from kuavo_humanoid_sdk.interfaces.data_types import (PoseQuaternion, HomogeneousMatrix)
from kuavo_humanoid_sdk.kuavo.core.ros.tools import KuavoRobotToolsCoreWebsocket
from typing import Union

class KuavoRobotTools:
    """机器人工具类,提供坐标系转换接口。
    
    该类封装了不同机器人坐标系之间的坐标变换查询功能,支持多种返回数据格式。
    """
    
    def __init__(self):
        self.tools_core = KuavoRobotToolsCoreWebsocket()

    def get_tf_transform(self, target_frame: str, source_frame: str, 
                       return_type: str = "pose_quaternion") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """获取指定坐标系之间的变换。
        
        Args:
            target_frame (str): 目标坐标系名称
            source_frame (str): 源坐标系名称
            return_type (str, optional): 返回数据格式类型。有效值: \n
                - "pose_quaternion" : 四元数姿态格式,  \n
                - "homogeneous" : 齐次矩阵格式。默认为"pose_quaternion"。\n
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: 
                指定格式的变换数据,如果失败则返回None
        
        Raises:
            ValueError: 如果提供了无效的 return_type
        """
        return self.tools_core._get_tf_tree_transform(target_frame, source_frame, return_type=return_type)

    def get_base_to_odom(self, return_type: str = "pose_quaternion") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """获取从base_link到odom坐标系的变换。
        
        Args:
            return_type (str, optional): 返回格式类型。与get_tf_transform相同，默认为"pose_quaternion"。
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: 变换数据或 None
        """
        return self.get_tf_transform("odom", "base_link", return_type)

    def get_camera_to_base(self, return_type: str = "homogeneous") -> Union[PoseQuaternion, HomogeneousMatrix, None]:
        """获取从camera_link到base_link坐标系的变换。
        
        Args:
            return_type (str, optional): 返回格式类型。与get_tf_transform相同，默认为"homogeneous"。
        
        Returns:
            Union[PoseQuaternion, HomogeneousMatrix, None]: 变换数据或None
        """
        return self.get_tf_transform("base_link", "camera_link", return_type)

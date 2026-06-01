#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import geometry_msgs.msg

from base import Base


class Scene(Base):
    """ 仿真环境管理器 """
    
    _instance = None
    def __new__(cls, *args, **kwargs):
        """ 单实例模式 """
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance
    
    
    def __init__(self) -> None:
        pass
    
    
    def init_scene(self) -> None:
        """ 初始化场景 """
        self.scene.remove_attached_object(self._eef_link)
        self.scene.remove_world_object()
    
    
    def _build_pose_stamped(self, pose: geometry_msgs.msg.Pose):
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = self._planning_frame
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose
        
        return pose_stamped
    
    
    def add_box(
        self,
        pose: geometry_msgs.msg.Pose,
        name: str = "box",
        size: tuple=(1, 1, 1)
    ) -> None:
        """添加长方体
        
        :param pose: 位置
        :param name: 物体名
        :param size: 大小
        """
        pose = self._build_pose_stamped(pose)
        self._scene.add_box(name, pose, size)
    
    
    def add_cylinder(
        self,
        pose: geometry_msgs.msg.Pose,
        name: str = "cylinder",
        height: float = 0.2,
        radius: float = 0.02
    ) -> None:
        """添加圆柱体
        
        :param pose: 位置
        :param name: 物体名
        :param height: 高度
        :param radius: 半径
        """
        pose = self._build_pose_stamped(pose)
        self._scene.add_cylinder(name, pose, height, radius)
    
    
    def add_sphere(
        self,
        pose: geometry_msgs.msg.Pose,
        name: str = "sphere",
        radius: float = 1
    ) -> None:
        """添加圆
        
        :param pose: 位置
        :param name: 物体名
        :param radius: 半径
        """
        pose = self._build_pose_stamped(pose)
        self._scene.add_sphere(name, pose, radius)

    
    
    def add_plane(
        self,
        pose: geometry_msgs.msg.Pose,
        name: str = "plane",
        normal: tuple = (0, 0, 1),
        offset: float = 0
    ) -> None:
        """添加平面
        
        :param pose: 位置
        :param name: 物体名
        :param normal: 法向量
        :param offset: 偏移
        """
        pose = self._build_pose_stamped(pose)
        self._scene.add_plane(name, pose, normal, offset)
    
    
    def add_mesh(
        self,
        pose: geometry_msgs.msg.Pose,
        name: str,
        path: str,
        size: tuple = (1, 1, 1)
    ) -> None:
        """添加自定义碰撞体
        
        :param pose: 位置
        :param name: 物体名
        :param path: 路径
        :param size: 大小
        """
        pose = self._build_pose_stamped(pose)
        self._scene.add_mesh(name, pose, path, size)
        
    
    
    def remove_object(self, object_name: str) -> None:
        """ 移除物体 """
        self.scene.remove_world_object(object_name)
    
    
#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import numpy as np
import geometry_msgs.msg
from tf.transformations import quaternion_matrix, quaternion_from_matrix


class Transformer(object):
    """ 坐标变换器 """
    
    _instance = None
    def __new__(cls, *args, **kwargs):
        """ 单实例模式 """
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance
    
    
    def __init__(self) -> None:
        pass
    
    
    def matrix_transform(self, matrix_1: list, matrix_2: list) -> list:
        """ 坐标变换 """
        return matrix_1 @ matrix_2
    
    
    def extract_x_rotation_from_matrix(self, matrix: list) -> list:
        """ 提取z轴旋转 """
        theta = -np.arcsin(matrix[2, 1])
        matrix[:3, :3] = [
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ]

        return matrix
    
    
    def extract_y_rotation_from_matrix(self, matrix: list) -> list:
        """ 提取z轴旋转 """
        theta = np.arctan2(matrix[2, 0], matrix[0, 0])
        matrix[:3, :3] = [
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ]

        return matrix
    
    
    def extract_z_rotation_from_matrix(self, matrix: list) -> list:
        """ 提取z轴旋转 """
        theta = np.arctan2(matrix[1, 0], matrix[0, 0])
        matrix[:3, :3] = [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ]

        return matrix
    
    def pose_to_matrix(self, pose: geometry_msgs.msg.Pose) -> list:
        """姿态转换为变换矩阵

        :param pose: 姿态
        :return: 变换矩阵
        """
        matrix = quaternion_matrix([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        matrix[:-1, -1] = [
            pose.position.x,
            pose.position.y,
            pose.position.z
        ]

        return matrix
    
    
    def matrix_to_pose(self, matrix: list) -> geometry_msgs.msg.Pose:
        """变换矩阵转换为姿态

        :param matrix: 变换矩阵
        :return: 姿态
        """
        pose = geometry_msgs.msg.Pose()

        quaternion = quaternion_from_matrix(matrix)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        pose.position.x = matrix[0, -1]
        pose.position.y = matrix[1, -1]
        pose.position.z = matrix[2, -1]

        return pose
    
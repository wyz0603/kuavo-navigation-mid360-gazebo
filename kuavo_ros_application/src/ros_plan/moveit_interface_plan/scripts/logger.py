#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import time
import json
import rospy
import moveit_msgs.msg
import rospy_message_converter.json_message_converter


class Logger(object):
    """ 记录器 """
    
    _instance = None
    def __new__(cls, *args, **kwargs):
        """ 单实例模式 """
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance
    
    
    def __init__(self) -> None:
        self._dir_name = None
        self._count = 1
    
    
    @staticmethod
    def get_current_timestamp() -> str:
        """ 获取当前时间戳 """
        return time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())


    def make_traj_dir(self) -> None:
        """ 创建日志目录 """
        dir_name = os.path.join(os.path.join(
            os.path.dirname(__file__), "traj", self.get_current_timestamp()
            )
        )
        self._count = 1
        self._dir_name = dir_name
        os.mkdir(dir_name)
        rospy.loginfo("创建轨迹日志目录{}".format(dir_name))


    def dump_traj(
        self,
        traj: moveit_msgs.msg.RobotTrajectory,
        file_name: str = None
    ) -> None:
        """ 存入轨迹 """
        traj = rospy_message_converter.json_message_converter.convert_ros_message_to_json(traj)
        if file_name:
            file_name = file_name + ".json"
        else:
            file_name = "traj_" + str(self._count) + ".json"
            self._count += 1
        path = os.path.join(self._dir_name, file_name)
        with open(path, "w") as f:
            json.dump(traj, f)
        rospy.loginfo("轨迹保存到{}".format(path))


    def load_traj(
        self,
        path: str
    ) -> moveit_msgs.msg.RobotTrajectory:
        """ 加载轨迹 """
        path = os.path.join(os.path.dirname(__file__), path)
        with open(path, "r") as f:
            traj = json.load(f)
        traj = rospy_message_converter.json_message_converter.convert_json_to_ros_message(
            "moveit_msgs/RobotTrajectory",
            traj
        )
        rospy.loginfo("轨迹从{}中加载".format(path))
        
        return traj


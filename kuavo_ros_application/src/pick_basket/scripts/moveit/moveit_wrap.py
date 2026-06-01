#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import json
import moveit_commander

"""
moveit配置文件解析类, 内容参考 config/moveit_config.json
"""
class MoveitConfig:
    def __init__(self, filename):
        with open(filename, 'r') as file:
            self.__dict__.update(json.load(file))

class MoveitInitError(Exception):
    def __init__(self, message):
        super().__init__(message)

class MoveitWrapBase(object):
    def __init__(self, config_path=None):
        if config_path is None:
            raise MoveitInitError("moveit初始化失败: 缺少配置文件路径")
            
        """ moveit配置 """
        MoveitWrapBase._config = MoveitConfig(config_path)
        
        # 初始化机械臂、场景控制器、轨迹规划器
        try:
            MoveitWrapBase._robot = moveit_commander.RobotCommander()
            MoveitWrapBase._move_group = moveit_commander.MoveGroupCommander(MoveitWrapBase._config.move_group_name)
        except Exception as e:
                raise MoveitInitError("moveit初始化失败")

        # 设置轨迹规划器参数
        MoveitWrapBase._move_group.set_planner_id(MoveitWrapBase._config.planner_id)     # 规划器 RRTstar...
        MoveitWrapBase._move_group.set_planning_time(MoveitWrapBase._config.planning_time)
        MoveitWrapBase._move_group.set_pose_reference_frame(MoveitWrapBase._config.planning_frame)
        MoveitWrapBase._move_group.set_num_planning_attempts(MoveitWrapBase._config.num_planning)
        
        MoveitWrapBase._move_group.set_max_acceleration_scaling_factor(MoveitWrapBase._config.max_acc_scaling_factor) # 最大加速度
        MoveitWrapBase._move_group.set_max_velocity_scaling_factor(MoveitWrapBase._config.max_vel_scaling_factor)     # 最大速度
        
        MoveitWrapBase._move_group.set_goal_joint_tolerance(MoveitWrapBase._config.joint_tolerance)
        MoveitWrapBase._move_group.set_goal_position_tolerance(MoveitWrapBase._config.position_tolerance)
        MoveitWrapBase._move_group.set_goal_orientation_tolerance(MoveitWrapBase._config.orientation_tolerance)
        
        # 获取常用配置参数
        MoveitWrapBase._base_frame = MoveitWrapBase._robot.get_planning_frame()
        MoveitWrapBase._eef_link = MoveitWrapBase._move_group.get_end_effector_link()

    @property
    def config(self):
        return self._config
    @property
    def move_group(self):
        return self._move_group
    
    @property
    def robot(self):
        return self._robot
    
    @property
    def base_frame(self):
        return self._base_frame

    @property
    def eef_link(self):
        return self._eef_link
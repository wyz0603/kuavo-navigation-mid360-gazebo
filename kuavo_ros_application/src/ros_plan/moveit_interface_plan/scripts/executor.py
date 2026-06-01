#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import moveit_msgs.msg

from base import Base

class Executor(Base):
    """ 执行器 """
    
    _instance = None
    def __new__(cls, *args, **kwargs):
        """ 单实例模式 """
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self) -> None:
        pass

    def execute_traj(
        self,
        traj: moveit_msgs.msg.RobotTrajectory,
        wait: bool = True
    ) -> None:
        """仿真执行轨迹
        
        :param traj: 待执行轨迹
        :param wait: 是否等待仿真执行
        """
        self._move_group.execute(traj, wait=wait)

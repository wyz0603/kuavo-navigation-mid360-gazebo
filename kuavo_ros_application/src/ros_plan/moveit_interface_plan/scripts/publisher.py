#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import threading
import sensor_msgs.msg
import moveit_msgs.msg

from base import Base
# from exception import PublishTnreadError, GetEmptyTraj
from utils import rad_to_angle


class Publisher(Base):
    """ 发布器 """
    
    _instance = None
    def __new__(cls, *args, **kwargs):
        """ 单实例模式 """
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance


    def __init__(self) -> None:
        self._auto_publish = False
        
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        self._traj_joints_publisher = rospy.Publisher(
            "/kuavo_arm_traj",
            sensor_msgs.msg.JointState,
            queue_size=20,
        )
        
    
    @classmethod
    def set_publish_rate(cls, publish_rate):
        """ 设置发布速率 """
        cls._PUBLISH_RATE = publish_rate
    
    
    def publish_traj_joints(
        self,
        traj: moveit_msgs.msg.RobotTrajectory,
        rad: bool = False
    ) -> None:
        """发布关节轨迹
        
        消息类型为JointState
        
        :param traj: 待发布轨迹
        :param rad: 以弧度或角度发布
        """
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header.frame_id = self._planning_frame
        joint_state.name = self._joint_name
        
        rate = rospy.Rate(self._PUBLISH_RATE)
        for point in traj.joint_trajectory.points:
            if rospy.is_shutdown():
                rospy.logerr("用户终止程序")
                exit(0)
            
            joint_state.header.stamp = rospy.Time.now()
            
            arm_positions=point.positions
            # 将 position 和 velocity 扩展到 14 维
            if rad:
                arm_positions = point.positions
            else:
                arm_positions = rad_to_angle(point.positions)
            
            # right arm gourp
            if self._move_group_name == "r_arm_group":
                joint_state.position = [0.0] * (14 - len(point.positions)) + arm_positions
            else:
                joint_state.position = arm_positions + [0.0] * (14 - len(point.positions))
            
            joint_state.velocity = [0.0] * (14 - len(point.velocities)) + list(point.velocities)
            self._traj_joints_publisher.publish(joint_state)
            
            rate.sleep()

    
    def start_auto_publish(self) -> None:
        """ 开启自动发布轨迹 """
        if hasattr(self, "_publish_thread"):
            rospy.logwarn("当前已存在一个发布线程")
            return
        
        try:
            self._publish_thread = threading.Thread(target=self._traj_consumer)
        except Exception as e:
            rospy.logwarn(e)
            return
        
        self._auto_publish = True
        self._publish_thread.start()
    
    
    def stop_auto_publish(self) -> None:
        """ 结束自动发布轨迹 """
        self._auto_publish = False
        self._publish_thread.join()
        rospy.loginfo("结束发布线程")
        
        del self._publish_thread
    
    
    def _traj_consumer(self) -> None:
        """ 轨迹消费者 """
        while True:
            if rospy.is_shutdown():
                rospy.loginfo("用户终止程序，结束发布线程")
                break
            
            if not self._auto_publish:
                break
            try:
                traj = self._traj_queue.get(timeout=1)
            except Exception as e:
                continue
            self.publish_traj_joints(traj)

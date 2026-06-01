#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoJointCommand, KuavoTwist)
from kuavo_humanoid_sdk.common.logger import SDKLogger
import roslibpy
from kuavo_humanoid_sdk.common.websocket_kuavo_sdk import WebSocketKuavoSDK

class KuavoRobotObservationCoreWebsocket:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            # 初始化 WebSocket 连接
            self.websocket = WebSocketKuavoSDK()
            
            # 初始化订阅者
            self._joint_cmd_subscriber = roslibpy.Topic(
                self.websocket.client, '/joint_cmd', 'kuavo_msgs/jointCmd')
            self._cmd_vel_subscriber = roslibpy.Topic(
                self.websocket.client, '/cmd_vel', 'geometry_msgs/Twist')
            self._cmd_pose_subscriber = roslibpy.Topic(
                self.websocket.client, '/cmd_pose', 'geometry_msgs/Twist')
            
            # 订阅话题
            self._joint_cmd_subscriber.subscribe(self._joint_cmd_callback)
            self._cmd_vel_subscriber.subscribe(self._cmd_vel_callback)
            self._cmd_pose_subscriber.subscribe(self._cmd_pose_callback)
            
            """ 数据初始化 """
            self._joint_cmd = KuavoJointCommand(
                joint_q = [0.0] * 28,
                joint_v = [0.0] * 28,
                tau = [0.0] * 28,
                tau_max = [0.0] * 28,
                tau_ratio = [0.0] * 28,
                joint_kp = [0.0] * 28,
                joint_kd = [0.0] * 28,
                control_modes = [0] * 28
            )

            self._cmd_vel = KuavoTwist(
                linear = (0.0, 0.0, 0.0),
                angular = (0.0, 0.0, 0.0)
            )

            self._cmd_pose = KuavoTwist(
                linear = (0.0, 0.0, 0.0),
                angular = (0.0, 0.0, 0.0)
            )

            self._initialized = True
            
    def _joint_cmd_callback(self, msg):
        """关节命令回调函数
        
        Args:
            msg: roslibpy 消息，包含关节命令数据
        """
        # 将 roslibpy 消息转换为我们的格式
        self._joint_cmd.joint_q = list(msg['joint_q'])
        self._joint_cmd.joint_v = list(msg['joint_v'])
        self._joint_cmd.tau = list(msg['tau'])
        self._joint_cmd.tau_max = list(msg['tau_max'])
        self._joint_cmd.tau_ratio = list(msg['tau_ratio'])
        self._joint_cmd.joint_kp = list(msg['joint_kp'])
        self._joint_cmd.joint_kd = list(msg['joint_kd'])
        self._joint_cmd.control_modes = list(msg['control_modes'])
        
    def _cmd_vel_callback(self, msg):
        """速度命令回调函数
        
        Args:
            msg: WebSocket 消息，包含速度命令数据
        """
        self._cmd_vel.linear = (msg['linear']['x'], msg['linear']['y'], msg['linear']['z'])
        self._cmd_vel.angular = (msg['angular']['x'], msg['angular']['y'], msg['angular']['z'])
        
    def _cmd_pose_callback(self, msg):
        """位姿命令回调函数
        
        Args:
            msg: WebSocket 消息，包含位姿命令数据
        """
        self._cmd_pose.linear = (msg['linear']['x'], msg['linear']['y'], msg['linear']['z'])
        self._cmd_pose.angular = (msg['angular']['x'], msg['angular']['y'], msg['angular']['z'])
        
        
    @property
    def joint_command(self) -> KuavoJointCommand:
        return self._joint_cmd
        
    @property
    def cmd_vel(self) -> KuavoTwist:
        return self._cmd_vel
        
    @property
    def cmd_pose(self) -> KuavoTwist:
        return self._cmd_pose
    
    @property    
    def arm_position_command(self) -> list:
        """Return the position commands for the arm joints (indices 12-25).
        
        Returns:
            list: Position commands for arm joints
        """
        return self._joint_cmd.joint_q[12:26]
    
    @property
    def head_position_command(self) -> list:
        """Return the position commands for the head joints (indices 26-27).
        
        Returns:
            list: Position commands for head joints
        """
        return self._joint_cmd.joint_q[-2:]


# TODO: 实现 Websocket 接口在这里
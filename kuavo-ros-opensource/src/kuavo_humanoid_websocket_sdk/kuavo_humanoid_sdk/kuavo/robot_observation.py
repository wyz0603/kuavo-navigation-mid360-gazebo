#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.interfaces.data_types import (
    KuavoJointCommand, KuavoTwist)
from kuavo_humanoid_sdk.kuavo.core.ros.observation import KuavoRobotObservationCoreWebsocket

class KuavoRobotObservation:
    """Class for accessing robot observation data.
    
    This class provides a high-level interface to access robot observation data,
    including joint commands, velocity commands, and pose commands.
    
    Attributes:
        _obs_core: The core observation object that handles ROS communication.
    """
    
    def __init__(self, robot_type: str = "kuavo"):
        """Initialize the robot observation interface.
        
        Args:
            robot_type (str): Type of the robot. Defaults to "kuavo".
        """
        self._obs_core = KuavoRobotObservationCoreWebsocket()

    @property
    def joint_command(self) -> KuavoJointCommand:
        """Get the current joint command.
        
        Returns:
            KuavoJointCommand: Object containing position, velocity, and torque commands
                for all robot joints.
        """
        return self._obs_core.joint_command

    @property
    def cmd_vel(self) -> KuavoTwist:
        """Get the current velocity command.
        
        Returns:
            KuavoTwist: Object containing linear velocity (m/s) and angular velocity (rad/s) commands.
        """
        return self._obs_core.cmd_vel

    @property
    def cmd_pose(self) -> KuavoTwist:
        """Get the current pose command.
        
        Returns:
            KuavoTwist: Object containing linear pose commands (m) and angular pose commands (rad).
        """
        return self._obs_core.cmd_pose

    @property    
    def arm_position_command(self) -> list:
        """Get the position commands for the arm joints.
        
        Returns:
            list: Position commands for arm joints (indices 12-25) in radians.
        """
        return self._obs_core.arm_position_command

    @property
    def head_position_command(self) -> list:
        """Get the position commands for the head joints.
        
        Returns:
            list: Position commands for head joints (indices 26-27) in radians.
        """
        return self._obs_core.head_position_command
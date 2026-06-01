from abc import ABC, abstractmethod
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk import KuavoRobotState
# from kuavo_humanoid_sdk import KuavoRobotTools
# from kuavo_humanoid_sdk import KuavoRobotVision

class KuavoRobotStrategyBase(ABC):
    """Kuavo机器人策略基础类，提供策略执行的抽象接口"""
    
    def __init__(self, robot:KuavoRobot, robot_state:KuavoRobotState, robot_tools:KuavoRobotTools, robot_vision:KuavoRobotVision):
        """初始化策略基础类
        
        Args:
            robot: KuavoRobot实例，提供机器人控制能力
            robot_state: KuavoRobotState实例，提供机器人状态信息
            robot_tools: KuavoRobotTools实例，提供坐标转换等工具
            robot_vision: KuavoRobotVision实例，提供视觉感知能力
        """
        self.robot = robot
        self.state = robot_state 
        self.tools = robot_tools 
        self.vision = robot_vision 
    
    @abstractmethod
    def head_find_target(self, target_info, **kwargs):
        """寻找特定ID的目标
        
        Args:
            target_id: 目标的ID标识
            **kwargs: 其他参数
            
        Returns:
            bool: 是否成功找到目标
        """
        pass
    
    @abstractmethod
    def walk_approach_target(self, target_info, target_distance=0.5, **kwargs):
        """走/接近特定的目标到指定距离
        
        Args:
            target_id: 目标的ID标识
            target_distance: 与目标的期望距离(米)
            **kwargs: 其他参数
            
        Returns:
            bool: 是否成功接近目标
        """
        pass
    
    @abstractmethod
    def arm_move_to_target(self, target_pose, **kwargs):
        """手臂移动到特定的位置(闭环)
        
        Args:
            target_pose: 目标位置姿态
            **kwargs: 其他参数
            
        Returns:
            bool: 是否成功移动到目标位置
        """
        pass


import pytest
import time
from unittest.mock import MagicMock, patch
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox
from kuavo_humanoid_sdk.interfaces.data_types import AprilTagData, PoseQuaternion

class TestKuavoGraspBox:
    
    @pytest.fixture
    def setup_grasp_box(self):
        # 创建模拟对象
        mock_robot = MagicMock()
        mock_robot_state = MagicMock()
        mock_robot_tools = MagicMock()
        mock_robot_vision = MagicMock()
        
        # 创建策略实例
        grasp_box = KuavoGraspBox(
            robot=mock_robot,
            robot_state=mock_robot_state,
            robot_tools=mock_robot_tools,
            robot_vision=mock_robot_vision
        )
        
        return grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision
    
    def test_walk_approach_target_success(self, setup_grasp_box):
        """测试成功接近目标"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的AprilTag数据
        test_tag = AprilTagData(
            id=[123],
            size=[0.1],
            pose=[PoseQuaternion(
                position=(1.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            )]
        )
        
        # 模拟vision返回的数据
        mock_robot_vision.get_data_by_id_from_odom.return_value = {
            "poses": [{
                "position": (1.0, 0.0, 0.0),
                "orientation": (0.0, 0.0, 0.0, 1.0)
            }]
        }
        
        # 模拟_approach_target方法
        grasp_box._approach_target = MagicMock(return_value=True)
        
        # 调用方法
        result = grasp_box.walk_approach_target(
            target_info=test_tag,
            target_distance=0.5,
            approach_speed=0.15
        )
        
        # 验证结果
        assert result is True
        grasp_box._approach_target.assert_called_once()
    
    def test_walk_approach_target_no_data(self, setup_grasp_box):
        """测试目标数据不存在的情况"""
        grasp_box, mock_robot, mock_robot_state, mock_robot_tools, mock_robot_vision = setup_grasp_box
        
        # 创建测试用的AprilTag数据
        test_tag = AprilTagData(
            id=[456],
            size=[0.2],
            pose=[PoseQuaternion(
                position=(0.0, 1.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            )]
        )
        
        # 模拟vision返回None
        mock_robot_vision.get_data_by_id_from_odom.return_value = None
        
        # 将 _approach_target 替换为 MagicMock 对象
        grasp_box._approach_target = MagicMock()
        
        # 调用方法
        result = grasp_box.walk_approach_target(
            target_info=test_tag,
            target_distance=0.8,
            approach_speed=0.1
        )
        
        # 验证结果
        assert result is False
        
        # 验证 _approach_target 没有被调用
        grasp_box._approach_target.assert_not_called()
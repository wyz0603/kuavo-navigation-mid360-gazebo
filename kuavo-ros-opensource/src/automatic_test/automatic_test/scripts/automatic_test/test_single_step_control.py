#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytest
import rospy
import numpy as np
from kuavo_msgs.msg import footPose, footPoseTargetTrajectories
from kuavo_msgs.srv import singleStepControl, singleStepControlRequest, singleStepControlResponse, getCurrentGaitName
from .conftest import AutoTrackingStats
import tf

current_gait = ""

class StepSizeConfig:
    """Step size config class"""
    def __init__(self, namespace):
        self.namespace = namespace
        self._load_config()

    def _load_config(self):
        """Load config from ROS param server"""
        self.config = {
            'forward': {
                'param': 'forward_step_size',
                'default': 0.08,
                'transform': lambda x: [x, 0, 0, 0]
            },
            'backward': {
                'param': 'backward_step_size',
                'default': -0.08,
                'transform': lambda x: [x, 0, 0, 0]
            },
            'left_move': {
                'param': 'left_move_step_size',
                'default': 0.05,
                'transform': lambda x: [0, x, 0, 0]
            },
            'right_move': {
                'param': 'right_move_step_size',
                'default': -0.05,
                'transform': lambda x: [0, x, 0, 0]
            },
            'rotate_left': {
                'param': 'rotate_left_step_size',
                'default': 45,
                'transform': lambda x: [0, 0, 0, np.radians(x)]
            },
            'rotate_right': {
                'param': 'rotate_right_step_size',
                'default': -45,
                'transform': lambda x: [0, 0, 0, np.radians(x)]
            }
        }

    def get_step_config(self):
        steps = {}
        for step_name, config in self.config.items():
            param_name = f'/{self.namespace}/{config["param"]}'
            value = rospy.get_param(param_name, config['default'])
            steps[step_name] = config['transform'](value)
            rospy.loginfo(f"Step config loaded: {step_name}: {steps[step_name]}")
        return steps

class StepConfig:
    STEPS = {}

    DT = 1.2

@pytest.mark.usefixtures("ros_setup")
class TestSingleStepControl:
    def setup_class(self):
        self.ros_namespace = 'test_single_step_control'
        
        self.current_gait_client = rospy.ServiceProxy(
            'humanoid_get_current_gait_name', 
            getCurrentGaitName
        )
        self.single_step_control_srv = rospy.ServiceProxy(
            '/humanoid_single_step_control', 
            singleStepControl
        )
        
        self.world_frame = rospy.get_param(f'/{self.ros_namespace}/world_frame', 'odom')
        self.robot_frame = rospy.get_param(f'/{self.ros_namespace}/robot_frame', 'base_link')
        
        step_config = StepSizeConfig(self.ros_namespace)
        StepConfig.STEPS = step_config.get_step_config()
        
        rospy.loginfo("Step config loaded:")
        for step_name, step_params in StepConfig.STEPS.items():
            rospy.loginfo(f"- {step_name}: {step_params}")
        
        rospy.sleep(1)
    
    def get_robot_pose(self):
        try:
            listener = tf.TransformListener()
            listener.waitForTransform(self.world_frame, self.robot_frame, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"get robot pose failed: {e}")
            return None, None

    def wait_for_completion(self, check_robot_alive, auto_tracking_stats, description=""):
        """wait for action completion and check status"""
        while current_gait != "stance":
            self.update_current_gait()
            rospy.sleep(0.1)
            assert check_robot_alive.get_status(), \
                   f"机器人死亡: {check_robot_alive.get_error()}"
            assert auto_tracking_stats.status == AutoTrackingStats.FOLLOWING, \
                   f"动捕天轨系统自动跟踪机器人失败 ({description})"
      
    def update_current_gait(self):
        result = self.current_gait_client()
        if result.success:
            global current_gait
            current_gait = result.gait_name
        else:
            current_gait = ""
            rospy.logerr(f"get current gait failed: {result.message}")
    

    def calculate_target_pose(self, current_trans, current_rot, step_config):
        """根据当前位姿和步态配置计算目标位姿"""
        current_euler = tf.transformations.euler_from_quaternion(current_rot)
        current_yaw = current_euler[2]
        
        # 创建变换矩阵
        current_matrix = tf.transformations.compose_matrix(
            translate=current_trans,
            angles=[0, 0, current_yaw]
        )
        
        # 计算步态增量的变换矩阵
        step_matrix = tf.transformations.compose_matrix(
            translate=[step_config[0], step_config[1], step_config[2]],
            angles=[0, 0, step_config[3]]
        )
        
        # 计算目标位姿
        target_matrix = np.dot(current_matrix, step_matrix)
        target_trans = tf.transformations.translation_from_matrix(target_matrix)
        target_angles = tf.transformations.euler_from_matrix(target_matrix)
        
        return target_trans, target_angles[2]  # 返回目标位置和偏航角

    def eval_pose_error(self, current_pose, target_pose):
        """评估位姿误差"""
        position_error = np.linalg.norm(np.array(current_pose[0]) - np.array(target_pose[0]))
        
        # 计算偏航角误差（考虑角度的周期性）
        yaw_error = abs(target_pose[1] - current_pose[1])
        yaw_error = min(yaw_error, 2*np.pi - yaw_error)  # 确保误差在 [0, pi] 范围内
        
        return {
            'position_error': position_error,
            'yaw_error': np.degrees(yaw_error)  # 转换为角度
        }

    def execute_single_step(self, torso_pose, check_robot_alive, auto_tracking_stats, description=""):
        """执行单步并评估结果"""
        # 获取初始位姿
        initial_trans, initial_rot = self.get_robot_pose()
        if initial_trans is None:
            raise Exception("无法获取机器人初始位姿")
        
        # 计算目标位姿
        target_trans, target_yaw = self.calculate_target_pose(
            initial_trans, 
            initial_rot, 
            torso_pose
        )
        rospy.loginfo(f"目标位置: {target_trans}, 目标偏航角: {np.degrees(target_yaw)}度")
        
        # 执行步态
        req = singleStepControlRequest()
        foot_pose_target_trajectories = footPoseTargetTrajectories()
        foot_pose_target_trajectories.timeTrajectory = [StepConfig.DT]
        
        foot_pose_msg = footPose()
        foot_pose_msg.torsoPose = torso_pose
        foot_pose_target_trajectories.footPoseTrajectory = [foot_pose_msg]
        req.foot_pose_target_trajectories = foot_pose_target_trajectories
        
        # 调用服务
        res = self.single_step_control_srv(req)
        assert res.success, f"单步控制失败: {res.message}"
        
        # 等待执行完成
        rospy.sleep(0.5)
        self.update_current_gait()
        self.wait_for_completion(check_robot_alive, auto_tracking_stats, description)
        
        # 获取执行后的位姿
        final_trans, final_rot = self.get_robot_pose()
        if final_trans is None:
            raise Exception("无法获取机器人最终位姿")
        
        final_euler = tf.transformations.euler_from_quaternion(final_rot)
        final_yaw = final_euler[2]
        
        # 评估结果
        errors = self.eval_pose_error(
            (final_trans, final_yaw),
            (target_trans, target_yaw)
        )
        
        # 输出评估结果
        rospy.loginfo(f"\n执行结果评估 ({description}):")
        rospy.loginfo(f"- 位置误差: {errors['position_error']:.4f} 米")
        rospy.loginfo(f"- 偏航角误差: {errors['yaw_error']:.2f} 度")
        
        # 检查误差是否在可接受范围内
        position_threshold = 0.05  # 5cm
        yaw_threshold = 5.0  # 5度
        
        rospy.loginfo(f"完成步态: {description}")
        return errors

    @pytest.mark.single_step_control
    def test_single_step_control(self, check_robot_ready, check_robot_alive, auto_tracking_stats, test_timer):
        """Test single step control"""
        test_sequence = [
            ('forward', 'forward'),
            ('backward', 'backward'),
            ('left_move', 'left_move'),
            ('right_move', 'right_move'),
            ('rotate_left', 'rotate_left'),
            ('rotate_right', 'rotate_right')
        ]
        
        test_results = {}
        for step_name, description in test_sequence:
            rospy.loginfo(f"\n=== 开始测试: {description} ===")
            round = rospy.get_param(f'/{self.ros_namespace}/round', 10)
            step_errors = []
            
            for i in range(round):
                rospy.loginfo(f"\n第 {i+1}/{round} 轮测试")
                errors = self.execute_single_step(
                    StepConfig.STEPS[step_name],
                    check_robot_alive,
                    auto_tracking_stats,
                    description
                )
                step_errors.append(errors)
            
            avg_position_error = np.mean([e['position_error'] for e in step_errors])
            avg_yaw_error = np.mean([e['yaw_error'] for e in step_errors])
            
            test_results[step_name] = {
                'avg_position_error': avg_position_error,
                'avg_yaw_error': avg_yaw_error
            }
            
            rospy.loginfo(f"- {description} 测试结果:")
            rospy.loginfo(f"- 平均位置误差: {avg_position_error:.4f} 米")
            rospy.loginfo(f"- 平均偏航角误差: {avg_yaw_error:.2f} 度")
    
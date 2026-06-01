#!/usr/bin/env python3

import rospy
import copy
import numpy as np
import json
import os
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from ocs2_msgs.msg import footPose
from kuavo_msgs.msg import footPoseTargetTrajectories, footPoses
from kuavo_msgs.srv import singleStepControl, singleStepControlRequest
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, TransformStamped
from stair_alignment.msg import StairAlignmentStatus
from nav_msgs.msg import Odometry

def quaternion_multiply(q1, q2):
    """ 四元数乘法 """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def rotate_around_y_axis(q, angle):
    """ 绕y轴旋转angle角度 """
    sin_half_angle = np.sin(angle / 2)
    cos_half_angle = np.cos(angle / 2)
    q_y = np.array([cos_half_angle, 0, sin_half_angle, 0])
    return quaternion_multiply(q_y, q)

class StairCloseToTagNode:
    def __init__(self, tag_id, expected_offset=None, threshold=None) -> None:
        self._tag_id = tag_id
        self._tag_pose = None
        self._last_detection_time = None  # 记录最后一次检测到tag的时间
        self._sub_apriltag = rospy.Subscriber("/robot_tag_info", AprilTagDetectionArray, self._detection_callback)
        self._pub_foot_pose_traj = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)
        
        # 添加TF相关组件用于获取机器人位置
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._current_robot_pose = None
        
        # 订阅里程计信息（如果有的话）
        self._sub_odom = rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)
        
        # 添加跳变检测相关变量
        self._last_tag_pose = None  # 存储上一次的tag位置
        self._last_target_pose = None  # 存储上一次计算的目标位置 [x, y, yaw]
        
        # Load configuration from config file
        self._config = self._load_config()
        self._max_step_sizes = self._config.get("max_step_sizes", {})
        self._timing_params = self._config.get("timing_params", {})
        self._stand_params = self._config.get("stand_params", {})
        
        # Use expected_offset from config if not provided
        if expected_offset is None:
            self._expected_offset = self._stand_params.get("expected_offset", [-0.4, 0, 0])
        else:
            self._expected_offset = expected_offset
            
        # Use threshold from config if not provided
        if threshold is None:
            x_threshold = self._stand_params.get("x_threshold", 0.02)
            y_threshold = self._stand_params.get("y_threshold", 0.02)
            yaw_deg_threshold = self._stand_params.get("yaw_deg_threshold", 10)
            self._threshold = [x_threshold, y_threshold, np.radians(yaw_deg_threshold)]
        else:
            self._threshold = threshold
            
        # 跳变检测阈值
        self._jump_detection_threshold = [
            self._stand_params.get("jump_x_threshold", 0.2),  # x方向跳变阈值 (米)
            self._stand_params.get("jump_y_threshold", 0.2),  # y方向跳变阈值 (米)
            np.radians(self._stand_params.get("jump_yaw_deg_threshold", 30))  # yaw跳变阈值 (弧度)
        ]
        
        rospy.loginfo(f"Loaded config: max_step_sizes={self._max_step_sizes}, timing_params={self._timing_params}")
        rospy.loginfo(f"Using stand_params: expected_offset={self._expected_offset}, threshold=[{self._threshold[0]}, {self._threshold[1]}, {np.degrees(self._threshold[2]):.1f}°]")

    def _load_config(self):
        """Load configuration from JSON file"""
        try:
            # Get the directory of this script
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(script_dir, '..', 'config', 'config.json')
            
            with open(config_path, 'r') as f:
                config = json.load(f)
            rospy.loginfo(f"Successfully loaded config from {config_path}")
            return config
        except Exception as e:
            rospy.logerr(f"Failed to load config from {config_path}: {e}")
            # Return default config if loading fails
            return {
                "max_step_sizes": {
                    "max_x_step": 0.04,
                    "max_y_step": 0.03,
                    "max_yaw_step_deg": 25
                },
                "timing_params": {
                    "step_duration": 1.2,
                    "wait_buffer": 2.0,
                    "detection_timeout": 1.0
                },
                "stand_params": {
                    "expected_offset": [-0.4, 0, 0],
                    "x_threshold": 0.02,
                    "y_threshold": 0.02,
                    "yaw_deg_threshold": 10,
                    "jump_x_threshold": 0.2,
                    "jump_y_threshold": 0.2,
                    "jump_yaw_deg_threshold": 30
                }
            }

    def _detection_callback(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return

        for detection in msg.detections:
            if detection.id[0] == self._tag_id:
                # extract the quaternion
                current_quaternion = np.array([
                    detection.pose.pose.pose.orientation.w,
                    detection.pose.pose.pose.orientation.x,
                    detection.pose.pose.pose.orientation.y,
                    detection.pose.pose.pose.orientation.z
                ])
                
                # WARN! WARN! WARN!: apriltag 垂直于楼梯表面张贴
                # 绕y轴旋转-90°
                # rotated_quaternion = rotate_around_y_axis(current_quaternion, np.pi / 2)
                
                # update the pose
                self._tag_pose = Pose()
                # 不需要旋转!
                # self._tag_pose.orientation = Quaternion(
                #     w=rotated_quaternion[0],
                #     x=rotated_quaternion[1],
                #     y=rotated_quaternion[2],
                #     z=rotated_quaternion[3]
                # )
                self._tag_pose.orientation =  copy.deepcopy(detection.pose.pose.pose.orientation)
                
                # save the position
                self._tag_pose.position =  copy.deepcopy(detection.pose.pose.pose.position)
                
                # 更新最后一次检测时间
                self._last_detection_time = rospy.Time.now()
                break

    def _odom_callback(self, msg: Odometry):
        """里程计回调函数，更新机器人当前位置"""
        self._current_robot_pose = msg.pose.pose

    def get_robot_pose_from_tf(self, target_frame="odom", source_frame="base_link"):
        """
        通过TF获取机器人在指定坐标系下的位置
        
        Args:
            target_frame: 目标坐标系，默认为"odom"
            source_frame: 源坐标系，默认为"base_link"
            
        Returns:
            geometry_msgs.Pose: 机器人位置，如果获取失败返回None
        """
        try:
            # 获取最新的变换
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            
            # 创建PoseStamped消息
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = source_frame
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = 0.0
            pose_stamped.pose.position.y = 0.0
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            # 变换到目标坐标系
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            
            return transformed_pose.pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get robot pose from TF: {e}")
            return None

    def get_current_robot_pose(self):
        """
        获取机器人当前位置，优先使用里程计，备选使用TF
        
        Returns:
            geometry_msgs.Pose: 机器人当前位置
        """
        # 优先使用里程计数据
        if self._current_robot_pose is not None:
            return self._current_robot_pose
        
        # 备选：通过TF获取
        tf_pose = self.get_robot_pose_from_tf()
        if tf_pose is not None:
            return tf_pose
        
        # 如果都获取不到，返回原点
        rospy.logwarn("Cannot get robot pose, using origin as default")
        default_pose = Pose()
        default_pose.orientation.w = 1.0
        return default_pose

    def call_single_step_control(self, time_traj, torso_traj):
        single_step_control_srv = rospy.ServiceProxy('/humanoid_single_step_control', singleStepControl)
        req = singleStepControlRequest()
        foot_pose_target_trajectories = footPoseTargetTrajectories()
        foot_pose_target_trajectories.timeTrajectory = time_traj
        foot_pose_target_trajectories.footIndexTrajectory = [0] * len(time_traj)  # Add foot index trajectory
        footPoseTrajectory = []
        for i in range(len(time_traj)):
            foot_pose_msg = footPose()
            foot_pose_msg.torsoPose = torso_traj[i]
            footPoseTrajectory.append(foot_pose_msg)
        foot_pose_target_trajectories.footPoseTrajectory = footPoseTrajectory
        req.foot_pose_target_trajectories = foot_pose_target_trajectories
        res = single_step_control_srv(req)
        rospy.sleep(0.5) # wait the trajectory to be executed
        if not res.success:
            print(res.message)
        return res.success
    
    def _check_threshold(self, x, y, yaw):
        # return abs(x) < self._threshold[0] and abs(y) < self._threshold[1] and abs(yaw) < self._threshold[2]
        print(f"x: {x}, y: {y}, yaw: {yaw}, threshold: {self._threshold}")
        print(f"abs(x) < self._threshold[0]: {abs(x) < self._threshold[0]}")
        print(f"abs(y) < self._threshold[1]: {abs(y) < self._threshold[1]}")
        print(f"abs(yaw) < self._threshold[2]: {abs(yaw) < self._threshold[2]}")
        return abs(x) < self._threshold[0] and abs(y) < self._threshold[1] and abs(yaw) < self._threshold[2]
    
    def _check_jump_detection(self, current_x, current_y, current_yaw):
        """
        检测当前识别结果是否存在跳变
        
        Args:
            current_x: 当前计算的x位置
            current_y: 当前计算的y位置  
            current_yaw: 当前计算的yaw角度
            
        Returns:
            bool: True表示检测到跳变，应该跳过这次识别结果；False表示正常
        """
        if self._last_target_pose is None:
            # 第一次识别，没有历史数据，不判断跳变
            return False
            
        last_x, last_y, last_yaw = self._last_target_pose
        
        # 计算与上一次的差值
        delta_x = abs(current_x - last_x)
        delta_y = abs(current_y - last_y)
        delta_yaw = abs(current_yaw - last_yaw)
        
        # 处理yaw角度的周期性 (-π, π]
        if delta_yaw > np.pi:
            delta_yaw = 2 * np.pi - delta_yaw
            
        # 检查是否超过跳变阈值
        jump_detected = (
            delta_x > self._jump_detection_threshold[0] or
            delta_y > self._jump_detection_threshold[1] or  
            delta_yaw > self._jump_detection_threshold[2]
        )
        
        if jump_detected:
            rospy.logwarn(f"Jump detected! Delta: x={delta_x:.3f}, y={delta_y:.3f}, yaw={np.degrees(delta_yaw):.1f}°")
            rospy.logwarn(f"Jump thresholds: x={self._jump_detection_threshold[0]:.3f}, y={self._jump_detection_threshold[1]:.3f}, yaw={np.degrees(self._jump_detection_threshold[2]):.1f}°")
            rospy.logwarn("Skipping this detection result...")
            
        return jump_detected

    def close_to_tag(self):
        # Get maximum step sizes from config
        max_x_step = self._max_step_sizes.get("max_x_step", 0.04)
        max_y_step = self._max_step_sizes.get("max_y_step", 0.03)
        max_yaw_step_deg = self._max_step_sizes.get("max_yaw_step_deg", 25)
        max_yaw_step = np.radians(max_yaw_step_deg)
        
        # Get timing parameters from config
        step_duration = self._timing_params.get("step_duration", 1.2)
        wait_buffer = self._timing_params.get("wait_buffer", 2.0)
        detection_timeout = self._timing_params.get("detection_timeout", 1.0)
        
        rospy.loginfo(f"Using config: max_x_step={max_x_step}, max_y_step={max_y_step}, max_yaw_step={max_yaw_step_deg}°")
        
        while not rospy.is_shutdown():
            if self._tag_pose is None:
                rospy.logwarn("No tag detected. Waiting...")
                rospy.sleep(detection_timeout)
                continue

            # Calculate target pose
            x = round(self._tag_pose.position.x + self._expected_offset[0], 3)
            y = round(self._tag_pose.position.y + self._expected_offset[1], 3)
            orientation_q = self._tag_pose.orientation
            _, _, tag_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            # 目标朝向 = AprilTag朝向 + 用户指定的偏移量
            yaw = round(tag_yaw + self._expected_offset[2], 3)

            rospy.loginfo(f"Target pose: x: {x}, y: {y}, yaw: {yaw}")

            if self._check_threshold(x, y, yaw):
                rospy.loginfo("Reached target pose within threshold.")
                break

            # Calculate the number of steps needed
            num_steps = max(
                abs(int(np.ceil(x / max_x_step))),
                abs(int(np.ceil(y / max_y_step))),
                abs(int(np.ceil(yaw / max_yaw_step)))
            )

            # Plan the trajectory steps
            steps = []
            for i in range(num_steps):
                next_x = min(max_x_step * (i + 1), abs(x)) * np.sign(x)
                next_y = min(max_y_step * (i + 1), abs(y)) * np.sign(y)
                next_yaw = min(max_yaw_step * (i + 1), abs(yaw)) * np.sign(yaw)
                
                steps.append([round(next_x, 3), round(next_y, 3), 0, round(next_yaw, 3)])

            # Ensure the last step reaches the exact target
            if steps[-1] != [x, y, 0, yaw]:
                steps.append([x, y, 0, yaw])

            # Prepare time trajectory for a single call
            time_traj = [i * step_duration for i in range(1, len(steps) + 1)]
            torso_traj = steps

            rospy.loginfo(f"Planned trajectory: {torso_traj}")

            # Call single_step_control once with all steps
            success = self.call_single_step_control(time_traj, torso_traj)

            if not success:
                rospy.logerr("Failed to execute trajectory. Retrying...")
                rospy.sleep(1)
                continue

            # Wait for the entire trajectory to complete
            total_time = time_traj[-1] if time_traj else 0
            rospy.sleep(total_time + wait_buffer)

        rospy.loginfo("Finished stair_close_to_tag operation.")
    
    def close_to_tag_step_by_step(self):
        # Get maximum step sizes from config
        max_x_step = self._max_step_sizes.get("max_x_step", 0.04)
        max_y_step = self._max_step_sizes.get("max_y_step", 0.03)
        max_yaw_step_deg = self._max_step_sizes.get("max_yaw_step_deg", 25)
        max_yaw_step = np.radians(max_yaw_step_deg)
        
        # Get timing parameters from config
        step_duration = self._timing_params.get("step_duration", 1.2)
        wait_buffer = self._timing_params.get("wait_buffer", 2.0)
        detection_timeout = self._timing_params.get("detection_timeout", 1.0)
        
        rospy.loginfo(f"Using config: max_x_step={max_x_step}, max_y_step={max_y_step}, max_yaw_step={max_yaw_step_deg}°")
        
        while not rospy.is_shutdown():
            if self._tag_pose is None:
                rospy.logwarn("No tag detected. Waiting...")
                rospy.sleep(detection_timeout)
                continue

            # Calculate target pose
            x = round(self._tag_pose.position.x + self._expected_offset[0], 3)
            y = round(self._tag_pose.position.y + self._expected_offset[1], 3)
            orientation_q = self._tag_pose.orientation
            _, _, tag_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            # 目标朝向 = AprilTag朝向 + 用户指定的偏移量
            yaw = round(tag_yaw + self._expected_offset[2], 3)

            rospy.loginfo(f"Target pose: x: {x}, y: {y}, yaw: {yaw}")

            if self._check_threshold(x, y, yaw):
                rospy.loginfo("Reached target pose within threshold.")
                break

            # Calculate the next step
            next_x = min(max_x_step, abs(x)) * np.sign(x)
            next_y = min(max_y_step, abs(y)) * np.sign(y)
            next_yaw = min(max_yaw_step, abs(yaw)) * np.sign(yaw)

            step = [round(next_x, 3), round(next_y, 3), 0, round(next_yaw, 3)]

            rospy.loginfo(f"Planned step: {step}")

            # Prepare time trajectory for a single step
            time_traj = [step_duration]
            torso_traj = [step]

            # Call single_step_control for this step
            success = self.call_single_step_control(time_traj, torso_traj)

            if not success:
                rospy.logerr("Failed to execute step. Retrying...")
                rospy.sleep(1)
                continue

            # Wait for the step to complete
            rospy.sleep(step_duration + wait_buffer)

        rospy.loginfo("Finished stair_close_to_tag operation.")

    def backward_specify_distance(self, target_x):
        backward_single_step = [-0.1, 0, 0, 0]
        dt = 1.2
        step_num = int(np.ceil(abs(target_x) / abs(backward_single_step[0])))
        
        time_traj = [dt * i for i in range(1, step_num + 1)]
        torso_traj = []
        
        accumulated_x = 0
        for _ in range(step_num):
            remaining_x = target_x - accumulated_x
            if abs(remaining_x) < abs(backward_single_step[0]):
                step_x = remaining_x
            else:
                step_x = backward_single_step[0]
            
            accumulated_x += step_x
            torso_traj.append([round(accumulated_x, 3), 0, 0, 0])

        # Ensure the last step reaches exactly the target distance
        if torso_traj[-1][0] != target_x:
            torso_traj[-1][0] = target_x

        rospy.loginfo(f"Trajectory: {torso_traj}")
        success = self.call_single_step_control(time_traj, torso_traj)
        
        if success:
            rospy.loginfo(f"Successfully moved backward by {target_x} meters.")
        else:
            rospy.logerr("Failed to execute backward movement.")

    def close_to_tag_with_status(self, status_pub, tag_id, offset_x, offset_y, offset_yaw):
        """
        带状态发布的单步控制方法
        """
        # Get maximum step sizes from config
        max_x_step = self._max_step_sizes.get("max_x_step", 0.04)
        max_y_step = self._max_step_sizes.get("max_y_step", 0.03)
        max_yaw_step_deg = self._max_step_sizes.get("max_yaw_step_deg", 25)
        max_yaw_step = np.radians(max_yaw_step_deg)
        
        # Get timing parameters from config
        step_duration = self._timing_params.get("step_duration", 1.2)
        wait_buffer = self._timing_params.get("wait_buffer", 3.0)
        detection_timeout = self._timing_params.get("detection_timeout", 1.0)
        
        rospy.loginfo(f"Using config: max_x_step={max_x_step}, max_y_step={max_y_step}, max_yaw_step={max_yaw_step_deg}°")
        
        step_count = 0
        total_steps = 0
        
        while not rospy.is_shutdown():
            # 检查tag是否丢失（通过时间戳判断）
            current_time = rospy.Time.now()
            tag_lost = False
            
            if self._tag_pose is None:
                tag_lost = True
                rospy.logwarn("No tag detected. Waiting...")
            elif self._last_detection_time is not None:
                time_since_last_detection = (current_time - self._last_detection_time).to_sec()
                if time_since_last_detection > detection_timeout:
                    tag_lost = True
                    rospy.logwarn(f"Tag lost for {time_since_last_detection:.2f}s. Clearing pose and waiting...")
                    # 清空tag位置，强制重新检测
                    self._tag_pose = None
                    self._last_detection_time = None
            
            if tag_lost:
                # 发布等待状态
                status_msg = StairAlignmentStatus()
                status_msg.tag_id = tag_id
                status_msg.current_state = "detecting"
                status_msg.message = "Waiting for AprilTag detection..."
                status_pub.publish(status_msg)
                rospy.sleep(detection_timeout)
                continue

            # Calculate current pose relative to target
            current_x = round(self._tag_pose.position.x + offset_x, 3)
            print(f"_tag_pose.position: {self._tag_pose.position.x}")
            current_y = round(self._tag_pose.position.y + offset_y, 3)
            print(f"_tag_pose.position: {self._tag_pose.position.y}")
            orientation_q = self._tag_pose.orientation
            _, _, tag_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            # tag_yaw = tag_yaw - np.pi / 2
            print(f"tag_yaw: {tag_yaw}")
            if tag_yaw > 0:
                tag_yaw = tag_yaw - np.pi / 2
            else:
                tag_yaw = tag_yaw + np.pi / 2
            
            
            # 目标朝向 = AprilTag朝向 + 用户指定的偏移量
            target_yaw = tag_yaw + offset_yaw
            
            
            target_yaw = round(target_yaw, 3)
            # 当前需要调整的yaw差值 = 目标朝向 - 当前机器人朝向(假设为0)
            current_yaw = target_yaw



            rospy.loginfo(f"Current pose: x: {current_x}, y: {current_y}, yaw: {current_yaw}")

            # Check if reached target within threshold
            if self._check_threshold(current_x, current_y, current_yaw):
                rospy.loginfo("Reached target pose within threshold.")
                return True

            # Calculate the next step (single step control)
            # 如果某个方向已经满足阈值，则该方向不进行移动
            if abs(current_x) < self._threshold[0]:
                next_x = 0.0  # x方向已满足阈值，不移动
                rospy.loginfo(f"X direction within threshold ({abs(current_x):.3f} < {self._threshold[0]:.3f}), no movement")
            else:
                next_x = min(max_x_step, abs(current_x)) * np.sign(current_x)
                
            if abs(current_y) < self._threshold[1]:
                next_y = 0.0  # y方向已满足阈值，不移动
                rospy.loginfo(f"Y direction within threshold ({abs(current_y):.3f} < {self._threshold[1]:.3f}), no movement")
            else:
                next_y = min(max_y_step, abs(current_y)) * np.sign(current_y)
                
            if abs(current_yaw) < self._threshold[2]:
                next_yaw = 0.0  # yaw方向已满足阈值，不移动
                rospy.loginfo(f"Yaw direction within threshold ({abs(current_yaw):.3f} < {self._threshold[2]:.3f}), no movement")
            else:
                next_yaw = min(max_yaw_step, abs(current_yaw)) * np.sign(current_yaw)

            step = [round(next_x, 3), round(next_y, 3), 0, round(next_yaw, 3)]
            
            # 检查是否所有方向都不需要移动
            if next_x == 0.0 and next_y == 0.0 and next_yaw == 0.0:
                rospy.loginfo("All directions within threshold, no movement needed. Continuing to next iteration...")
                rospy.sleep(detection_timeout)
                return True
                
            step_count += 1

            # Calculate estimated total steps
            remaining_x = abs(current_x) - abs(next_x)
            remaining_y = abs(current_y) - abs(next_y)
            remaining_yaw = abs(current_yaw) - abs(next_yaw)
            estimated_total = step_count + max(
                int(np.ceil(remaining_x / max_x_step)) if remaining_x > 0 else 0,
                int(np.ceil(remaining_y / max_y_step)) if remaining_y > 0 else 0,
                int(np.ceil(remaining_yaw / max_yaw_step)) if remaining_yaw > 0 else 0
            )

            rospy.loginfo(f"Planned step {step_count}: {step}")

            # Publish status before executing step
            status_msg = StairAlignmentStatus()
            status_msg.tag_id = tag_id
            status_msg.current_state = "aligning"
            status_msg.current_x = current_x
            status_msg.current_y = current_y
            status_msg.current_yaw = current_yaw
            status_msg.target_x = offset_x
            status_msg.target_y = offset_y
            status_msg.target_yaw = offset_yaw
            status_msg.step_count = step_count
            status_msg.total_steps = estimated_total
            status_msg.message = f"Executing step {step_count}: moving by [{next_x:.3f}, {next_y:.3f}, {next_yaw:.3f}]"
            status_msg.is_aligned = False
            status_pub.publish(status_msg)

            # Prepare time trajectory for a single step
            time_traj = [step_duration]
            torso_traj = [step]

            # Call single_step_control for this step
            success = self.call_single_step_control(time_traj, torso_traj)

            if not success:
                rospy.logerr("Failed to execute step. Retrying...")
                # Publish failure status
                status_msg.message = f"Step {step_count} failed, retrying..."
                status_pub.publish(status_msg)
                rospy.sleep(1)
                continue

            # Wait for the step to complete
            rospy.sleep(step_duration + wait_buffer)

        rospy.loginfo("Finished close_to_tag_with_status operation.")
        return False

    def move_to_target_position_gazebo(self, target_x, target_y, target_yaw=0.0, coordinate_frame="odom"):
        """
        基于Gazebo仿真环境的目标位置移动函数
        
        Args:
            target_x: 目标X坐标 (米)
            target_y: 目标Y坐标 (米) 
            target_yaw: 目标偏航角 (弧度)
            coordinate_frame: 坐标系，默认为"odom"
        
        Returns:
            bool: 是否成功到达目标位置
        """
        # Get maximum step sizes from config
        max_x_step = self._max_step_sizes.get("max_x_step", 0.04)
        max_y_step = self._max_step_sizes.get("max_y_step", 0.03)
        max_yaw_step_deg = self._max_step_sizes.get("max_yaw_step_deg", 25)
        max_yaw_step = np.radians(max_yaw_step_deg)
        
        # Get timing parameters from config
        step_duration = self._timing_params.get("step_duration", 1.2)
        wait_buffer = self._timing_params.get("wait_buffer", 2.0)
        
        rospy.loginfo(f"Moving to target position: x={target_x:.3f}, y={target_y:.3f}, yaw={np.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"Using config: max_x_step={max_x_step}, max_y_step={max_y_step}, max_yaw_step={max_yaw_step_deg}°")
        
        step_count = 0
        max_attempts = 100  # 防止无限循环
        
        while not rospy.is_shutdown() and step_count < max_attempts:
            # 获取当前机器人位置
            current_pose = self.get_current_robot_pose()
            if current_pose is None:
                rospy.logerr("Failed to get current robot pose")
                rospy.sleep(1.0)
                continue
            
            # 计算当前位置与目标位置的差值
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            _, _, current_yaw = euler_from_quaternion([
                current_pose.orientation.x,
                current_pose.orientation.y, 
                current_pose.orientation.z,
                current_pose.orientation.w
            ])
            
            # 计算需要移动的距离和角度
            delta_x = target_x - current_x
            delta_y = target_y - current_y
            delta_yaw = target_yaw - current_yaw
            
            # 处理角度归一化 (-π, π]
            while delta_yaw > np.pi:
                delta_yaw -= 2 * np.pi
            while delta_yaw <= -np.pi:
                delta_yaw += 2 * np.pi
            
            rospy.loginfo(f"Current pose: x={current_x:.3f}, y={current_y:.3f}, yaw={np.degrees(current_yaw):.1f}°")
            rospy.loginfo(f"Target delta: dx={delta_x:.3f}, dy={delta_y:.3f}, dyaw={np.degrees(delta_yaw):.1f}°")
            
            # 检查是否已经到达目标位置
            if self._check_threshold(delta_x, delta_y, delta_yaw):
                rospy.loginfo("Successfully reached target position!")
                return True
            
            # 计算下一步的移动量（限制在最大步长内）
            next_x = min(max_x_step, abs(delta_x)) * np.sign(delta_x)
            next_y = min(max_y_step, abs(delta_y)) * np.sign(delta_y)
            next_yaw = min(max_yaw_step, abs(delta_yaw)) * np.sign(delta_yaw)
            
            step = [round(next_x, 3), round(next_y, 3), 0, round(next_yaw, 3)]
            step_count += 1
            
            rospy.loginfo(f"Step {step_count}: Moving by [{next_x:.3f}, {next_y:.3f}, {np.degrees(next_yaw):.1f}°]")
            
            # 准备轨迹数据
            time_traj = [step_duration]
            torso_traj = [step]
            
            # 执行单步控制
            success = self.call_single_step_control(time_traj, torso_traj)
            
            if not success:
                rospy.logerr(f"Failed to execute step {step_count}. Retrying...")
                rospy.sleep(1)
                continue
            
            # 等待步骤完成
            rospy.sleep(step_duration + wait_buffer)
        
        if step_count >= max_attempts:
            rospy.logerr(f"Failed to reach target position after {max_attempts} attempts")
            return False
        
        rospy.loginfo("Finished move_to_target_position_gazebo operation.")
        return True

    def test_gazebo_navigation(self):
        """
        测试函数：在Gazebo环境中进行导航测试
        """
        rospy.loginfo("Starting Gazebo navigation test...")
        
        # 等待系统初始化
        rospy.sleep(2.0)
        
        # 测试移动到几个不同的目标点
        test_targets = [
            (1.0, 0.0, 0.0),      # 前进1米
            (1.0, 1.0, np.pi/2),  # 移动到(1,1)并转向90度
            (0.0, 1.0, np.pi),    # 移动到(0,1)并转向180度
            (0.0, 0.0, 0.0),      # 回到原点
        ]
        
        for i, (target_x, target_y, target_yaw) in enumerate(test_targets):
            rospy.loginfo(f"\n=== Test {i+1}: Moving to ({target_x}, {target_y}, {np.degrees(target_yaw):.1f}°) ===")
            
            success = self.move_to_target_position_gazebo(target_x, target_y, target_yaw)
            
            if success:
                rospy.loginfo(f"✓ Test {i+1} completed successfully!")
            else:
                rospy.logerr(f"✗ Test {i+1} failed!")
                break
            
            # 在目标点停留一段时间
            rospy.sleep(3.0)
        
        rospy.loginfo("Gazebo navigation test completed!")


if __name__ == "__main__":
    rospy.init_node('stair_close_to_tag_node', anonymous=True)
    
    # 从命令行参数获取测试模式
    import sys
    test_mode = "apriltag"  # 默认模式
    
    if len(sys.argv) > 1:
        test_mode = sys.argv[1]
    
    stair_tag_id = 1  # Default stair tag ID
    stair_close_to_tag_node = StairCloseToTagNode(stair_tag_id)
    
    if test_mode == "gazebo":
        rospy.loginfo("Running in Gazebo test mode...")
        stair_close_to_tag_node.test_gazebo_navigation()
    elif test_mode == "move_to":
        # 从命令行参数获取目标位置
        if len(sys.argv) >= 5:
            target_x = float(sys.argv[2])
            target_y = float(sys.argv[3]) 
            target_yaw = float(sys.argv[4])
            rospy.loginfo(f"Moving to target position: ({target_x}, {target_y}, {target_yaw})")
            success = stair_close_to_tag_node.move_to_target_position_gazebo(target_x, target_y, target_yaw)
            if success:
                rospy.loginfo("Successfully reached target position!")
            else:
                rospy.logerr("Failed to reach target position!")
        else:
            rospy.logerr("Usage: python stair_close_to_tag.py move_to <x> <y> <yaw_radians>")
    else:
        # 默认AprilTag模式
        rospy.loginfo("Running in AprilTag mode...")
        while not rospy.is_shutdown():
            import time
            time.sleep(1)
            stair_close_to_tag_node.close_to_tag()
            break

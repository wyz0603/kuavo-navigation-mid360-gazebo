#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import yaml
import json
import os
from geometry_msgs.msg import Pose, PoseArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from kuavo_msgs.srv import changeArmCtrlMode, twoArmHandPoseCmdSrv
from kuavo_msgs.msg import robotHandPosition, sensorsData
import numpy as np
from scipy.interpolate import CubicSpline
import copy
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam, twoArmHandPose
from geometry_msgs.msg import Transform, TransformStamped

import copy

reverse_joint_states_index = [1, 2, 4, 5]

class Robot:
    def __init__(self):
        """Initialize Robot class instance"""
        rospy.loginfo("Initializing Robot...")
        
        # Initialize TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Robot state
        self.is_gripper_open = True
        self.current_left_end_effector_pose = None
        self.current_right_end_effector_pose = None
        self.control_hand_side = rospy.get_param('~control_hand_side', 1)  # Default to right hand
        # Add a variable to track which arm was used for picking
        self.picking_arm = "left" if self.control_hand_side == 0 else "right"
        # End effector frames
        self.left_ee_frame = "zarm_l7_end_effector"  # Updated to match the keyboard control script
        self.right_ee_frame = "zarm_r7_end_effector"  # Updated to match the keyboard control script
        self.base_frame = "base_link"
        
        # Load pick and place parameters from ROS parameters
        self.load_pick_place_params()
        
        # IK parameters
        self.use_custom_ik_param = True
        self.joint_angles_as_q0 = True
        self.ik_solve_param = ikSolveParam()
        # SNOPT parameters
        self.ik_solve_param.major_optimality_tol = 1e-3
        self.ik_solve_param.major_feasibility_tol = 1e-3
        self.ik_solve_param.minor_feasibility_tol = 1e-3
        self.ik_solve_param.major_iterations_limit = 100
        # Constraint and cost parameters
        self.ik_solve_param.oritation_constraint_tol = 1e-3
        self.ik_solve_param.pos_constraint_tol = 1e-4
        self.ik_solve_param.pos_cost_weight = 10.0
        self.joint_names = ['zarm_l7_joint_1', 'zarm_l7_joint_2', 'zarm_l7_joint_3', 'zarm_l7_joint_4', 'zarm_l7_joint_5', 'zarm_l7_joint_6', 'zarm_l7_joint_7', 'zarm_r7_joint_1', 'zarm_r7_joint_2', 'zarm_r7_joint_3', 'zarm_r7_joint_4', 'zarm_r7_joint_5', 'zarm_r7_joint_6', 'zarm_r7_joint_7']
        
        # Initialize publishers and subscribers
        self.joint_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        self.gripper_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=1, tcp_nodelay=True)
        
        # IK service client
        self.ik_service_client = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)

        self.sensor_data_sub = rospy.Subscriber(
            '/sensors_data_raw', 
            sensorsData, 
            self.sensor_data_callback, 
            queue_size=1, 
            tcp_nodelay=True
        )
        
        
        # 添加PoseArray发布器用于轨迹可视化
        self.trajectory_pub = rospy.Publisher('/trajectory_visualization', PoseArray, queue_size=10)
        
        self.left_ee_pose_pub = rospy.Publisher('/left_ee_pose', Pose, queue_size=10)
        self.right_ee_pose_pub = rospy.Publisher('/right_ee_pose', Pose, queue_size=10)
        
        self.tf_pub_timer = rospy.Timer(rospy.Duration(0.01), self.publish_ee_poses)  # 20Hz

        self.has_wrist = rospy.get_param('~has_wrist', True)
        
        # Wait for IK service to be available
        rospy.loginfo("Waiting for IK service to be available...")
        try:
            rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv', timeout=10.0)
            rospy.loginfo("IK service is available")
        except rospy.ROSException:
            rospy.logerr("IK service not available after 10 seconds")
            raise
        
        rospy.loginfo("Robot initialized successfully")
    
    def load_pick_place_params(self):
        """Load pick and place parameters from ROS parameter server"""
        rospy.loginfo("Loading pick and place parameters from ROS parameter server...")
        
        # Create dictionaries to store configuration for different pose types
        self.pose_configs = {}
        
        # Set up pose types we'll use
        pose_types = ['pick_grasp', 'place_place']
        
        self.pose_configs['pick_grasp'] = {
            'x_offset': rospy.get_param('/offsets/pick/grasp_x', 0.0),
            'y_offset': rospy.get_param('/offsets/pick/grasp_y', 0.0),
            'z_offset': rospy.get_param('/offsets/pick/grasp_z', 0.05),
            'roll': math.radians(rospy.get_param('/orientation/pick/roll', 0.0)),
            'pitch': math.radians(rospy.get_param('/orientation/pick/pitch', 90.0)),
            'yaw': math.radians(rospy.get_param('/orientation/pick/yaw', 0.0))
        }
        
        
        self.pose_configs['place_place'] = {
            'x_offset': rospy.get_param('/offsets/place/place_x', 0.0),
            'y_offset': rospy.get_param('/offsets/place/place_y', 0.0),
            'z_offset': rospy.get_param('/offsets/place/place_z', 0.05),
            'roll': math.radians(rospy.get_param('/orientation/place/roll', 0.0)),
            'pitch': math.radians(rospy.get_param('/orientation/place/pitch', 90.0)),
            'yaw': math.radians(rospy.get_param('/orientation/place/yaw', 0.0))
        }
        

        
        
        # Load path planning parameters
        self.path_points = rospy.get_param('/path/points', 200)
        self.arc_height_factor = rospy.get_param('/path/arc_height_factor', 0.25)
        self.max_arc_height = rospy.get_param('/path/max_arc_height', 0.15)
        
        # Gripper parameters
        self.open_hand = rospy.get_param('/gripper/open_hand', [0, 0, 0, 0, 0, 0])
        self.view_hand = [0, 100, 0, 0, 0, 0]
        self.close_hand = rospy.get_param('/gripper/close_hand', [100, 100, 80, 75, 75, 75])
        
        # Load joint transition path configuration
        self.transition_joint_states = {}
        joint_poses = rospy.get_param('/home_to_transition_path_poses', {})
        for pose_name, joint_values in joint_poses.items():
            # Create 14-dimensional joint array (left arm + right arm)
            full_joints = [0.0]*14
            if self.picking_arm == "left":
                full_joints[0:7] = joint_values  # Left arm uses configured values
                for i in reverse_joint_states_index:
                    full_joints[i] = -full_joints[i]
            else:
                full_joints[7:14] = joint_values  # Right arm uses configured values
            self.transition_joint_states[pose_name] = np.degrees(full_joints)  # Convert radians to degrees
        
        rospy.loginfo("Pick and place parameters loaded successfully")
    
    def compute_ik(self, poses_dict):
        """Compute IK for both arms based on the poses dictionary
        """
        
        # Create IK command message
        ik_cmd = twoArmHandPoseCmd()
        ik_cmd.ik_param = self.ik_solve_param
        ik_cmd.use_custom_ik_param = self.use_custom_ik_param
        ik_cmd.joint_angles_as_q0 = self.joint_angles_as_q0
        
        # Initialize joint angles (will be filled by IK solver)
        ik_cmd.hand_poses.left_pose.joint_angles = self.current_arm_joint_state[0:7]
        ik_cmd.hand_poses.right_pose.joint_angles = self.current_arm_joint_state[7:14]
        
        # Set elbow positions (not used in this implementation)
        ik_cmd.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)
        ik_cmd.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
        
        # Process left arm pose
        
        # Set left arm pose
        ik_cmd.hand_poses.left_pose.pos_xyz = [
            poses_dict['left'].transform.translation.x,
            poses_dict['left'].transform.translation.y,
            poses_dict['left'].transform.translation.z
        ]
        ik_cmd.hand_poses.left_pose.quat_xyzw = [
            poses_dict['left'].transform.rotation.x,
            poses_dict['left'].transform.rotation.y,
            poses_dict['left'].transform.rotation.z,
            poses_dict['left'].transform.rotation.w
        ]
        
        
        # Set right arm pose
        ik_cmd.hand_poses.right_pose.pos_xyz = [
            poses_dict['right'].transform.translation.x,
            poses_dict['right'].transform.translation.y,
            poses_dict['right'].transform.translation.z
        ]
        ik_cmd.hand_poses.right_pose.quat_xyzw = [
            poses_dict['right'].transform.rotation.x,
            poses_dict['right'].transform.rotation.y,
            poses_dict['right'].transform.rotation.z,
            poses_dict['right'].transform.rotation.w
        ]
        
        try:
            # Call IK service
            response = self.ik_service_client(ik_cmd)
            
            if response.success:
                rospy.loginfo(f"IK solved successfully in {response.time_cost:.2f}ms")
                return response.q_arm
            else:
                rospy.logerr("IK service call failed")
                return None
                
        except rospy.ServiceException as e:
            rospy.logerr(f"IK service call failed: {e}")
            return None

    def cubic_spline_interpolate(self, start_transform, end_pose, num_points=200):
        """Generate a path using cubic spline interpolation for position and slerp for rotation"""
        
        if num_points is None:
            num_points = self.path_points
        
        # Convert end_pose to position array for easier calculation
        end_pos = np.array([
            end_pose.position.x,
            end_pose.position.y,
            end_pose.position.z
        ])
        
        # Get start position from transform
        start_pos = np.array([
            start_transform.transform.translation.x,
            start_transform.transform.translation.y,
            start_transform.transform.translation.z
        ])
        
        # Get start rotation as quaternion
        start_quat = np.array([
            start_transform.transform.rotation.x,
            start_transform.transform.rotation.y,
            start_transform.transform.rotation.z,
            start_transform.transform.rotation.w
        ])
        
        # Get end rotation as quaternion
        end_quat = np.array([
            end_pose.orientation.x,
            end_pose.orientation.y,
            end_pose.orientation.z,
            end_pose.orientation.w
        ])
        
        # If end quaternion is all zeros, use start quaternion
        if np.all(end_quat == 0):
            end_quat = start_quat
        
        # Normalize quaternions to ensure proper interpolation
        start_quat = start_quat / np.linalg.norm(start_quat)
        end_quat = end_quat / np.linalg.norm(end_quat)
        
        # Generate path transforms
        path_transforms = []
        
        # Calculate the distance between start and end positions
        distance = np.linalg.norm(end_pos - start_pos)
        
        # Define control points for cubic spline
        # We'll use 4 control points: start, after start, before end, end
        # This allows us to control the entry and exit velocities
        t_points = np.array([0.0, 0.3, 0.7, 1.0])
        
        # Add a mid-point with higher z value to create an arc
        control_points = np.zeros((4, 3))
        control_points[0] = start_pos
        control_points[3] = end_pos
        
        # Calculate direction vector from start to end
        direction = end_pos - start_pos
        
        # Generate intermediate control points with higher z
        control_points[1] = start_pos + 0.3 * direction
        control_points[2] = start_pos + 0.7 * direction
        
        # Add arc if the distance is significant
        if distance > 0.1:
            arc_height = min(0.15, distance * 0.25)  # Arc height proportional to distance, but capped
            control_points[1][2] += arc_height * 0.7  # First intermediate point: 70% of max height
            control_points[2][2] += arc_height  # Second intermediate point: max height
        
        # Create cubic spline for each dimension (x, y, z)
        cs_x = CubicSpline(t_points, control_points[:, 0])
        cs_y = CubicSpline(t_points, control_points[:, 1])
        cs_z = CubicSpline(t_points, control_points[:, 2])
        
        # Create PoseArray for trajectory visualization
        trajectory_msg = PoseArray()
        trajectory_msg.header.frame_id = self.base_frame
        trajectory_msg.header.stamp = rospy.Time.now()
        
        # Generate points along the spline
        for i in range(num_points):
            # Calculate interpolation parameter (0 to 1)
            t = i / (num_points - 1) if num_points > 1 else 0
            
            # Calculate position using cubic spline
            pos_x = cs_x(t)
            pos_y = cs_y(t)
            pos_z = cs_z(t)
            
            # Create a new transform for this waypoint
            new_transform = copy.deepcopy(start_transform)
            
            # Update position in transform
            new_transform.transform.translation.x = float(pos_x)
            new_transform.transform.translation.y = float(pos_y)
            new_transform.transform.translation.z = float(pos_z)
            
            # Perform spherical linear interpolation (Slerp) for rotation
            # Calculate the dot product between quaternions
            dot = np.sum(start_quat * end_quat)
            
            # If the dot product is negative, negate one quaternion to take the shorter path
            if dot < 0.0:
                end_quat = -end_quat
                dot = -dot
            
            # Clamp dot product to valid range for acos
            dot = min(1.0, max(-1.0, dot))
            
            # Calculate the angle between quaternions
            theta = np.arccos(abs(dot))
            
            # If the angle is very small, use linear interpolation
            if theta < 1e-6:
                interp_quat = (1 - t) * start_quat + t * end_quat
            else:
                # Perform Slerp
                sin_theta = np.sin(theta)
                interp_quat = (np.sin((1 - t) * theta) / sin_theta) * start_quat + (np.sin(t * theta) / sin_theta) * end_quat
            
            # Normalize the interpolated quaternion
            interp_quat = interp_quat / np.linalg.norm(interp_quat)
            
            # Update rotation in transform
            new_transform.transform.rotation.x = float(interp_quat[0])
            new_transform.transform.rotation.y = float(interp_quat[1])
            new_transform.transform.rotation.z = float(interp_quat[2])
            new_transform.transform.rotation.w = float(interp_quat[3])
            
            # Create Pose message for trajectory visualization
            traj_pose = Pose()
            traj_pose.position.x = float(pos_x)
            traj_pose.position.y = float(pos_y)
            traj_pose.position.z = float(pos_z)
            traj_pose.orientation.x = float(interp_quat[0])
            traj_pose.orientation.y = float(interp_quat[1])
            traj_pose.orientation.z = float(interp_quat[2])
            traj_pose.orientation.w = float(interp_quat[3])
            
            # Add to trajectory message
            trajectory_msg.poses.append(traj_pose)
            
            # Add to path
            path_transforms.append(new_transform)
        
        # Publish visualization message
        self.trajectory_pub.publish(trajectory_msg)
        rospy.loginfo(f"Published trajectory with {len(trajectory_msg.poses)} waypoints")
        
        return path_transforms

    def move_to_pose(self, target_pose, arm=None):
        """Move robot to specified pose using IK service and direct joint control"""
        rospy.loginfo(f"Using {arm} arm for movement")
        
        # Get current transform of the selected arm
        current_transform = self.get_transform(self.base_frame, 
                                              self.left_ee_frame if arm == "left" else self.right_ee_frame)
        
        if current_transform is None:
            rospy.logerr(f"Failed to get current transform for {arm} arm")
            return False
        
        # Get current transform of the other arm
        other_arm = "right" if arm == "left" else "left"
        other_transform = self.get_transform(self.base_frame,
                                            self.left_ee_frame if other_arm == "left" else self.right_ee_frame)
        
        if other_transform is None:
            rospy.logerr(f"Failed to get current transform for {other_arm} arm")
            return False
        
        # Generate interpolation path
        path_transforms = self.cubic_spline_interpolate(current_transform, target_pose, self.path_points)
        
        # For each point in the path, compute IK and move
        for i, transform in enumerate(path_transforms):
            
            if ( i == len(path_transforms) / 2  and self.has_wrist) :
                rospy.loginfo("靠近物体位置，使用手腕相机进一步调整位置")
                rospy.sleep(2)
                object_transform = self.get_transform("base_link", "wrist_marker_object_frame")
                place_transform = self.get_transform("base_link", "wrist_marker_place_frame")

            # Create poses dictionary for IK computation
            poses_dict = {
                "left": transform if arm == "left" else other_transform,
                "right": transform if arm == "right" else other_transform,
            }
            
            # Compute IK for this pose using service
            joint_angles = self.compute_ik(poses_dict)
            
            if joint_angles is not None:
                # Publish joint trajectory directly
                joint_msg = JointState()
                joint_msg.header.stamp = rospy.Time.now()
                joint_msg.name = self.joint_names
                joint_angles = list(joint_angles)
                if arm == "right":
                    joint_angles = list(self.current_arm_joint_state[0:7]) + list(joint_angles[7:14])
                else:
                    joint_angles = list(joint_angles[0:7]) + list(self.current_arm_joint_state[7:14])
                joint_msg.position = [math.degrees(i) for i in joint_angles]
                self.joint_pub.publish(joint_msg)
                rospy.sleep(0.01)  # Small delay between waypoints
            else:
                rospy.logerr(f"Failed to compute IK for waypoint {i}")
                return False
        final_poses_dict = {
            "left": self.pose_to_transformstamped(target_pose) if arm == "left" else other_transform,
            "right": self.pose_to_transformstamped(target_pose) if arm == "right" else other_transform,
        }
        
        rospy.loginfo("Sending final pose command multiple times to increase precision...")
        for _ in range(10): 
            joint_angles = self.compute_ik(final_poses_dict)
            if joint_angles is not None:
                joint_msg = JointState()
                joint_msg.header.stamp = rospy.Time.now()
                joint_msg.name = self.joint_names
                if arm == "right":
                    joint_angles = list(self.current_arm_joint_state[0:7]) + list(joint_angles[7:14])
                else:
                    joint_angles = list(joint_angles[0:7]) + list(self.current_arm_joint_state[7:14])
                joint_msg.position = [math.degrees(i) for i in joint_angles]

                self.joint_pub.publish(joint_msg)
            rospy.sleep(0.01) 
        
        rospy.loginfo("Movement completed")
        return True
    
    def sensor_data_callback(self, msg):
        self.current_arm_joint_state = msg.joint_data.joint_q[12:26]

    def call_change_arm_ctrl_mode_service(self, arm_ctrl_mode):
        result = False
        service_name = "arm_traj_change_mode"
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            change_arm_ctrl_mode = rospy.ServiceProxy(
                service_name, changeArmCtrlMode
            )
            print(change_arm_ctrl_mode(control_mode=arm_ctrl_mode))
            rospy.loginfo("Change arm ctrl mode Service call successful")
            result = True
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
            result = False
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
            result = False
        finally:
            return result
        

    def load_marker_config(self, config_file):
        """Load ArUco marker configuration"""
        rospy.loginfo(f"Loading marker configuration from {config_file}")
        
        file_extension = os.path.splitext(config_file)[1].lower()
        
        try:
            with open(config_file, 'r') as file:
                if file_extension == '.yaml' or file_extension == '.yml':
                    config = yaml.safe_load(file)
                elif file_extension == '.json':
                    config = json.load(file)
                else:
                    rospy.logerr(f"Unsupported file format: {file_extension}")
                    return None
                    
            rospy.loginfo(f"Marker configuration loaded successfully")
            return config
        except Exception as e:
            rospy.logerr(f"Error loading marker configuration: {e}")
            return None
    
    def get_transform(self, target_frame, source_frame, timeout=1.0):
        """Get TF transform"""
        
        try:
            # Wait for transform to be available
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(timeout)
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # rospy.logerr(f"Failed to get transform: {e}")
            return None
    
    def transform_to_pose(self, transform):
        """Convert TF transform to Pose"""
        pose = Pose()
        
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        
        pose.orientation.x = transform.transform.rotation.x
        pose.orientation.y = transform.transform.rotation.y
        pose.orientation.z = transform.transform.rotation.z
        pose.orientation.w = transform.transform.rotation.w
        
        return pose

    def pose_to_transformstamped(self, pose):
        """Convert Pose to TF transform"""
        transform = TransformStamped()
        
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        
        transform.transform.rotation.x = pose.orientation.x
        transform.transform.rotation.y = pose.orientation.y
        transform.transform.rotation.z = pose.orientation.z
        transform.transform.rotation.w = pose.orientation.w

        return transform
        
    def detect_object(self, marker_config):
        """Detect object using ArUco marker"""
        rospy.loginfo("Detecting target object using ArUco marker...")
        
        has_wrist = rospy.get_param('~has_wrist', True)
        
        # 首先使用head摄像头检测marker
        marker_frame = marker_config['marker_object_frame']['frame_name']
        base_frame = 'base_link'
        
        # Get transform from base to marker
        transform = self.get_transform(base_frame, marker_frame)
        
        if transform is None:
            rospy.logerr(f"Could not find the object marker with head camera: {marker_frame}")
            return None
        
        # Convert to Pose
        object_pose = self.transform_to_pose(transform)
        
        rospy.loginfo(f"Object detected at position: x={object_pose.position.x}, y={object_pose.position.y}, z={object_pose.position.z}")
        
        return object_pose

    def define_place_location(self, marker_config):
        """Define place location using ArUco marker"""
        rospy.loginfo("Defining place location using ArUco marker...")
        
        # 首先使用head摄像头检测marker
        marker_frame = marker_config['marker_place_frame']['frame_name']
        base_frame = 'base_link'
        
        # Get transform from base to marker
        transform = self.get_transform(base_frame, marker_frame)
        
        if transform is None:
            rospy.logerr(f"Could not find the placement marker with head camera: {marker_frame}")
            return None
        
        # Convert to Pose
        place_pose = self.transform_to_pose(transform)
        
        rospy.loginfo(f"Place location detected at position: x={place_pose.position.x}, y={place_pose.position.y}, z={place_pose.position.z}")
        
        return place_pose
    
    def view_gripper(self):
        """view gripper"""
        rospy.loginfo("Opening gripper to view...")
        
        # Create hand position message
        hand_pose_msg = robotHandPosition()
        if self.picking_arm == "left":
            hand_pose_msg.left_hand_position = self.view_hand
            hand_pose_msg.right_hand_position = self.open_hand
        else:
            hand_pose_msg.left_hand_position = self.open_hand
            hand_pose_msg.right_hand_position = self.view_hand

        
        # Publish hand position
        self.gripper_pub.publish(hand_pose_msg)
        rospy.sleep(0.5)  # Wait for gripper to open
        
        self.is_gripper_open = True
        rospy.loginfo("Gripper opened")
        return True
    
    def open_gripper(self):
        """Open gripper"""
        rospy.loginfo("Opening gripper...")
        
        # Create hand position message
        hand_pose_msg = robotHandPosition()
        hand_pose_msg.left_hand_position = self.open_hand
        hand_pose_msg.right_hand_position = self.open_hand
        
        # Publish hand position
        self.gripper_pub.publish(hand_pose_msg)
        rospy.sleep(0.5)  # Wait for gripper to open
        
        self.is_gripper_open = True
        rospy.loginfo("Gripper opened")
        return True
    
    def close_gripper(self):
        """Close gripper"""
        rospy.loginfo("Closing gripper...")
        
        # Create hand position message
        hand_pose_msg = robotHandPosition()
        if self.picking_arm == "left":
            hand_pose_msg.left_hand_position = self.close_hand
            hand_pose_msg.right_hand_position = self.open_hand
        else:
            hand_pose_msg.left_hand_position = self.open_hand
            hand_pose_msg.right_hand_position = self.close_hand
        
        # Publish hand position
        self.gripper_pub.publish(hand_pose_msg)
        rospy.sleep(0.5)  # Wait for gripper to close
        
        self.is_gripper_open = False
        rospy.loginfo("Gripper closed")
        return True
    
    def joint_cubic_spline_interpolate(self, trajectory, rate):
        """Improved joint space cubic spline interpolation (supports multi-segment trajectories)"""
        if len(trajectory) < 2:
            return [trajectory[0]['pos']]

        # Extract time and position data
        times = [point['time'] for point in trajectory]
        positions = [point['pos'] for point in trajectory]
        
        # Calculate total steps and interpolation times
        total_time = times[-1] - times[0]
        num_steps = int(total_time * rate)
        interpolation_times = np.linspace(times[0], times[-1], num_steps + 1)
        
        interpolated = []
        num_joints = len(positions[0])

        # Calculate interpolated positions for each joint
        for joint_idx in range(num_joints):
            joint_positions = []
            
            # Iterate over all time points for interpolation
            for t in interpolation_times:
                # Find the segment the current time belongs to
                if t >= times[-1]:
                    seg_idx = len(times) - 2
                else:
                    seg_idx = 0
                    for i in range(len(times)-1):
                        if times[i] <= t <= times[i+1]:
                            seg_idx = i
                            break
                
                # Get segment parameters
                x0 = times[seg_idx]
                x1 = times[seg_idx+1]
                y0 = positions[seg_idx][joint_idx]
                y1 = positions[seg_idx+1][joint_idx]
                
                # Calculate cubic spline coefficients (assuming zero start/end velocity)
                delta_x = x1 - x0
                a = y0
                b = 0.0  # Start velocity
                c = 3*(y1 - y0)/(delta_x**2)
                d = -2*(y1 - y0)/(delta_x**3)
                
                # Calculate interpolated position
                delta_t = t - x0
                pos = a + b*delta_t + c*delta_t**2 + d*delta_t**3
                joint_positions.append(pos)
            
            interpolated.append(joint_positions)
        
        # Transpose and convert to list
        return np.array(interpolated).T.tolist()

    def publish_joint_trajectory(self, joint_trajectory):
        """Publish joint trajectory (modified version)"""
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        
        for point in joint_trajectory:
            joint_msg.position = point
            joint_msg.name = self.joint_names
            self.joint_pub.publish(joint_msg)
            rospy.sleep(0.01)  # Maintain original publish rate
        
    def home_to_transition(self):
        """Move from home to transition pose using improved joint space method"""
        rospy.loginfo("Moving from home to transition pose")
        
        # Build trajectory data (with timestamps)
        joint_poses = list(self.transition_joint_states.values())
        trajectory = [
            {'time': i*1.0, 'pos': joint_poses[i]}  # Assume each segment takes 1 second
            for i in range(len(joint_poses))
        ]
        
        # Generate interpolated trajectory (100Hz rate)
        full_trajectory = self.joint_cubic_spline_interpolate(trajectory, rate=100)
        
        # Publish trajectory
        self.publish_joint_trajectory(full_trajectory)
        rospy.loginfo("Transition movement completed")
        return True

    def transition_to_home(self):
        """Return to home using improved joint space method"""
        rospy.loginfo("Returning to home from transition pose")
        
        os.system("rosnode kill /ik_ros_uni")
        # Reverse and build trajectory data
        joint_poses = list(self.transition_joint_states.values())
        joint_poses.append([math.degrees(q) for q in self.current_arm_joint_state])
        joint_poses.reverse()

        trajectory = [
            {'time': i*1.0, 'pos': joint_poses[i]}  # Assume each segment takes 1 second
            for i in range(len(joint_poses))
        ]
        
        # Generate interpolated trajectory
        full_trajectory = self.joint_cubic_spline_interpolate(trajectory, rate=100)
        
        # Publish trajectory
        self.publish_joint_trajectory(full_trajectory)
        rospy.loginfo("Return to home completed")
        return True
    
    def create_pose(self, reference_pose, pose_type):
        """Create a pose based on reference pose and pose type with specified offsets and orientation
        
        Args:
            reference_pose: The reference Pose
            pose_type: String indicating which type of pose to create (e.g., 'pick_pre_grasp')
        
        Returns:
            Pose: The created pose with applied offsets and orientation
        """
        if pose_type not in self.pose_configs:
            rospy.logerr(f"Unknown pose type: {pose_type}")
            return None
        
        config = self.pose_configs[pose_type]
        
        # Create new pose
        new_pose = Pose()
        
        # Apply position offsets
        new_pose.position.x = reference_pose.position.x + config['x_offset']
        new_pose.position.y = reference_pose.position.y + config['y_offset']
        new_pose.position.z = reference_pose.position.z + config['z_offset']
        # new_pose.orientation = reference_pose.orientation
        # Set orientation from RPY
        print(config['roll'], config['pitch'], config['yaw'])
        quat = quaternion_from_euler(config['roll'], config['pitch'], config['yaw'])
        new_pose.orientation.x = quat[0]
        new_pose.orientation.y = quat[1]
        new_pose.orientation.z = quat[2]
        new_pose.orientation.w = quat[3]
        
        return new_pose

    def apply_pid_control(self, target_pose, arm, max_attempts=5, threshold=0.002):
        """
        应用PID控制来精确调整机器人位置，直接使用IK而不进行路径插值
        
        Args:
            target_pose: 目标位姿
            arm: 使用的机械臂 ('left' or 'right')
            max_attempts: 最大尝试次数
            threshold: 位置误差阈值(米)
        
        Returns:
            bool: 是否成功达到目标精度
        """
        Kp = 0.5  # 比例系数
        Ki = 0.2  # 积分系数
        Kd = 0.2  # 微分系数
        
        integral_error = [0.0, 0.0, 0.0]  # x,y,z方向的积分误差
        last_error = [0.0, 0.0, 0.0]  # 上一次的误差
        
        # 获取另一个臂的当前位置（保持不变）
        other_arm = "right" if arm == "left" else "left"
        other_transform = self.get_transform(
            self.base_frame,
            self.left_ee_frame if other_arm == "left" else self.right_ee_frame
        )
        
        for attempt in range(max_attempts):
            actual_ee_transform = self.get_transform(
                self.base_frame, 
                self.left_ee_frame if arm == "left" else self.right_ee_frame
            )
            actual_ee_pose = self.transform_to_pose(actual_ee_transform)
            
            # 计算位置误差
            pos_error = [
                target_pose.position.x - actual_ee_pose.position.x,
                target_pose.position.y - actual_ee_pose.position.y,
                target_pose.position.z - actual_ee_pose.position.z
            ]
            
            # 如果误差已经足够小，退出循环
            if all(abs(e) < threshold for e in pos_error):
                rospy.loginfo("Position error within acceptable range")
                return True
            
            # 更新积分项
            integral_error = [integral_error[i] + pos_error[i] for i in range(3)]
            
            # 计算微分项
            derivative_error = [pos_error[i] - last_error[i] for i in range(3)]
            
            # PID控制计算补偿值
            compensation = [
                Kp * pos_error[i] + Ki * integral_error[i] + Kd * derivative_error[i]
                for i in range(3)
            ]
            
            # 更新目标位置
            adjusted_pose = copy.deepcopy(target_pose)
            adjusted_pose.position.x += compensation[0]
            adjusted_pose.position.y += compensation[1]
            adjusted_pose.position.z += compensation[2]
            
            rospy.loginfo(f"PID iteration {attempt + 1}: Adjusting position by {compensation}")
            
            # 直接使用IK计算新位置
            poses_dict = {
                "left": self.pose_to_transformstamped(adjusted_pose) if arm == "left" else other_transform,
                "right": self.pose_to_transformstamped(adjusted_pose) if arm == "right" else other_transform,
            }
            
            # 计算IK并等待执行完成
            joint_angles = self.compute_ik(poses_dict)
            if joint_angles is not None:
                # Publish joint trajectory directly
                joint_msg = JointState()
                joint_msg.header.stamp = rospy.Time.now()
                joint_msg.name = self.joint_names
                if arm == "right":
                    joint_angles = list(self.current_arm_joint_state[0:7]) + list(joint_angles[7:14])
                else:
                    joint_angles = list(joint_angles[0:7]) + list(self.current_arm_joint_state[7:14])
                joint_msg.position = [math.degrees(i) for i in joint_angles]
                self.joint_pub.publish(joint_msg)
            rospy.sleep(0.1)  # 给系统一个短暂的时间来执行命令
            
            last_error = pos_error
        
        rospy.logwarn("Failed to achieve desired precision after maximum attempts")
        return False

    def place_object(self, place_pose):
        """Place object"""
        rospy.loginfo("Placing object...")
        
        # Use the same arm that was used for picking
        if self.picking_arm is None:
            rospy.logerr("\033[31m未选择执行臂，无法执行放置动作\033[0m")
            return False
        
        rospy.loginfo(f"使用 {self.picking_arm} 手臂进行放置")
        
        # 创建目标放置位姿
        target_place_pose = self.create_pose(place_pose, 'place_place')
        
        # 移动到目标位置
        if not self.move_to_pose(target_place_pose, self.picking_arm):
            rospy.logerr("\033[31m移动到放置位置失败\033[0m")
            return False
        
        rospy.sleep(0.1)  # 等待IK计算完成
        
        # 应用PID控制来精确调整放置位置
        if not self.apply_pid_control(target_place_pose, self.picking_arm):
            rospy.logwarn("\033[31m未能达到期望的放置精度\033[0m")
        
        # 记录最终误差
        actual_ee_transform = self.get_transform(
            self.base_frame, 
            self.left_ee_frame if self.picking_arm == "left" else self.right_ee_frame
        )
        actual_ee_pose = self.transform_to_pose(actual_ee_transform)
        
        # 显示位置信息
        rospy.loginfo(f"目标位置: x={target_place_pose.position.x:.3f}, y={target_place_pose.position.y:.3f}, z={target_place_pose.position.z:.3f}")
        rospy.loginfo(f"实际位置: x={actual_ee_pose.position.x:.3f}, y={actual_ee_pose.position.y:.3f}, z={actual_ee_pose.position.z:.3f}")
        
        pos_error = self.calculate_position_error(target_place_pose, actual_ee_pose)
        ori_error = self.calculate_orientation_error(target_place_pose, actual_ee_pose)
        
        rospy.loginfo(f"放置精度 - 位置误差: {pos_error:.4f}m, 姿态误差: {ori_error:.4f}rad")
        
        # self.open_gripper()
        
        rospy.loginfo("物体放置成功")
        return True

    def pick_object(self, object_pose):
        """Pick object"""
        rospy.loginfo("Picking object...")
        
        rospy.loginfo(f"使用 {self.picking_arm} 手臂进行抓取")
        
        # self.open_gripper()
        # 创建目标抓取位姿
        target_grasp_pose = self.create_pose(object_pose, 'pick_grasp')
        
        # 移动到抓取位置
        if not self.move_to_pose(target_grasp_pose, self.picking_arm):
            rospy.logerr("\033[31m移动到抓取位置失败\033[0m")
            return False
        
        rospy.sleep(0.1)  # 等待IK计算完成

        # 应用PID控制来精确调整抓取位置
        if not self.apply_pid_control(target_grasp_pose, self.picking_arm):
            rospy.logwarn("\033[31m未能达到期望的抓取精度\033[0m")
        
        # 记录最终误差
        actual_ee_transform = self.get_transform(
            self.base_frame, 
            self.left_ee_frame if self.picking_arm == "left" else self.right_ee_frame
        )
        actual_ee_pose = self.transform_to_pose(actual_ee_transform)
        
        # 显示位置信息
        rospy.loginfo(f"目标位置: x={target_grasp_pose.position.x:.3f}, y={target_grasp_pose.position.y:.3f}, z={target_grasp_pose.position.z:.3f}")
        rospy.loginfo(f"实际位置: x={actual_ee_pose.position.x:.3f}, y={actual_ee_pose.position.y:.3f}, z={actual_ee_pose.position.z:.3f}")
        
        pos_error = self.calculate_position_error(target_grasp_pose, actual_ee_pose)
        ori_error = self.calculate_orientation_error(target_grasp_pose, actual_ee_pose)
        
        rospy.loginfo(f"抓取精度 - 位置误差: {pos_error:.4f}m, 姿态误差: {ori_error:.4f}rad")
        
        # 执行抓取动作
        #if not self.close_gripper():
        #    rospy.logerr("\033[31m抓取器关闭失败\033[0m")
        #    return False
        
        rospy.sleep(0.1)  # 等待抓取器动作完成
        
        rospy.loginfo("物体抓取成功")
        return True

    def calculate_position_error(self, target_pose, actual_pose):
        dx = target_pose.position.x - actual_pose.position.x
        dy = target_pose.position.y - actual_pose.position.y
        dz = target_pose.position.z - actual_pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_orientation_error(self, target_pose, actual_pose):
        q1 = [target_pose.orientation.x, target_pose.orientation.y, 
              target_pose.orientation.z, target_pose.orientation.w]
        q2 = [actual_pose.orientation.x, actual_pose.orientation.y, 
              actual_pose.orientation.z, actual_pose.orientation.w]
        
        dot = min(1.0, max(-1.0, q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]))
        
        return 2.0 * math.acos(abs(dot))

    def get_user_input(self, prompt):
        try:
            user_input = input(prompt)
            if user_input.lower() == 'q':
                rospy.loginfo("Demo interrupted by user (q)")
                return False
            return True
        except KeyboardInterrupt:
            rospy.loginfo("Demo interrupted by user (Ctrl+C)")
            return False

    def detect_with_retry(self, detect_type="object", marker_config=None):
        """
        通用的检测重试函数
        
        Args:
            detect_type: 检测类型，"object" 或 "place"
            marker_config: marker配置信息
        
        Returns:
            pose: 检测到的位姿，失败则返回None
        """
        has_wrist = rospy.get_param('~has_wrist', True)
        wrist_frame = rospy.get_param(
            f'~wrist_marker_{detect_type}_frame',
            f"wrist_marker_{detect_type}_frame"
        )
        
        while True:
            user_input = input(f"按回车键尝试检测{'物体' if detect_type == 'object' else '放置位置'}，按'q'退出检测: ")
            if user_input.lower() == 'q':
                rospy.loginfo("用户选择退出检测")
                return None
            
            # 首先尝试使用手腕相机检测
            if has_wrist:
                rospy.loginfo(f"尝试使用手腕相机检测{'物体' if detect_type == 'object' else '放置位置'}...")
                wrist_transform = self.get_transform(self.base_frame, wrist_frame)
                
                if wrist_transform is not None:
                    rospy.loginfo(f"手腕相机成功检测到{'物体' if detect_type == 'object' else '放置位置'}")
                    pose = self.transform_to_pose(wrist_transform)
                    break
                else:
                    rospy.logwarn("\033[31m手腕相机未检测到目标，尝试使用头部相机\033[0m")
            
            # 使用head相机检测
            rospy.loginfo("使用头部相机检测...")
            if detect_type == "object":
                pose = self.detect_object(marker_config)
            else:
                pose = self.define_place_location(marker_config)
            
            if pose is not None:
                rospy.loginfo("头部相机成功检测到目标")
                break
            else:
                rospy.logerr(f"\033[31m两个相机都未能检测到{'物体' if detect_type == 'object' else '放置位置'}，请调整后重试\033[0m")
        
        # 显示位置信息
        self.display_pose_info(pose, detect_type)
        return pose

    def display_pose_info(self, pose, detect_type):
        """显示位姿信息"""
        rpy = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        
        type_str = "物体" if detect_type == "object" else "放置位置"
        rospy.loginfo(f"{type_str}位置: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
        rospy.loginfo(f"{type_str}姿态(RPY): roll={math.degrees(rpy[0]):.1f}°, pitch={math.degrees(rpy[1]):.1f}°, yaw={math.degrees(rpy[2]):.1f}°")

    def execute_pick_and_place(self, marker_config):
        """Execute complete pick and place task"""
        try:
            # 初始化
            print("\n=== Interactive Pick and Place Demo ===")
            print("Press ENTER after each step to proceed to the next one.")
            print("Press Ctrl+C or 'q' to interrupt the demo at any time.")
            self.call_change_arm_ctrl_mode_service(2)
            if not self.get_user_input("Press ENTER to start the demo..."):
                return False
            
            # Step 1: 移动到过渡位置
            print("\n[Step 1] Moving from home to transition position")
            if not self.get_user_input("Press ENTER to move to transition position..."):
                return False
            
            if not self.home_to_transition():
                rospy.logerr("\033[31m移动到过渡位置失败\033[0m")
                return False
            
            # Step 2: 检测物体
            self.view_gripper()
            print("\n[Step 2] Detecting object")
            object_pose = self.detect_with_retry("object", marker_config)
            if object_pose is None:
                #self.open_gripper()
                self.transition_to_home()
                return False
            
            # Step 3: 抓取物体
            print("\n[Step 3] Picking object")
            if not self.get_user_input("Press ENTER to pick the object..."):
                return False
            self.open_gripper()
            if not self.pick_object(object_pose):
                rospy.logerr("\033[31m抓取物体失败，请检查后重试\033[0m")
                return False
            self.close_gripper()
            rospy.sleep(1)
            # Step 4: 检测放置位置
            print("\n[Step 4] Detecting place location")
            place_pose = self.detect_with_retry("place", marker_config)
            if place_pose is None:
                self.open_gripper()
                self.transition_to_home()
                return False
            
            # Step 5: 放置物体
            print("\n[Step 5] Placing object")
            print(f"使用 {self.picking_arm} 手臂进行放置")
            if not self.get_user_input("Press ENTER to place the object..."):
                return False
            if not self.place_object(place_pose):
                rospy.logerr("\033[31m放置物体失败，请检查后重试\033[0m")
                return False
            self.open_gripper()

            # Step 6: 返回初始位置
            rospy.sleep(1)
            print("\n[Step 6] Returning to home position")
            if not self.get_user_input("Press ENTER to return to home position..."):
                return False
            self.transition_to_home()
            
            print("\n=== Pick and Place Demo Completed ===")
            rospy.loginfo("任务成功完成！")
            return True
            
        except KeyboardInterrupt:
            rospy.loginfo("用户中断程序 (Ctrl+C)")
            return False
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS系统中断程序")
            return False
        except Exception as e:
            rospy.logerr(f"\033[31m程序执行出错: {e}\033[0m")
            return False

    def publish_ee_poses(self, event):
        try:
            left_transform = self.get_transform(self.base_frame, self.left_ee_frame)
            if left_transform:
                left_pose = self.transform_to_pose(left_transform)
                self.left_ee_pose_pub.publish(left_pose)
            
            right_transform = self.get_transform(self.base_frame, self.right_ee_frame)
            if right_transform:
                right_pose = self.transform_to_pose(right_transform)
                self.right_ee_pose_pub.publish(right_pose)
        
        except Exception as e:
            pass

# Main function
def main():
    try:
        # Initialize ROS node
        rospy.init_node('pick_and_place_node')
        
        # Create robot instance
        robot = Robot()
        
        # Load marker configuration
        config_file = rospy.get_param('~marker_config_file', 'config/markers.yaml')
        marker_config = robot.load_marker_config(config_file)
        
        if marker_config is None:
            rospy.logerr("Failed to load marker configuration. Exiting.")
            return
        
        # Execute pick and place task
        robot.execute_pick_and_place(marker_config)
        
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user (Ctrl+C)")
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by ROS shutdown")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

if __name__ == '__main__':
    main()

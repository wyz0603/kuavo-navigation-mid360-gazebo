#!/usr/bin/env python3

import rospy
import copy
import numpy as np
from tf.transformations import euler_from_quaternion
from ocs2_msgs.msg import footPose, footPoseTargetTrajectories
from ocs2_msgs.srv import singleStepControl, singleStepControlRequest
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose, Quaternion

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

class CloseToTagNode:
    def __init__(self, tag_id, expected_offset=[-0.4, 0, 0], threshold = [0.05, 0.05, np.radians(10)]) -> None:
        self._tag_id = tag_id
        self._tag_pose = None
        self._expected_offset = expected_offset
        self._threshold = threshold
        self._sub_apriltag = rospy.Subscriber("/robot_tag_info", AprilTagDetectionArray, self._detection_callback)
        self._pub_foot_pose_traj = rospy.Publisher('/humanoid_mpc_foot_pose_target_trajectories', footPoseTargetTrajectories, queue_size=10)

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
                
                # WARN! WARN! WARN!: apriltag 垂直于桌面表面张贴
                # 绕y轴旋转-90°
                rotated_quaternion = rotate_around_y_axis(current_quaternion, np.pi / 2)
                
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
                break

    def call_single_step_control(self, time_traj, torso_traj):
        single_step_control_srv = rospy.ServiceProxy('/humanoid_single_step_control', singleStepControl)
        req = singleStepControlRequest()
        foot_pose_target_trajectories = footPoseTargetTrajectories()
        foot_pose_target_trajectories.timeTrajectory = time_traj
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
        return abs(x) < self._threshold[0] and abs(y) < self._threshold[1] and abs(yaw) < self._threshold[2]

    def close_to_tag(self):
        # Define maximum step sizes for x, y, and yaw
        max_x_step = 0.05
        max_y_step = 0.04
        max_yaw_step = np.radians(30)
        
        while not rospy.is_shutdown():
            if self._tag_pose is None:
                rospy.logwarn("No tag detected. Waiting...")
                rospy.sleep(1)
                continue

            # Calculate target pose
            x = round(self._tag_pose.position.x + self._expected_offset[0], 3)
            y = round(self._tag_pose.position.y + self._expected_offset[1], 3)
            orientation_q = self._tag_pose.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            yaw = round(yaw + self._expected_offset[2], 3)

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
            time_traj = [i * 1.2 for i in range(1, len(steps) + 1)]  # Adjust time as needed
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
            rospy.sleep(total_time + 2)  # Add a small buffer

        rospy.loginfo("Finished close_to_tag operation.")
    
    def close_to_tag_step_by_step(self):
        # Define maximum step sizes for x, y, and yaw
        max_x_step = 0.05
        max_y_step = 0.04
        max_yaw_step = np.radians(30)
        
        while not rospy.is_shutdown():
            if self._tag_pose is None:
                rospy.logwarn("No tag detected. Waiting...")
                rospy.sleep(1)
                continue

            # Calculate target pose
            x = round(self._tag_pose.position.x + self._expected_offset[0], 3)
            y = round(self._tag_pose.position.y + self._expected_offset[1], 3)
            orientation_q = self._tag_pose.orientation
            _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            yaw = round(yaw + self._expected_offset[2], 3)

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
            time_traj = [1.2]  # Single step takes 1.2 seconds
            torso_traj = [step]

            # Call single_step_control for this step
            success = self.call_single_step_control(time_traj, torso_traj)

            if not success:
                rospy.logerr("Failed to execute step. Retrying...")
                rospy.sleep(1)
                continue

            # Wait for the step to complete
            rospy.sleep(1.2 + 1)  # Step time + buffer

        rospy.loginfo("Finished close_to_tag operation.")

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




if __name__ == "__main__":
    rospy.init_node('close_to_tag_node', anonymous=True)
    
    desktop_tag_id = 7  # Assuming the desktop tag ID is 1
    close_to_tag_node = CloseToTagNode(desktop_tag_id)
    while not rospy.is_shutdown():
        import time
        time.sleep(1)
        close_to_tag_node.close_to_tag()
        # rospy.sleep(10)
        # close_to_tag_node.backward_specify_distance(-0.5)      
        break

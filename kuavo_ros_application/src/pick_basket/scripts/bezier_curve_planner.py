#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os 
import json
import rospy
import threading
import time
import math
import numpy as np
from common.bezier_utils import Point, CurveCalculator
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from pick_basket.msg import planArmState
from pick_basket.msg import bezierCurveCubicPoint, jointBezierTrajectory
from pick_basket.srv import changeArmCtrlModeOCS2
from pick_basket.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from ocs2_msgs.msg import mpc_observation


INIT_ARM_POS = [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
START_FRAME_TIME = 0
x_shift = START_FRAME_TIME - 1
KEYFRAME_FACTOR = 80

def add_init_frame(frames):
    action_data = {}
    for frame in frames:
        servos, keyframe, attribute = frame["servos"], frame["keyframe"], frame["attribute"]
        for index, value in enumerate(servos):
            key = index + 1
            if key == 15:
                break
            if key not in action_data:
                action_data[key] = []
                if keyframe != 0 and len(action_data[key]) == 0:
                    if key <= len(INIT_ARM_POS):
                        action_data[key].append([
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                            [0, math.radians(INIT_ARM_POS[key-1])],
                ])
            if value is not None:
                CP = attribute[str(key)]["CP"]
                left_CP, right_CP = CP
                action_data[key].append([
                    [round(keyframe/KEYFRAME_FACTOR, 1), math.radians(value)],
                    [round((keyframe+left_CP[0])/KEYFRAME_FACTOR, 1), math.radians(value+left_CP[1])],
                    [round((keyframe+right_CP[0])/KEYFRAME_FACTOR, 1), math.radians(value+right_CP[1])],
                ])
    return action_data

import json

def dump_action_data_to_tact_file(action_data, file_path) -> list:
    if not action_data or not all(action_data):
        rospy.logerr("Invalid action_data provided")
        return []

    frames = []
    # print(action_data[0])
    for i in range(len(action_data[1])):
        servos = []
        keyframe = int(action_data[1][i][0][0]*KEYFRAME_FACTOR)
        attribute = {}    
        for index in range(len(action_data)):
            key = index + 1
            q = math.degrees(action_data[key][i][0][1])
            servos.append(round(q, 1))
            
            left_cp  = [action_data[key][i][1][0]*KEYFRAME_FACTOR - keyframe,
                        math.degrees(action_data[key][i][1][1]) - q]
            
            right_cp  = [action_data[key][i][2][0]*KEYFRAME_FACTOR - keyframe,
                        math.degrees(action_data[key][i][2][1]) - q]

            CP = [left_cp, right_cp]
            
            attr_item = {}
            attr_item["CP"] = CP
            attr_item["CPType"] = ["AUTO", "AUTO"]
            attr_item["select"] = False
            attribute[str(key)] = attr_item
        # servos = servos + [0.0]*(len(INIT_ARM_POS) - len(servos))   
        frame = {
            "servos": servos,
            "keyframe": keyframe,
            "attribute": attribute
        }
        frames.append(frame)    

    # Dump frames to JSON file
    try:
        with open(file_path, "w") as f:
            data = {}
            data["frames"] = frames
            json.dump(data, f, indent=4)
        rospy.loginfo(f"Successfully dumped {len(frames)} frames to {file_path}")
    except IOError as e:
        rospy.logerr(f"Error writing to file {file_path}: {e}")

    return frames

def load_action_frames_file(file_path) -> list:

    if not os.path.exists(file_path):
        rospy.logerr(f"File does not exist: {file_path}")
        return None
    try:
        with open(file_path, "r") as f:
            data = json.load(f)
            if not data:
                return None
        frames = data["frames"]
        action_data = add_init_frame(frames)
        rospy.loginfo(f"Successfully loaded frames from file {file_path}")
        return action_data
    except (IOError, json.JSONDecodeError, KeyError) as e:
        rospy.logerr(f"Error processing file {file_path}: {e}")
        return None

def pushfront_action_frames_rad(action_frames, time_gap, q_target_rad:list):
    q_target_deg = [math.degrees(q) for q in q_target_rad]
    return _pushfront_action_frames_rad(action_frames, time_gap, q_target_deg)

def pushback_action_frames_rad(action_frames, time_gap, q_target_rad:list):
    q_target_deg = [math.degrees(q) for q in q_target_rad]
    return _append_action_frames(action_frames, time_gap, q_target_deg)

def _pushfront_action_frames_rad(action_frames, time_gap, q_target:list):
    if len(q_target) != 14:
        print("append_action_frames, q_target should be of length 14!")
        return
    
    keyframe = (time_gap / 100) * KEYFRAME_FACTOR
    calculator = CurveCalculator()
    for index, q in enumerate(q_target):
        key = index + 1
        if key == 15:
            break
        
        q_keyframe = 0
        q0_keyframe = (keyframe + action_frames[key][0][0][0])*KEYFRAME_FACTOR
        q1_keyframe = (keyframe + action_frames[key][1][0][0])*KEYFRAME_FACTOR
        
        # 修改第一个点
        q0 = math.degrees(action_frames[key][0][0][1])  # 第一帧关节值
        q1 = math.degrees(action_frames[key][1][0][1])  # 第二帧关节值

        q_point = Point(q_keyframe, q)
        q0_point = Point(q0_keyframe, q0)
        q1_point = Point(q1_keyframe, q1)

        # 重新计算第一帧控制点
        left_CP, right_CP = calculator.get_control_point(prev_point = q_point, target_point=q0_point, next_point=q1_point)
        # 更新第一帧的左右控制点
        action_frames[key][0][0] = [round((q0_keyframe)/KEYFRAME_FACTOR, 1), math.radians(q0)]
        action_frames[key][0][1] = [round((q0_keyframe+left_CP[0])/KEYFRAME_FACTOR, 1), math.radians(q0 + left_CP[1])]
        action_frames[key][0][2] = [round((q0_keyframe+right_CP[0])/KEYFRAME_FACTOR, 1), math.radians(q0 + right_CP[1])]
        
        for j in range(1, len(action_frames[key])):
            # print("keyframe:", action_frames[key][j][0][0])
            # print("keyframe left", action_frames[key][j][1][0])
            # print("keyframe right",action_frames[key][j][2][0])
            action_frames[key][j][0][0] += keyframe
            action_frames[key][j][1][0] += keyframe
            action_frames[key][j][2][0] += keyframe
        
        # 计算插入的帧的控制点
        left_CP, right_CP = calculator.get_control_point(prev_point = q_point, target_point=q_point, next_point=q0_point)
        q_frame = [
            [0, math.radians(q)],
            [0, math.radians(q + left_CP[1])],
            [0, math.radians(q + right_CP[1])],
        ]
        action_frames[key].insert(0, q_frame)
def _append_action_frames(action_frames, time_gap, q_target:list):
    if len(q_target) != 14:
        print("append_action_frames, q_target should be of length 14!")
        return
    
    calculator = CurveCalculator()
    for index, q in enumerate(q_target):
        key = index + 1
        prev_keyframe = action_frames[key][-2][0][0]*KEYFRAME_FACTOR  # 倒数第二帧
        last_keyframe = action_frames[key][-1][0][0]*KEYFRAME_FACTOR  # 最后一帧
        append_keyframe = time_gap*KEYFRAME_FACTOR + last_keyframe    # 追加的帧  
        
        prev_q = math.degrees(action_frames[key][-2][0][1])           # 倒数第二帧关节值
        last_q = math.degrees(action_frames[key][-1][0][1])           # 最后一帧关节值
        
        prev_point = Point(prev_keyframe, prev_q)
        last_point = Point(last_keyframe, last_q)
        append_point = Point(append_keyframe,  q)
        
        # 重新计算最后一帧的控制点
        left_CP, right_CP = calculator.get_control_point(prev_point = prev_point, target_point=last_point, next_point=append_point)
        # 更新最后一帧的左右控制点
        action_frames[key][-1][1] = [round((last_keyframe+left_CP[0])/KEYFRAME_FACTOR, 1), math.radians(last_q + left_CP[1])]
        action_frames[key][-1][2] = [round((last_keyframe+right_CP[0])/KEYFRAME_FACTOR, 1), math.radians(last_q + right_CP[1])]
        
        # 计算末尾追加帧的控制点
        left_CP, right_CP = calculator.get_control_point(prev_point = last_point, target_point=append_point, next_point=None)
        end_point = [round(append_keyframe/KEYFRAME_FACTOR, 1), math.radians(q)]
        action_frames[key].append([
            end_point,
            [round((append_keyframe+left_CP[0])/KEYFRAME_FACTOR, 1), math.radians(q + left_CP[1])],
            end_point
        ])

def last_q_from_action_frames(action_frames) -> list:
    q = []
    for i in action_frames:
        end_point_i = action_frames[i][-1][0][1]
        q.append(end_point_i)
    return q

def first_q_from_action_frames(action_frames) -> list:
    q = []
    for i in action_frames:
        start_point_i = action_frames[i][0][0][1]
        q.append(start_point_i)
    return q

class ArmJointPublisher():
    def __init__(self):
        self.kuavo_arm_traj_pub = rospy.Publisher('/kuavo_arm_traj', JointState, queue_size=1, tcp_nodelay=True)
        self._traj_sub = rospy.Subscriber('/bezier/arm_traj', JointTrajectory, self._traj_callback,queue_size=1, tcp_nodelay=True)

        self.joint_state = JointState()
        self._running = True
        self._flag_publish = False

        """ Thread """
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.start()

    def start_publish(self):
        self._flag_publish = True

    def stop_publish(self):
        self._flag_publish = False

    def stop(self):
        self._running = False

    def _traj_callback(self, msg):
        if len(msg.points) == 0:
            return
        point = msg.points[0]
        self.joint_state.name = [
            "l_arm_pitch",
            "l_arm_roll",
            "l_arm_yaw",
            "l_forearm_pitch",
            "l_hand_yaw",
            "l_hand_pitch",
            "l_hand_roll",
            "r_arm_pitch",
            "r_arm_roll",
            "r_arm_yaw",
            "r_forearm_pitch",
            "r_hand_yaw",
            "r_hand_pitch",
            "r_hand_roll",
        ]

        # print("CubicSpline Trajectory Received")

        self.joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
        self.joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
        self.joint_state.effort = [0] * 14
        self.kuavo_arm_traj_pub.publish(self.joint_state)

    def _publish_loop(self):
        rate = 100
        while not rospy.is_shutdown() and self._running:
            try:
                if len(self.joint_state.position) == 0:
                    continue
                if not self._flag_publish:
                    continue
                self.kuavo_arm_traj_pub.publish(self.joint_state)
            except Exception as e:
                rospy.logerr(f"Failed to publish arm trajectory: {e}")
            except KeyboardInterrupt:
                break
            rospy.sleep(1/rate)

class BezierCurvePlanner():
    def __init__(self):
        """ Variables """
        self._srv_plan_bezier_curve_name = '/bezier/plan_arm_trajectory'
        self._current_arm_joint_state = []
        # joint_state = JointState()
        self._flag_planing = False
        self._arm_traj_state = planArmState()
        self._arm_traj_state.is_finished = False
        self._arm_traj_state.progress = 0

        """ ROS  """
        self.mpc_obs_sub = rospy.Subscriber('/humanoid_mpc_observation', mpc_observation, self._mpc_obs_callback)
        self._sub_arm_traj_state = rospy.Subscriber("/bezier/arm_traj_state", planArmState, self._arm_traj_state_callback)

        self._arm_joint_publisher = ArmJointPublisher() 

    def _arm_traj_state_callback(self, msg):
        """ 轨迹规划状态回调函数
        :param msg: planArmState
        """
        if not self._flag_planing:
            return
        
        self._arm_traj_state = msg

    def _mpc_obs_callback(self, msg):
        self._current_arm_joint_state = msg.state.value[24:]
        self._current_arm_joint_state = [round(pos, 2) for pos in self._current_arm_joint_state]
        self._current_arm_joint_state.extend([0] * 14)

    def _filter_data(self, action_data):
        filtered_action_data = {}
        for key, frames in action_data.items():
            filtered_frames = []
            found_start = False
            skip_next = False
            for i in range(-1, len(frames)):
                frame = frames[i]
                if i == len(frames) - 1:
                    next_frame = frame
                else:
                    next_frame = frames[i+1]
                end_time = next_frame[0][0]

                if not found_start and end_time >= START_FRAME_TIME:
                    found_start = True
                    
                    p0 = np.array([0, self._current_arm_joint_state[key-1]])
                    p3 = np.array([next_frame[0][0] - x_shift, next_frame[0][1]])
                    
                    # Calculate control points for smooth transition
                    curve_length = np.linalg.norm(p3 - p0)
                    p1 = p0 + curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the right
                    p2 = p3 - curve_length * 0.25 * np.array([1, 0])  # Move 1/4 curve length to the left
                    
                    # Create new frame
                    frame1 = [
                        p0.tolist(),
                        p0.tolist(),
                        p1.tolist()
                    ]
                    
                    # Modify next_frame's left control point
                    next_frame[1] = p2.tolist()
                    
                    filtered_frames.append(frame1)
                    skip_next = True
                
                if found_start:
                    if skip_next:
                        skip_next = False
                        continue
                    end_point = [round(frame[0][0] - x_shift, 1), round(frame[0][1], 1)]
                    left_control_point = [round(frame[1][0] - x_shift, 1), round(frame[1][1], 1)]
                    right_control_point = [round(frame[2][0] - x_shift, 1), round(frame[2][1], 1)]
                    filtered_frames.append([end_point, left_control_point, right_control_point])

            filtered_action_data[key] = filtered_frames
        return filtered_action_data

    def change_arm_ctrl_mode(self, arm_ctrl_mode):
        """ 切换手臂规划模式 
        :param control_mode: uint8, # 0: keep pose, 1: auto_swing_arm, 2: external_control 
        :return: bool, 服务调用结果
        """
        result = True
        service_name = 'humanoid_change_arm_ctrl_mode'
        try:
            rospy.wait_for_service(service_name, timeout=0.5)
            change_arm_ctrl_mode = rospy.ServiceProxy(
                service_name, changeArmCtrlModeOCS2
            )
            change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
            rospy.loginfo("Service call successful")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
            result = False
        except rospy.ROSException:
            rospy.logerr(f"Service {service_name} not available")
            result = False
        finally:
            return result
    
    def _create_bezier_request(self, action_data):
        filter_data = self._filter_data(action_data)
        req = planArmTrajectoryBezierCurveRequest()
        for key, value in filter_data.items():
            msg = jointBezierTrajectory()
            for frame in value:
                point = bezierCurveCubicPoint()
                point.end_point, point.left_control_point, point.right_control_point = frame
                msg.bezier_curve_points.append(point)
            req.multi_joint_bezier_trajectory.append(msg)
        req.start_frame_time = START_FRAME_TIME
        req.end_frame_time = filter_data[1][-1][2][0]
        print("req end_frame_time:", req.end_frame_time)
        req.joint_names = ["l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch", "l_hand_yaw", "l_hand_pitch", "l_hand_roll", "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch", "r_hand_yaw", "r_hand_pitch", "r_hand_roll"]
        return req
    
    def plan(self, action_data, timeout=40):
        req = self._create_bezier_request(action_data)
        service_name = self._srv_plan_bezier_curve_name
        rospy.wait_for_service(service_name)
        try:
            plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
            res = plan_service(req)
            if res.success:
                self._arm_joint_publisher.start_publish()   # start publish
                self._arm_traj_state.is_finished = False    # update state
                self._arm_traj_state.progress = 0
                time.sleep(0.5)
                self._flag_planing = True                   # set flag
                if self.wait_finish(timeout=timeout):  
                    self._flag_planing = False
                    self._arm_joint_publisher.stop_publish()
                    return True
                else:
                    self._flag_planing = False                # set flag
                    self._arm_joint_publisher.stop_publish()
                    return False
            else:
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def wait_finish(self, timeout=40) -> bool:
        """ 等待轨迹规划完成
        :param timeout: 超时时间，单位秒，默认40秒
        :return: bool
        """
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self._arm_traj_state.is_finished:
                rospy.loginfo(f"Trajectory execute finished, time cost: {rospy.Time.now().to_sec() - start_time:.2f}")
                return True
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.loginfo(f"Trajectory execute timeout, time cost: {rospy.Time.now().to_sec() - start_time:.2f}")
                return False
            rospy.sleep(0.1)  # 防止CPU占用过高

def main():
    rospy.init_node("bezier_curve_planner_demo_node")
    planner = BezierCurvePlanner()

    tact_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/action_files/pick1.tact")
    print(tact_path)
    action_frames = load_action_frames_file(tact_path)
    
    if not action_frames:
        return
    
    print("last q: ", last_q_from_action_frames(action_frames))

    rospy.loginfo("Planning arm trajectory...")
    planner.change_arm_ctrl_mode(2) # change arm control mode

    success = planner.plan(action_frames)
    if success:
        rospy.loginfo("Arm trajectory planned successfully")
    else:
        rospy.logerr("Failed to plan arm trajectory")
    
    rospy.signal_shutdown("Done")
if __name__ == "__main__":

    main()
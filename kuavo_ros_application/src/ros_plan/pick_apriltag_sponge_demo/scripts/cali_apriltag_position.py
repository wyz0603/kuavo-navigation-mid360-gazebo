#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import rospy
import moveit_commander
import os
import sys
import time
from scipy.spatial.transform import Rotation

from std_msgs.msg  import Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest,GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray

script_dir = os.path.dirname(os.path.abspath(__file__))
kuavo_sdk_dir = script_dir + "/../../moveit_interface_plan/scripts/"
sys.path.append(kuavo_sdk_dir)
from utils import angle_to_rad
from base import Base
from planner import Planner
from logger import Logger
from publisher import Publisher
from executor import Executor
from kuavoRobotSDK import kuavo
from joint_posestamped import JointPoseStamped
from arm_ik import ArmIk
from pydrake.all import StartMeshcat

# 读取配置文件
Base.load_config(os.path.join(script_dir, "../config/config.json"))

rospy.init_node("apriltag_pick_sponge_cali_node", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

planner = Planner()
logger = Logger()
publisher = Publisher()
executor = Executor()
joint_posestamped = JointPoseStamped()
joint_state = JointState()

#### ----- 配置项 begin
TAG_ID = 2                  # 检测识别的 AprilTag ID
MAX_TRAJECTORY_COUNT = 3    
CALI_POSTION = Point(0.44, -0.08, 0.25)
TARGET_POSITION = Point(0.436, -0.139, 0.282)
MAX_CALI_COUNT = 10
ROBOT_IS_HALF = False       # 机器人版本(True 半身版本 | False 全身版本)

# 左手固定运动轨迹点
Point_0 = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
Point_3 = angle_to_rad([-50, 50, 0, -30,  0, -20, -20])
Point_ready = angle_to_rad([-50,  20, 0, -30,  0, -20, -10])
ready_points = [Point_0, Point_1, Point_2, Point_3, Point_ready]
#### ----- 配置项 end

if ROBOT_IS_HALF:
    from dynamic_biped.msg import robotArmInfo
else:
    from dynamic_biped.msg import robotArmQVVD
Failed_count = 1        # 失败计数器
trajectory_counter = 1  # 初始化计数器
cali_count = 1
cali_data = Point(0.00, 0.00, 0.00)

def orientation_to_rpy(ori):
    quat = [ori.x, ori.y, ori.z, ori.w]
    return Rotation.from_quat(quat).as_euler('xyz', degrees=False)

def rpy_to_orientation(rpy):
    quat = Rotation.from_euler('xyz', rpy).as_quat()
    return Quaternion(quat[0], quat[1], quat[2], quat[3])

def print_ascii_text_ok():
    print("\033[92m" '''
   ____  _  __
  / __ \| |/ /
 | |  | | ' / 
 | |  | |  <  
 | |__| | . \ 
  \____/|_|\_\         
    ''' "\033[0m")
def print_ascii_text_bad():
    print("\033[91m" '''
  _               _ 
 | |__   __ _  __| |
 | '_ \ / _` |/ _` |
 | |_) | (_| | (_| |
 |_.__/ \__,_|\__,_|                     
    ''' "\033[0m")
class PickSpongeDemo(object):
    def __init__(self):
        self._move_group_name = planner.get_move_group_name()
        self._joint_name = planner.get_joint_name()
        self._kuavo_robot = kuavo("4_1_kuavo")
        self._ready_cali = False

        # ArmIK
        end_frames_name = ['torso', 'l_hand_end_virtual', 'r_hand_end_virtual']
        model_file_path = script_dir + "/../../../ros_robotModel/biped_s3/urdf/biped_s3_arm.urdf"
        meshcat = StartMeshcat()
        self._arm_ik = ArmIk(model_file_path, end_frames_name, meshcat)
        self._arm_ik.init_state(0.0, 0.0, [0] * 14)

        if self._move_group_name == "r_arm_group":
            self.ee_name = "r_hand_end_virtual"
        else:
            self.ee_name = "l_hand_end_virtual"
    def move_group_name(self):
        return self._move_group_name
    def hand_zero(self):
        zero_pose = [100, 100, 0, 0, 0, 0]
        self._kuavo_robot.set_end_control(zero_pose, zero_pose)

    def computeIK(self, curr_joint_state, posestamp:PoseStamped):
        joint_state_list = list(curr_joint_state.position)
        if self._move_group_name == "r_arm_group":
            arm_joint_state = [0.0]*7 + joint_state_list
            self._arm_ik.set_arm_joint_state(arm_joint_state)
            curr_q = self._arm_ik.curr_q()
            arm_q = self._arm_ik.computeArmIK(curr_q, None, posestamp)
            if arm_q is not None:
                return arm_q[-7:]
        else:
            arm_joint_state = joint_state_list + [0.0]*7
            self._arm_ik.set_arm_joint_state(arm_joint_state)
            curr_q = self._arm_ik.curr_q()
            arm_q = self._arm_ik.computeArmIK(curr_q, posestamp, None)
            if arm_q is not None:
                return arm_q[:7]
        return None
    
    def calculate_forward_kinematics(self, joint_state):
        fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = 'torso'    
        fk_request.fk_link_names = [self.ee_name]
        fk_request.robot_state = RobotState()
        fk_request.robot_state.joint_state = joint_state
        try:
            fk_response = fk_service(fk_request)
            if fk_response.error_code.val == fk_response.error_code.SUCCESS:
                return fk_response.pose_stamped[0]
            else:
                rospy.logerr("Forward kinematics failed with error code: %d", fk_response.error_code.val)
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return None
        
    def calculate_inverse_kinematics(self, curr_joint_state, target_pose_stamped):
        ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = planner.get_move_group_name()  # 设置运动规划组名称
        ik_request.ik_request.robot_state.joint_state.name = self._joint_name
        ik_request.ik_request.robot_state.joint_state.position = curr_joint_state.position
        ik_request.ik_request.pose_stamped = target_pose_stamped  # 设置目标末端位姿
        ik_request.ik_request.timeout = rospy.Duration(0.2)  # 设置逆解计算时间限制

        try:
            ik_response = ik_service(ik_request)
            if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                left_arm_angles_rad   = ik_response.solution.joint_state.position[0:7]    # 左手的关节角度
                right_arm_angles_rad  = ik_response.solution.joint_state.position[15:22]  # 右手的关节角度
                if planner.get_move_group_name() == "r_arm_group":
                    return right_arm_angles_rad
                else:
                    return left_arm_angles_rad
            else:
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return None

    def arm_ik(self,target_pose_stamped:PoseStamped):
        global joint_state
        ik_joint_state = self.calculate_inverse_kinematics(joint_state, target_pose_stamped)
        if ik_joint_state is None:
            rospy.logerr("Calculate Inverse kinematics failed")
            ik_joint_state = self.computeIK(joint_state, target_pose_stamped)
        return ik_joint_state

    def plan_to_target_pose(self, target_joint_state) -> bool:
        global planner,executor, logger 
        planner.set_start_state(joint_state.position)
        traj = planner.plan_to_target_joints(target_joint_state) 
        if traj:
            logger.dump_traj(traj, file_name="plan")
            executor.execute_traj(traj, wait=True)
            # 加入等待
            time.sleep(1)
            return True
        return False  

    def arm_reset(self):
        global joint_state
        
        ready_joint_state = JointState()
        ready_joint_state.header = Header()
        ready_joint_state.header.stamp = rospy.Time.now()
        ready_joint_state.name = self._joint_name
        ready_joint_state.position = Point_ready
        ee_ready_posestamped = self.calculate_forward_kinematics(ready_joint_state)
        curr_posestamped = self.calculate_forward_kinematics(joint_state)
        if curr_posestamped is not None:
            if (self._move_group_name == 'r_arm_group' and ee_ready_posestamped.pose.position.y > curr_posestamped.pose.position.y) or \
               (self._move_group_name == 'l_arm_group' and ee_ready_posestamped.pose.position.y < curr_posestamped.pose.position.y):
                self.back_to_ready()
                return

        # else
        self._kuavo_robot.set_robot_arm_ctl_mode(True)
        self.plan_to_target_pose(Point_0)

    def back_to_ready(self):
        global joint_state
        print("======================== back to ready ========================")
        curr_joint_state = joint_state.position
        planner.set_start_state(curr_joint_state)
        traj = planner.plan_to_target_joints(Point_ready)
        executor.execute_traj(traj, wait=True)
        logger.dump_traj(traj, file_name="backto_ready")

    def ready_catch(self):
        global ready_points        
        self._kuavo_robot.set_robot_arm_ctl_mode(True)
        planner.set_start_state(ready_points[0])
        ready_traj = planner.plan_to_target_joints(ready_points[1], optimize=True, add_queue=False)
        
        for index in range(1, len(ready_points)-1):
            planner.set_start_state(ready_points[index])
            traj = planner.plan_to_target_joints(ready_points[index+1], optimize=True, add_queue=False)
            for point in traj.joint_trajectory.points:
                ready_traj.joint_trajectory.points.append(point) # 合并轨迹
        
        # 从 Point_0 原点规划到 ready_point
        ready_traj = planner.retime_trajectory(ready_traj)
        logger.dump_traj(ready_traj, file_name="ready_traj")
        
        planner.push_trajectory(ready_traj)
        executor.execute_traj(ready_traj, wait=True)
        time.sleep(2.0)
    
    def go_back(self):
        global joint_state, ready_points
        curr_joint_state = joint_state.position
        planner.set_start_state(curr_joint_state)
        traj = planner.plan_to_target_joints(Point_ready)
        executor.execute_traj(traj, wait=True)
        logger.dump_traj(traj, file_name="backto_ready")
        time.sleep(1)

        back_points  = list(reversed(ready_points))
        self._kuavo_robot.set_robot_arm_ctl_mode(True)
        
        planner.set_start_state(back_points[0])
        goback_traj = planner.plan_to_target_joints(back_points[1], optimize=True, add_queue=False)
        
        for index in range(1, len(back_points)-1):
            planner.set_start_state(back_points[index])
            traj = planner.plan_to_target_joints(back_points[index+1], optimize=True, add_queue=False)
            for point in traj.joint_trajectory.points:
                goback_traj.joint_trajectory.points.append(point) # 合并轨迹
        
        # 从ready_point 规划到 Point_0 原点
        goback_traj = planner.retime_trajectory(goback_traj)
        logger.dump_traj(goback_traj, file_name="goback_traj")
        planner.push_trajectory(goback_traj)
        executor.execute_traj(goback_traj, wait=True)

        time.sleep(3.50)
        rospy.signal_shutdown("finish!")

    def move_to_cali_pose(self):
        global trajectory_counter, Failed_count
        global joint_state
        
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id ="torso"
        target_pose_stamped.pose.position.x = CALI_POSTION.x
        target_pose_stamped.pose.position.y = CALI_POSTION.y
        target_pose_stamped.pose.position.z = CALI_POSTION.z
        target_pose_stamped.pose.orientation = rpy_to_orientation([1.5707963/36.0, -1.8007963, 1.5707963/6.0])
        
        ik_joint_state = self.arm_ik(target_pose_stamped)
        if ik_joint_state is None:
            rospy.logwarn("无法规划到改位置抓取，请放置在合理的区域内！")
            self.go_back()
            return

        while trajectory_counter < MAX_TRAJECTORY_COUNT:
            # 规划执行
            if self.plan_to_target_pose(ik_joint_state):
                trajectory_counter += 1
            else:
                rospy.logerr("Failed to plan trajectory")
                Failed_count+=1

            if Failed_count > MAX_TRAJECTORY_COUNT:
                break

        if Failed_count > MAX_TRAJECTORY_COUNT:
            rospy.logwarn("规划失败次数过多")
            self.go_back()
            return
        
        rospy.logwarn("等待识别 Apriltag，请将 Apriltag 按照图示粘贴在手上，并检查表面是否平整")
        self._ready_cali = True
        return
    
    def joint_callback(self, data):
        global joint_state
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self._joint_name
        if self._move_group_name == "r_arm_group":
            joint_state.position = data.q[-7:] # 右手关节数据 rad
        else:
            joint_state.position = data.q[:7] # 左手关节数据 rad
    def detection_callback(self, msg:AprilTagDetectionArray):
        global MAX_CALI_COUNT, cali_count, cali_data

        if self._ready_cali == False:
            return

        target_detection = None
        # 遍历所有检测结果，查找 ID 为 TAG_ID 的目标
        for detection in msg.detections:
            # print("Detected valid object with ID:", detection.id[0])
            if detection.id[0] == TAG_ID:
                # rospy.loginfo("Detected object with ID: %d", detection.id[0])
                target_detection = detection
                break
        if target_detection is None:
            rospy.logwarn("未识别到 ID 为 %d 的AprilTag 海绵块，请检查apriltag是否平整，视角是否有遮掩", TAG_ID)
            return
        
        # 开始校准
        if cali_count > MAX_CALI_COUNT:
            self._ready_cali = False

            x_offset = cali_data.x / MAX_CALI_COUNT
            y_offset = cali_data.y / MAX_CALI_COUNT
            z_offset = cali_data.z / MAX_CALI_COUNT
            
            print("CALI_POSTION:\n", CALI_POSTION)
            fk_posestamped = self.calculate_forward_kinematics(joint_state)
            if fk_posestamped is not None:
                print("fk posestamped:\n", fk_posestamped.pose)
                print("=====================================================")
            posestamped = joint_posestamped.posestamped(self.ee_name)
            if posestamped is not None:
                print("ee pose_stamped:\n", posestamped.pose)
                print("=====================================================")
            
            print(" x 平均误差:", x_offset)
            print(" y 平均误差:", y_offset)
            print(" z 平均误差:", z_offset)
            print("=====================================================")

            # 确保误差在 1cm
            if x_offset > 0.01 or y_offset > 0.01 or z_offset > 0.02:
                print_ascii_text_bad()
            else:
                print_ascii_text_ok()

            _ = input("\033[93m校准结束，按下任意键返回-------------\033[0m")
            self.go_back() # 返回原点
            return
        
        cali_count += 1
        target_detection_pose=target_detection.pose.pose.pose
        cali_data.x += abs(TARGET_POSITION.x - target_detection_pose.position.x)
        cali_data.y += abs(TARGET_POSITION.y - target_detection_pose.position.y)
        cali_data.z += abs(TARGET_POSITION.z - target_detection_pose.position.z)

def l_to_r(point):
    point[1] = -point[1]
    point[2] = -point[2]
    point[4] = -point[4]
    point[6] = -point[6]
    return point

if __name__ == "__main__":    
    # 初始化固定的位置， 右手时需要调整(左手镜像)
    demo_instance = PickSpongeDemo()

    if demo_instance.move_group_name() == "r_arm_group":
       Point_1 = l_to_r(Point_1)
       Point_2 = l_to_r(Point_2)
       Point_3 = l_to_r(Point_3)
       Point_ready = l_to_r(Point_ready)
       
    logger.make_traj_dir()
    publisher.start_auto_publish()
        
    # 订阅
    if ROBOT_IS_HALF:
        joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmInfo, demo_instance.joint_callback)
    else:
        joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmQVVD, demo_instance.joint_callback)
    
    # 订阅 /robot_tag_info 话题
    apriltag_sub = rospy.Subscriber("/robot_tag_info", AprilTagDetectionArray, demo_instance.detection_callback)
    
    rate = rospy.Rate(10)     # 设置频率为 10 Hz
    while not joint_state.position and not rospy.is_shutdown():
        rate.sleep()

    demo_instance.arm_reset()  # 归位到原点
    demo_instance.hand_zero()
  
    demo_instance.ready_catch() # 准备抓取 

    demo_instance.move_to_cali_pose() # 移动到校准点

    while not rospy.is_shutdown():
        # 在循环中处理回调
        # tag = AprilTagDetection()
        # tag.id.append(TAG_ID)
        # tag.size = 0.042
        # tag.pose.header.seq = 183
        # tag.pose.pose.pose.position = Point(0.437, -0.133, 0.272)
        # tag.pose.pose.pose.orientation = Quaternion(0.0889, -0.04944, -0.762769, 0.6386189)
        # msg=AprilTagDetectionArray()
        # msg.detections.append(tag)
        # demo_instance.detection_callback(msg)
        rate.sleep()

    # 结束自动发布
    publisher.stop_auto_publish()
    exit(0)
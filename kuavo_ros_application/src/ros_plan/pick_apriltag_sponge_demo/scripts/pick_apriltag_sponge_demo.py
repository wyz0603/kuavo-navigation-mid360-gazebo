#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
    检测 AprilTag 海绵块的位姿 pose，从当前手臂位置规划到 pose

    首先通过固定轨迹点去到待抓取的位置，接下来视觉感知识别实时抓取
    （一次规划的试规划为3次，找到方差最小的轨迹进行发布），然后抓取，抓取完之后把手收回去
"""
import sys
import rospy
import moveit_commander
import os
import time
from enum import Enum

from std_msgs.msg  import Header, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest,GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from visualization_msgs.msg import Marker

script_dir = os.path.dirname(os.path.abspath(__file__))
kuavo_sdk_dir = script_dir + "/../../moveit_interface_plan/scripts/"
sys.path.append(kuavo_sdk_dir)
from base import Base
from utils import angle_to_rad, rad_to_angle, load_traj, rpy_to_orientation
from planner import Planner
from logger import Logger
from publisher import Publisher
from executor import Executor
from kuavoRobotSDK import kuavo
from joint_posestamped import JointPoseStamped
from arm_ik import ArmIk
from pydrake.all import StartMeshcat

rospy.init_node("apriltag_pick_sponge_demo_node", anonymous=True)
Base.load_config(os.path.join(script_dir, "../config/config.json")) # 读取配置文件
moveit_commander.roscpp_initialize(sys.argv)

planner = Planner()
logger = Logger()
publisher = Publisher()
executor = Executor()
joint_posestamped = JointPoseStamped()
joint_state = JointState()

####   ----- 配置项 begin
TAG_ID = 2                  # 检测识别的 AprilTag ID
X_TO_MOVEIT_OFFSET = -0.010 # X轴偏移量 -1.0cm  抓矿泉水瓶 0.000
Y_TO_MOVEIT_OFFSET = 0.005  # Y轴偏移量 +0.05cm 抓矿泉水瓶 0.000
Z_TO_MOVEIT_OFFSET = 0.013  # Z轴偏移量 +1.3cm  抓矿泉水瓶 -0.055 下降5.5cm左右
TEST_FLAG=False              # Mock: 规划到给定的位置
TEST_POSTION = Point(0.40, -0.08, 0.15) # 给定的位置
ROBOT_IS_HALF = False       # 机器人版本(True 半身版本 | False 全身版本)
USED_TRAJ_FILE = True       # 是否使用轨迹文件

# 左手固定运动轨迹点 （可镜像成右手）
Point_0 = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
Point_3 = angle_to_rad([-15, 90, 0, -50, 45, -30, 0])
Point_4 = angle_to_rad([-50, 50, 0, -30,  0, -20, -20])
Point_ready = angle_to_rad([-50,  20, 0, -30,  0, -20, -10])
ready_points = [Point_0, Point_1, Point_4, Point_ready]

# 末端执器抓取的姿态
r_sloping_rpy = [-1.78308487, -1.03491955,  1.99132322]  # 斜着抓取
r_sloping1_rpy = [-1.43798843, -0.94898864,  1.71157395] # Quaternion(0.13097846817384481, 0.670168928338911, -0.3095140628070015, -0.6617547077948237)
r_sloping2_rpy = [-1.45556747, -0.72811687, 1.86692764]  # Quaternion(-0.156287, -0.6561, 0.4210, 0.6064)
r_sloping3_rpy = [-1.36301882, -0.58394214,  1.79082344] # Quaternion(0.20527206931054925, 0.6100630959779163, -0.46960326069990077, -0.604284017682766)
r_pose2_rpy = [1.5707963/18.0, -1.7007963, 1.5707963/6.0] # 正着垂直抓取

l_sloping_rpy = [1.81113426, -1.19459714, -2.0690475]

r_catch_poses = [r_sloping_rpy] # 右手抓取姿态
l_catch_poses = [l_sloping_rpy] # 左手抓取姿态 # TODO 测试左手

####   ----- 配置项 end

if ROBOT_IS_HALF:
    from dynamic_biped.msg import robotArmInfo
else:
    from dynamic_biped.msg import robotArmQVVD
r_ee_position_range = {
    "x" : [0.36, 0.48],
    "y" : [-0.20, -0.06],
    "z" : [0.09, 0.34],
}
r_ee_best_position = Point(0.4, -0.10, 0.18)

l_ee_position_range = { # TODO 测试左手
    "x" : [0.36, 0.48],
    "y" : [0.20, 0.40],
    "z" : [0.09, 0.34],
}
l_ee_best_position = Point(0.4, 0.28, 0.18) # TODO 测试左手

def check_vaild_joint_rad(joint_state:list):
    joint_angle = rad_to_angle(joint_state)
    return all(abs(x) < 110 for x in joint_angle)

class PickSpongeDemo(object):
    class DexterousHandPose(Enum):
        Zero = 0
        Ready = 1,
        Catch = 2,
        Ok = 3,
        Fist = 4

    def __init__(self):
        self._move_group_name = planner.get_move_group_name()
        self._joint_name = planner.get_joint_name()
        self._kuavo_robot = kuavo("4_1_kuavo")
        self._ready_ok = False      # 已就绪
        self._has_apriltag = False  # 已识别到 apriltag
        self._apriltag = AprilTagDetection()

        # ArmIK
        end_frames_name = ['torso', 'l_hand_end_virtual', 'r_hand_end_virtual']
        model_file_path = script_dir + "/../../../ros_robotModel/biped_s3/urdf/biped_s3_arm.urdf"
        meshcat = StartMeshcat()
        self._arm_ik = ArmIk(model_file_path, end_frames_name, meshcat)
        self._arm_ik.init_state(0.0, 0.0, [0] * 14)

        if self._move_group_name == "r_arm_group":
            self.ee_name = "r_hand_end_virtual"
            self.ee_catch_poses = r_catch_poses
            self.ee_position_range = r_ee_position_range
            self.ee_best_position = r_ee_best_position
            self.ready_traj_path = os.path.join(os.path.dirname(__file__), "../config/r_ready_traj.json")
            self.goback_traj_path = os.path.join(os.path.dirname(__file__), "../config/r_goback_traj.json")
        else:
            self.ee_name = "l_hand_end_virtual"
            self.ee_catch_poses = l_catch_poses
            self.ee_position_range = l_ee_position_range
            self.ee_best_position = l_ee_best_position
            self.ready_traj_path = os.path.join(os.path.dirname(__file__), "../config/l_ready_traj.json")
            self.goback_traj_path = os.path.join(os.path.dirname(__file__), "../config/l_goback_traj.json")
        # test    
        if TEST_FLAG == True:
            self._has_apriltag = True
            self._apriltag.pose.pose.pose.position = TEST_POSTION
            self._visualization_marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)   
    def move_group_name(self):
        return self._move_group_name
    def control_hand(self, pose:DexterousHandPose):
        zero_pose = [0, 0, 0, 0, 0, 0]
        hand_pose = [0, 0, 0, 0, 0, 0]
        if pose == PickSpongeDemo.DexterousHandPose.Zero:
            hand_pose = [0, 0, 0, 0, 0, 0]       # Zero
        elif pose == PickSpongeDemo.DexterousHandPose.Ready:
            hand_pose = [0, 100, 0, 0, 0, 0]     # Open
        elif pose == PickSpongeDemo.DexterousHandPose.Fist:
            hand_pose = [65, 65, 90, 80, 80, 90] # Fist     
        elif pose == PickSpongeDemo.DexterousHandPose.Ok:
            hand_pose = [65, 65, 60, 0, 0, 0]    # OK
        elif pose == PickSpongeDemo.DexterousHandPose.Catch:
            hand_pose = [100, 50, 65, 70, 70, 70] # 抓矿泉水瓶 => [100, 100, 80, 75, 75, 75]

        if self._move_group_name == "r_arm_group":  
            self._kuavo_robot.set_end_control(hand_pose, hand_pose)
        else:
            self._kuavo_robot.set_end_control(hand_pose, zero_pose)

    def computeIK(self, curr_joint_state, posestamp:PoseStamped):
        joint_state_list = list(curr_joint_state.position)
        ik_joint_state = None
        if self._move_group_name == "r_arm_group":
            arm_joint_state = [0.0]*7 + joint_state_list
            self._arm_ik.set_arm_joint_state(arm_joint_state)
            curr_q = self._arm_ik.curr_q()
            arm_q = self._arm_ik.computeArmIK(curr_q, None, posestamp)
            if arm_q is not None:
                ik_joint_state = arm_q[-7:]
        else:
            arm_joint_state = joint_state_list + [0.0]*7
            self._arm_ik.set_arm_joint_state(arm_joint_state)
            curr_q = self._arm_ik.curr_q()
            arm_q = self._arm_ik.computeArmIK(curr_q, posestamp, None)
            if arm_q is not None:
                ik_joint_state =  arm_q[:7]
        
        # check vaild joint
        if ik_joint_state is not None and check_vaild_joint_rad(ik_joint_state):
            return ik_joint_state
        
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
        ik_request.ik_request.group_name = self._move_group_name  # 设置运动规划组名称
        ik_request.ik_request.robot_state.joint_state.name = self._joint_name
        ik_request.ik_request.robot_state.joint_state.position = curr_joint_state.position
        ik_request.ik_request.pose_stamped = target_pose_stamped  # 设置目标末端位姿
        ik_request.ik_request.timeout = rospy.Duration(0.2)  # 设置逆解计算时间限制

        try:
            ik_response = ik_service(ik_request)
            if ik_response.error_code.val == ik_response.error_code.SUCCESS:
                left_arm_angles_rad   = ik_response.solution.joint_state.position[0:7]    # 左手的关节角度
                right_arm_angles_rad  = ik_response.solution.joint_state.position[15:22]  # 右手的关节角度
                if self._move_group_name == "r_arm_group":
                    if check_vaild_joint_rad(right_arm_angles_rad):
                        return right_arm_angles_rad
                else:
                    if check_vaild_joint_rad(left_arm_angles_rad):
                        return left_arm_angles_rad
            else:
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return None

    def arm_ik(self,target_pose_stamped:PoseStamped):
        global joint_state
        time.sleep(0.2)
        ik_joint_pos = self.calculate_inverse_kinematics(joint_state, target_pose_stamped)
        if ik_joint_pos is None:
            rospy.logerr("Calculate Inverse kinematics failed")
            ik_joint_pos = self.computeIK(joint_state, target_pose_stamped)
        return ik_joint_pos

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
            # 防止打到桌子
            if (self._move_group_name == 'r_arm_group' and ee_ready_posestamped.pose.position.y < curr_posestamped.pose.position.y \
                and abs(curr_posestamped.pose.position.z - ee_ready_posestamped.pose.position.z) < 0.10 \
                and abs(curr_posestamped.pose.position.x - ee_ready_posestamped.pose.position.x) < 0.10 ) or \
               (self._move_group_name == 'l_arm_group' and ee_ready_posestamped.pose.position.y > curr_posestamped.pose.position.y \
                and abs(curr_posestamped.pose.position.z - ee_ready_posestamped.pose.position.z) < 0.10 \
                and abs(curr_posestamped.pose.position.x - ee_ready_posestamped.pose.position.x) < 0.10 ):
                self.back_to_ready(joint_state.positions)
                self.back_to_zero()
                return

        # else
        self._kuavo_robot.set_robot_arm_ctl_mode(True)
        planner.set_start_state(joint_state.position)
        traj = planner.plan_to_target_joints(Point_0) 
        if traj:
            logger.dump_traj(traj, file_name="arm_reset")
            executor.execute_traj(traj, wait=False)
            time.sleep(2.5)

    def back_to_ready(self, curr_joint_position):
        print("======================== back to ready ========================")
        planner.set_start_state(curr_joint_position)
        traj = planner.plan_to_target_joints(Point_ready)
        executor.execute_traj(traj, wait=False)
        logger.dump_traj(traj, file_name="backto_ready")
        time.sleep(1)

    def back_to_zero(self):
        global ready_points
        back_points  = list(reversed(ready_points))
        self._kuavo_robot.set_robot_arm_ctl_mode(True)
        goback_traj = None
        if USED_TRAJ_FILE:
            goback_traj = load_traj(self.goback_traj_path)
        else:
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
        executor.execute_traj(goback_traj, wait=False)
        time.sleep(5.5)

    def raise_hand(self, curr_joint_position):
        global planner, executor, logger 

        curr_joint_state = JointState()
        curr_joint_state.header = Header()
        curr_joint_state.header.stamp = rospy.Time.now()
        curr_joint_state.name = self._joint_name
        curr_joint_state.position = curr_joint_position

        # 末端当前位置
        current_pose = self.calculate_forward_kinematics(curr_joint_state)
        if current_pose is None:
            return
        current_pose.pose.position.y -= 0.02    
        current_pose.pose.position.z += 0.03
        current_pose.pose.orientation = rpy_to_orientation([1.5707963/18.0, -1.7007963, 1.5707963/6.0])
        start_joint_pos = self.arm_ik(current_pose)
        
        # 末端手抬高
        current_pose.pose.position.y -= 0.03 
        current_pose.pose.position.z += 0.04 # 抓矿泉水瓶时需要抬得更高些 0.06
        target_joint_pos = self.arm_ik(current_pose)
        
        if target_joint_pos is not None and start_joint_pos is not None:
            planner.set_start_state(start_joint_pos)
            traj = planner.plan_to_target_joints(target_joint_pos)
            if traj:
                logger.dump_traj(traj, file_name="raise_hand")
                executor.execute_traj(traj, wait=False)
                time.sleep(0.5)
                self.back_to_ready(target_joint_pos)
        else:
            # 逆解失败，那就不抬手
            self.back_to_ready(curr_joint_position)
            
    def catch_finish(self):
        time.sleep(0.8) # 等一会再松开
        self.control_hand(PickSpongeDemo.DexterousHandPose.Zero)
        time.sleep(1.5) # 松开手保持
        self.back_to_zero()
        rospy.signal_shutdown("catch_finish!")

    def is_ready(self):
        return  self._ready_ok

    def ready_catch(self):
        self._ready_ok = True

    def has_apriltag(self):
        return self._has_apriltag
    def check_apriltag_valid(self) -> bool:        
        x = self._apriltag.pose.pose.pose.position.x
        y = self._apriltag.pose.pose.pose.position.y
        z = self._apriltag.pose.pose.pose.position.z

        ready_joint_state = JointState()
        ready_joint_state.header = Header()
        ready_joint_state.header.stamp = rospy.Time.now()
        ready_joint_state.name = self._joint_name
        ready_joint_state.position = Point_ready

        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id ="torso"
        target_pose_stamped.pose.position.x = (x + X_TO_MOVEIT_OFFSET)
        target_pose_stamped.pose.position.y = (y + Y_TO_MOVEIT_OFFSET)
        target_pose_stamped.pose.position.z = (z + Z_TO_MOVEIT_OFFSET)

        ik_joint_state = None
        for rpy in self.ee_catch_poses:
            target_pose_stamped.pose.orientation = rpy_to_orientation(rpy)
            ik_joint_state = self.calculate_inverse_kinematics(ready_joint_state, target_pose_stamped)
            if ik_joint_state is not None:
                # print("check_apriltag_valid calculate_inverse_kinematics: ", ik_joint_state)
                break
            else:
                ik_joint_state = self.computeIK(ready_joint_state, target_pose_stamped)
                if ik_joint_state is not None:
                    # print("check_apriltag_valid calculate_inverse_kinematics: ", ik_joint_state)
                    break
        
        if ik_joint_state is None:
            # print("check_apriltag_valid ik fail!")
            return False

        pos_range = self.ee_position_range
        if (x < pos_range['x'][0] or x > pos_range['x'][1]) or \
           (y < pos_range['y'][0] or y > pos_range['y'][1]) or \
           (z < pos_range['z'][0] or z > pos_range['z'][1]):
            return False
        
        return True
    def detection_callback(self, msg:AprilTagDetectionArray):
        """
        AprilTagDetectionArray:
        id: [2]
            size: [0.04]
            pose: 
            header: 
                seq: 183
                stamp: 
                secs: 1722596831
                nsecs: 116998672
                frame_id: "torso"
            pose: 
                pose: 
                position: 
                    x: 0.5977108687518586
                    y: 0.30289356735460526
                    z: 0.047286603804560845
                orientation: 
                    x: -0.01726684419236475
                    y: 0.030777435391906402
                    z: 0.9743550086375224
                    w: -0.22223162633010823
        """  
        global joint_state
        global Y_TO_MOVEIT_OFFSET, X_TO_MOVEIT_OFFSET, Z_TO_MOVEIT_OFFSET
        
        if self.is_ready(): # 已就绪，忽视海绵块被移动的情况
            return
        
        # 安全检查
        if not msg.detections:
            # rospy.logwarn("未识别到 AprilTag 海绵块，请摆放在合理的位置并让表面平整")
            return
        
        # 初始化一个变量来存储检测到的目标
        target_detection = None

        # # 遍历所有检测结果，查找 ID 为 TAG_ID 的目标
        for detection in msg.detections:
            # print("Detected valid object with ID:", detection.id[0])
            if detection.id[0] == TAG_ID:
                # rospy.loginfo("Detected object with ID: %d", detection.id[0])
                target_detection = detection
                self._apriltag = detection
                self._has_apriltag = True
                break

        if target_detection is None:
            rospy.logwarn("未识别到 ID 为 %d 的AprilTag 海绵块，请摆放在合理的位置", TAG_ID)
            return
    def catch_sponge(self):
        global joint_state
        global planner, executor, logger 
        global Y_TO_MOVEIT_OFFSET, X_TO_MOVEIT_OFFSET, Z_TO_MOVEIT_OFFSET
        
        # 提取目标检测信息
        target_detection_pose = self._apriltag.pose.pose.pose
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id ="torso"
        target_pose_stamped.pose.position.x = (target_detection_pose.position.x + X_TO_MOVEIT_OFFSET)
        target_pose_stamped.pose.position.y = (target_detection_pose.position.y + Y_TO_MOVEIT_OFFSET)
        target_pose_stamped.pose.position.z = (target_detection_pose.position.z + Z_TO_MOVEIT_OFFSET)

        # 逆解目标关节
        ik_joint_pos = None
        for rpy in self.ee_catch_poses:
            target_pose_stamped.pose.orientation = rpy_to_orientation(rpy)
            ik_joint_pos = self.arm_ik(target_pose_stamped)
            if ik_joint_pos is not None:
                print("ee pose rpy:", rpy)
                break

        if ik_joint_pos is None:
            return False

        # 从 Point_ready 规划到目标点
        planner.set_start_state(Point_ready)
        catch_traj = planner.plan_to_target_joints(ik_joint_pos, optimize=True, add_queue=False)
        if catch_traj is None:
            return False

        # 从 Point_0 到 Point_ready 的轨迹
        ready_traj = None
        if USED_TRAJ_FILE:
            ready_traj = load_traj(self.ready_traj_path)
        else:
            planner.set_start_state(ready_points[0])
            ready_traj = planner.plan_to_target_joints(ready_points[1], optimize=True, add_queue=False)
            
            for index in range(1, len(ready_points)-1):
                planner.set_start_state(ready_points[index])
                traj = planner.plan_to_target_joints(ready_points[index+1], optimize=True, add_queue=False)
                for point in traj.joint_trajectory.points:
                    ready_traj.joint_trajectory.points.append(point) # 合并轨迹
            # 保存轨迹
            logger.dump_traj(ready_traj, file_name="ready_traj")

        # 合并轨迹
        for point in catch_traj.joint_trajectory.points:
            ready_traj.joint_trajectory.points.append(point)

        # 优化并发布轨迹
        traj = planner.retime_trajectory(ready_traj)
        planner.push_trajectory(traj)
        executor.execute_traj(traj, wait=False)
        # 等待到达目标点
        time_total = len(traj.joint_trajectory.points) * (1/ Base._PUBLISH_RATE)
        rospy.logwarn("海绵块抓取轨迹执行预计耗时:%.2f s", time_total)
        time.sleep(time_total)

        # 到达目标点，张开并抓取
        demo_instance.control_hand(PickSpongeDemo.DexterousHandPose.Ready)  # 张开
        time.sleep(0.8)
        demo_instance.control_hand(PickSpongeDemo.DexterousHandPose.Catch) # 抓取
        time.sleep(0.5)

        # --- finish ----
        demo_instance.raise_hand(ik_joint_pos)  # 抓完抬手
        demo_instance.catch_finish() # 抓取完成, 返回原位

        return True

    def joint_callback(self, data):
        global joint_state
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self._joint_name
        if self._move_group_name == "r_arm_group":
            joint_state.position = data.q[-7:] # 右手关节数据 rad
        else:
            joint_state.position = data.q[:7] # 左手关节数据 rad
    def fake_detection_apriltag(self):
        marker = Marker()
        marker.header = Header(frame_id="torso", stamp=rospy.Time.now())
        marker.ns = "cube"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = TEST_POSTION
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.052, 0.052, 0.052) 
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        self._visualization_marker_pub.publish(marker)

        # fake april tag
        tag = AprilTagDetection()
        tag.id.append(TAG_ID)
        tag.size = 0.042
        tag.pose.header.seq = 183
        tag.pose.pose.pose.position = TEST_POSTION
        tag.pose.pose.pose.orientation = Quaternion(0.0889, -0.04944, -0.762769, 0.6386189)
        msg=AprilTagDetectionArray()
        msg.detections.append(tag)
        self.detection_callback(msg)
    def tips_to_move(self):
        y_move_direction = 0
        x_move_direction = 0

        if self._apriltag.pose.pose.pose.position.y > self.ee_best_position.y:
            y_move_direction = -1 # 往小的方向移动
        else:
            y_move_direction = 1 # 往大的方向移动
        if self._apriltag.pose.pose.pose.position.x > self.ee_best_position.x:
            x_move_direction = -1 # 往小的方向移动
        else:
            x_move_direction = 1 # 往大的方向移动

        if x_move_direction > 0 and y_move_direction > 0:
            print("请跟随箭头提示移动海绵海绵块：↘")
        elif x_move_direction > 0 and y_move_direction < 0:
            print("请跟随箭头方向，在桌面上移动海绵海绵块：↙")
        elif x_move_direction < 0 and y_move_direction > 0:
            print("请跟随箭头方向，在桌面上移动海绵海绵块：↗")
        elif x_move_direction < 0 and y_move_direction < 0:
            print("请跟随箭头方向，在桌面上移动海绵海绵块：↖")

        if self._apriltag.pose.pose.pose.position.z < self.ee_best_position.z- 0.03: 
            print("请垫高海绵块一点高度：↑")
        elif self._apriltag.pose.pose.pose.position.z > self.ee_best_position.z + 0.03: 
            print("请降低海绵块一点高度：↓")
def l_to_r(point):
    point[1] = -point[1]
    point[2] = -point[2]
    point[4] = -point[4]
    point[6] = -point[6]
    return point

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

if __name__ == "__main__":    
    # 初始化固定的位置， 右手时需要调整(左手镜像)
    demo_instance = PickSpongeDemo()

    if demo_instance.move_group_name() == "r_arm_group":
       Point_1 = l_to_r(Point_1)
       Point_2 = l_to_r(Point_2)
       Point_3 = l_to_r(Point_3)
       Point_4 = l_to_r(Point_4)
       Point_ready = l_to_r(Point_ready)
       
    logger.make_traj_dir()
    publisher.start_auto_publish()
        
    # 订阅
    if ROBOT_IS_HALF:
        joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmInfo, demo_instance.joint_callback)
    else:
        joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmQVVD, demo_instance.joint_callback)
    if TEST_FLAG == False:
        # 订阅 /robot_tag_info 话题
        apriltag_sub = rospy.Subscriber("/robot_tag_info", AprilTagDetectionArray, demo_instance.detection_callback)
    
    # 等待 kuavo 节点发布 joint_state 
    print("\033[93m ******** 等待 Kuavo ROS 节点启动 ******** \033[0m")
    rate = rospy.Rate(Base._PUBLISH_RATE)
    while not joint_state.position and not rospy.is_shutdown():
        rate.sleep()

    # 手臂归位到原点
    demo_instance.arm_reset()  
    demo_instance.control_hand(PickSpongeDemo.DexterousHandPose.Zero)
    
    # 等待 ar_controller 节点发布 apriltag 信息
    print("\033[93m \n******** 请将海绵块放置到标定的合理区间内，且不要移动，参考范围如下:********")
    print("  x:", demo_instance.ee_position_range['x'])
    print("  y:", demo_instance.ee_position_range['y'])
    print("  z:", demo_instance.ee_position_range['z'])
    print("当海绵块摆放在合理区间时，会摆出 OK 的手势，并输出 `OK`")
    print("如果无法到达该点进行抓取则会握紧拳头，并输出 `BAD`")
    print("\033[0m")

    demo_instance.control_hand(PickSpongeDemo.DexterousHandPose.Fist)
    print_bad_flag = False
    check_valid_rate = rospy.Rate(1) # 1 Hz 1s
    while not rospy.is_shutdown():
        if demo_instance.check_apriltag_valid():
            print("apriltag position:\n", demo_instance._apriltag.pose.pose.pose.position)
            print("\033[92m******************************************************")
            print("*               海绵块在合理区域内")
            print("******************************************************\033[0m")
            demo_instance.control_hand(PickSpongeDemo.DexterousHandPose.Ok)
            print_ascii_text_ok()
            _ = input("\033[92m按下Enter键 开始进行 AprilTag 抓取规划，请勿再移动海绵块 \033[0m")
            break

        demo_instance.control_hand(PickSpongeDemo.DexterousHandPose.Fist)    
        
        if demo_instance.has_apriltag():
            if print_bad_flag == False:
                print_ascii_text_bad()
                print_bad_flag  = True
            else:
                demo_instance.tips_to_move()
        else:
           print(f"未识别到 ID为: {TAG_ID} 的海绵块")

        check_valid_rate.sleep()

    if rospy.is_shutdown():
        publisher.stop_auto_publish()
        exit(0)
    
    start_time = time.time()

    # 就绪
    demo_instance.control_hand(PickSpongeDemo.DexterousHandPose.Zero)
    demo_instance.ready_catch()

    # 抓取
    if demo_instance.catch_sponge() == False:
        rospy.signal_shutdown("catch_fail!")
        exit(0)

    while not rospy.is_shutdown():
        if TEST_FLAG == True:
            demo_instance.fake_detection_apriltag()
        rate.sleep()
    
    end_time = time.time()
    rospy.logwarn("-------------------- 程序运行时间:%f s",  end_time - start_time)

    # 结束自动发布
    publisher.stop_auto_publish()
    exit(0)
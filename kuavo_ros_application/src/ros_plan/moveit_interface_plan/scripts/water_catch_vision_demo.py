#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
    目标检测的结果
    当作pose
    直接pose调取规划

    首先通过固定轨迹点去到待抓取的位置，接下来视觉感知识别实时抓取，视觉规划epoch2次
    （一次规划的试规划为3次，找到方差最小的轨迹进行发布），然后抓取，抓取完之后把手收回去
"""
import sys
import rospy
import moveit_commander
import math
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import JointState
from planner import Planner
from logger import Logger
from publisher import Publisher
from executor import Executor
import time
from utils import angle_to_rad
from std_msgs.msg  import Header
from kuavoRobotSDK import kuavo

Point_zero = angle_to_rad([0, 0, 0, 0, 0, 0, 0])
Point_1 = angle_to_rad([ 20, 50, 0,   0, 10,   0, 0])
Point_2 = angle_to_rad([ 30, 90, 0, -50, 90, -30, 0])
Point_3 = angle_to_rad([-15, 90, 0, -50, 45, -40, 0])
Point_4 = angle_to_rad([-50, 50, 0, -30,  0, -50, 0])
Point_5 = angle_to_rad([-50,  0, 0, -30,  0, -50, 0])

rospy.init_node("water_catch_vision_demo_test_node", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

planner = Planner()
logger = Logger()
publisher = Publisher()
executor = Executor()

joint_state = JointState()

# 机器人版本(True 半身版本 | False 全身版本)
ROBOT_IS_HALF = False
if ROBOT_IS_HALF:
    from dynamic_biped.msg import robotArmInfo
else:
    from dynamic_biped.msg import robotArmQVVD

# 初始化标志变量
IF_NEW_FLAG = True
# 是否为第一次规划
FIRST_TRAJECTORY_FLAG = True
# 轨迹失败标志
Failed_TRAJ_FLAG = False
# 失败计数器
Failed_count = 1          
# 初始化计数器
trajectory_counter = 1
# 最大轨迹次数 2 规划一次 / 3 规划2次 / 4 规划3次
MAX_TRAJECTORY_COUNT = 3
# 初始化机器人
robot_instance = kuavo("4_1_kuavo")
# Y轴偏移量
Y_TO_MOVEIT_OFFSET = 0

# 定义灵巧手抓取函数
def end_control_to_chosse(kuavo_robot, chosse_flag):
    zero_pose = [0, 0, 0, 0, 0, 0]

    catch_left_pose = [65, 65, 90, 90, 90, 90]
    catch_right_pose = [65, 65, 90, 90, 90, 90]

    open_left_pose = [100, 0, 0, 0, 0, 0]
    open_right_pose = [100, 0, 0, 0, 0, 0]

    if chosse_flag == 0:
        kuavo_robot.set_end_control(zero_pose, zero_pose)
    elif chosse_flag == 1:
        kuavo_robot.set_end_control(catch_left_pose, catch_right_pose)
    elif chosse_flag == 2:
        kuavo_robot.set_end_control(open_left_pose, open_right_pose)

# 获取目标姿态的四元数
def robot_euler_from_quaternion(orientation_quaternion):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_quaternion)

    roll_degrees = np.degrees(roll)
    pitch_degrees = np.degrees(pitch)
    yaw_degrees = np.degrees(yaw)

    print("Roll (degrees):", roll_degrees, "Pitch (degrees):", pitch_degrees, "Yaw (degrees):", yaw_degrees)

    return roll_degrees, pitch_degrees, yaw_degrees

# 调用逆解服务
def calculate_inverse_kinematics(target_pose_stamped):
    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = 'l_arm_group'  # 设置运动规划组名称
    ik_request.ik_request.pose_stamped = target_pose_stamped  # 设置目标末端位姿
    ik_request.ik_request.timeout = rospy.Duration(0.2)  # 设置逆解计算时间限制

    try:
        ik_response = ik_service(ik_request)
        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            return ik_response.solution.joint_state
        else:
            rospy.logerr("Inverse kinematics failed with error code: %d", ik_response.error_code.val)
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))
        return None

# 显示末端逆解的结果
def display_inverse_kinematics_result(target_pose_stamped):
    # 逆解出关节角度
    joint_angles = calculate_inverse_kinematics(target_pose_stamped)
    if joint_angles is None:
        rospy.logerr("Failed to calculate inverse kinematics")
        return

    left_arm_angles_rad   = joint_angles.position[2:9]  # 左手的关节角度
    right_arm_angles_rad  = joint_angles.position[19:26]  # 右手的关节角度

    left_arm_angles_deg = [math.degrees(angle) for angle in left_arm_angles_rad]
    right_arm_angles_deg = [math.degrees(angle) for angle in right_arm_angles_rad]

    print("left_arm_angles_deg : ", left_arm_angles_deg)
    print("right_arm_angles_deg : ", right_arm_angles_deg)

def detection_callback(msg):
    global IF_NEW_FLAG, trajectory_counter
    global FIRST_TRAJECTORY_FLAG, Failed_count
    global joint_state
    global robot_instance
    global Y_TO_MOVEIT_OFFSET
    global planner, logger, publisher, executor 

    # 安全检查
    if not msg.detections:
        rospy.logwarn("No detections in message.")
        return
    
    # 初始化一个变量来存储检测到的目标
    target_detection = None

    # 遍历所有检测结果，查找 ID 为 39 的目标
    for detection in msg.detections:
        if detection.results[0].id == 39:
            rospy.loginfo("Detected valid object with ID 39")
            target_detection = detection
            break

    # 如果没有找到 ID 为 39 的目标，则返回
    if target_detection is None:
        return 

    # 提取目标检测信息
    x = target_detection.results[0].pose.pose.position.x
    y = target_detection.results[0].pose.pose.position.y
    z = target_detection.results[0].pose.pose.position.z 

    # 转换为pose_stamped
    target_pose_stamped = PoseStamped()
    target_pose_stamped.header.frame_id = "torso"
    target_pose_stamped.pose.position.x = x
    target_pose_stamped.pose.position.y = (y + Y_TO_MOVEIT_OFFSET)
    target_pose_stamped.pose.position.z = z

    # 修改成斜着抓(老方法)
    # [ x: -179.9671049, y: -75.5501686, z: -179.8963931 ]
    target_pose_stamped.pose.orientation.x = -0.0005388071066334781
    target_pose_stamped.pose.orientation.y = -0.7904212674887817
    target_pose_stamped.pose.orientation.z = 0.00032694187655405566
    target_pose_stamped.pose.orientation.w = 0.6125633213777487
    
    # 设置规划的初始点
    now_joint_state = joint_state.position
    print("=====================================================")
    print(" now_joint_state : ", now_joint_state)
    print("=====================================================")

    # 规划
    planner.set_start_state(now_joint_state)
    traj = planner.plan_to_target_pose(target_pose_stamped)

    # 发布
    if traj:
        if not IF_NEW_FLAG:
            publisher.start_auto_publish()
            IF_NEW_FLAG = True
        print(" object traj success ! --- now is {0} traj ---".format(trajectory_counter))
        logger.dump_traj(traj, file_name="test1_moveit_point")
        trajectory_counter += 1  # 增加计数器
        Failed_count = 0         # 失败计数器清0
        # 执行 等待rviz执行结果
        executor.execute_traj(traj, wait=True)

        # 加入等待
        time.sleep(3)
    else:
        rospy.logerr("Failed to plan trajectory")
        Failed_count+=1
        if Failed_count < 2:
            publisher.stop_auto_publish()
            IF_NEW_FLAG = False

    # 计数器
    if trajectory_counter >= MAX_TRAJECTORY_COUNT:
        time.sleep(5)
        # ------------------- 抓取服务 -------------------
        # 打开虎口
        end_control_to_chosse(robot_instance, 2)
        time.sleep(2)

        # 合并爪子
        end_control_to_chosse(robot_instance, 1)
        time.sleep(2)

        # ------------------- 抓取服务 -------------------
        print("=====================================================")
        planner.set_start_state(now_joint_state)
        traj = planner.plan_to_target_joints(Point_5)
        executor.execute_traj(traj, wait=True)
        logger.dump_traj(traj, file_name="testkk")

        # 回去
        # 抓完回去
        print("=====================================================")
        planner.set_start_state(Point_5)
        traj = planner.plan_to_target_joints(Point_4)
        executor.execute_traj(traj, wait=True)
        logger.dump_traj(traj, file_name="test6")

        print("=====================================================")
        planner.set_start_state(Point_4)
        traj = planner.plan_to_target_joints(Point_3)
        executor.execute_traj(traj, wait=True)
        logger.dump_traj(traj, file_name="test7")

        print("=====================================================")
        planner.set_start_state(Point_3)
        traj = planner.plan_to_target_joints(Point_2)
        executor.execute_traj(traj, wait=True)
        logger.dump_traj(traj, file_name="test8")

        print("=====================================================")
        planner.set_start_state(Point_2)
        traj = planner.plan_to_target_joints(Point_1)
        executor.execute_traj(traj, wait=True)
        logger.dump_traj(traj, file_name="test9")

        print("=====================================================")
        planner.set_start_state(Point_1)
        traj = planner.plan_to_target_joints(Point_zero)
        executor.execute_traj(traj, wait=True)
        logger.dump_traj(traj, file_name="test10")
        # 等待执行完毕 
        time.sleep(10)

        # ------------------- 结束 -------------------------
        rospy.loginfo("Planned 3 successful trajectories, shutting down...")
        rospy.signal_shutdown("Trajectory count limit reached")

def joint_callback(data):
    # 提取左手关节角度
    global joint_state
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['l_arm_pitch', 'l_arm_roll', 'l_arm_yaw', 'l_forearm_pitch','l_hand_yaw', 'l_hand_pitch', 'l_hand_roll','r_arm_pitch', 'r_arm_roll', 'r_arm_yaw', 'r_forearm_pitch','r_hand_yaw', 'r_hand_pitch', 'r_hand_roll']
    joint_state.position = data.q[:7]

if __name__ == "__main__":
    # 等待服务启动，按下inter键即可开始视觉抓取规划
    key = input(" -----moveit规划器 + pythonAPI启动完毕，按下Enter键 开始进行视觉规划 -------------")

    # 开始进行demo抓取 
    end_control_to_chosse(robot_instance, 0)
    
    logger.make_traj_dir()
    publisher.start_auto_publish()

    print("================= 固定点 轨迹规划 =====================")
    print("=====================================================")
    planner.set_start_state(Point_zero)
    traj = planner.plan_to_target_joints(Point_1)
    logger.dump_traj(traj, file_name="test1")
    executor.execute_traj(traj, wait=True)

    print("=====================================================")
    planner.set_start_state(Point_1)
    traj = planner.plan_to_target_joints(Point_2)
    logger.dump_traj(traj, file_name="test2")
    executor.execute_traj(traj, wait=True)

    print("=====================================================")
    planner.set_start_state(Point_2)
    traj = planner.plan_to_target_joints(Point_3)
    logger.dump_traj(traj, file_name="test3")
    executor.execute_traj(traj, wait=True)

    print("=====================================================")
    planner.set_start_state(Point_3)
    traj = planner.plan_to_target_joints(Point_4)
    logger.dump_traj(traj, file_name="test4")
    executor.execute_traj(traj, wait=True)
    
    print("=====================================================")
    planner.set_start_state(Point_4)
    traj = planner.plan_to_target_joints(Point_5)
    executor.execute_traj(traj, wait=True)
    logger.dump_traj(traj, file_name="test5")

    print("=====================================================")

    # 等待固定轨迹发布完毕
    time.sleep(7)

    # 订阅
    if ROBOT_IS_HALF:
        joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmInfo, joint_callback)
    else:
        joint_sub = rospy.Subscriber('/robot_arm_q_v_tau', robotArmQVVD, joint_callback)

    # 订阅 /object_yolo_tf2_torso_result 话题
    yolov_sub = rospy.Subscriber("/object_yolo_tf2_torso_result", Detection2DArray, detection_callback)

    # 设置频率为 10 Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # 在循环中处理回调
        rate.sleep()

    # 结束自动发布
    publisher.stop_auto_publish()


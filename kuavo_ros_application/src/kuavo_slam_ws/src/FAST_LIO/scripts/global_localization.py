#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading
import time
import math

import open3d as o3d
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
import tf
import tf.transformations
from sensor_msgs import point_cloud2
import argparse
from kuavo_mapping.srv import SetInitialPose, SetInitialPoseResponse

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None
print_initial_pose = False

def parse_bool(value):
    if value.lower() in ['true', '1', 'yes', 'y']:
        return True
    elif value.lower() in ['false', '0', 'no', 'n']:
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--keep_localization', type=parse_bool, default=False)
    parser.add_argument('--odom_source', type=str, default='robot')
    return parser.parse_known_args()

def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    # 使用 point_cloud2.read_points 替代 ros_numpy.numpify
    points = np.array(list(point_cloud2.read_points(pc_msg, field_names=('x', 'y', 'z'), skip_nans=True)))
    return points


def registration_at_scale(pc_scan, pc_map, initial, scale):
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
    )

    return result_icp.transformation, result_icp.fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    # 创建 PointCloud2 消息
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
    ]
    
    # 创建点云数据
    points = []
    for i in range(len(pc)):
        point = [pc[i, 0], pc[i, 1], pc[i, 2]]
        if pc.shape[1] == 4:
            point.append(pc[i, 3])
        points.append(point)
    
    # 创建 PointCloud2 消息
    msg = point_cloud2.create_cloud(header, fields, points)
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # # 发布fov内点云
    # header = cur_odom.header
    # header.frame_id = 'map'
    # publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def thread_tf_publisher():
    global T_map_to_odom, cur_odom
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        try:
            br.sendTransform(xyz, quat, rospy.Time.now() + rospy.Duration(0.005), 'odom', 'map')
            rospy.sleep(1.0/500) 
        except rospy.exceptions.ROSException as e:
            rospy.logwarn(f"Topic closed, exiting map->odom tf publisher thread")
            break
        except Exception as e:
            rospy.logwarn(f"Unexpected error in map->odom tf publisher: {e}")
            break 


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    # rospy.loginfo('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    # rospy.loginfo('Time: {}'.format(toc - tic))
    # rospy.loginfo('')

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation
        
        # 计算RPY角度
        rotation_matrix = T_map_to_odom[:3, :3]
        
        # 从旋转矩阵提取RPY角度
        if abs(rotation_matrix[2, 0]) != 1:
            pitch = -math.asin(rotation_matrix[2, 0])
            roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            # 万向锁情况
            yaw = math.atan2(-rotation_matrix[0, 1], rotation_matrix[1, 1])
            roll = 0
            pitch = -math.pi/2 if rotation_matrix[2, 0] == 1 else math.pi/2
        
        # 转换为度数
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        global print_initial_pose
        if print_initial_pose is False:
            print("=== T_map_to_odom 变换矩阵 ===")
            print("旋转矩阵 (3x3):")
            print(rotation_matrix)
            print("RPY角度:")
            print(f"  Roll (翻滚): {roll_deg:.2f}° ({roll:.4f} rad)")
            print(f"  Pitch (俯仰): {pitch_deg:.2f}° ({pitch:.4f} rad)")
            print(f"  Yaw (偏航): {yaw_deg:.2f}° ({yaw:.4f} rad)")
            print("平移向量 (x, y, z):")
            print(f"  x: {T_map_to_odom[0, 3]:.3f} 米")
            print(f"  y: {T_map_to_odom[1, 3]:.3f} 米") 
            print(f"  z: {T_map_to_odom[2, 3]:.3f} 米")
            print("完整变换矩阵:")
            print(T_map_to_odom)
            print("================================")
            print_initial_pose = True
        return True
    else:
        T_map_to_odom = np.eye(4)
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan
    # # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    # pc_msg.header.frame_id = 'odom'
    # pc_msg.header.stamp = rospy.Time().now()
    # pub_pc_in_map.publish(pc_msg)

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])


def thread_localization():
    global T_map_to_odom
    while not rospy.is_shutdown():
        # 每隔一段时间进行全局定位
        rospy.sleep(1/FREQ_LOCALIZATION)
        global_localization(T_map_to_odom)


def handle_set_initialpose_service(req):
    """处理设置初始位姿的服务请求"""
    global initialized

    rospy.loginfo(f"[DEBUG] Received service request")
    rospy.loginfo(f"[DEBUG] Request type: {type(req)}")
    rospy.loginfo(f"[DEBUG] Request attributes: {dir(req)}")

    try:
        rospy.loginfo(f"[DEBUG] req.initial_pose: {req.initial_pose}")
        rospy.loginfo(f"[DEBUG] req.initial_pose type: {type(req.initial_pose)}")

        if hasattr(req, 'initial_pose') and req.initial_pose is not None:
            rospy.loginfo(f"[DEBUG] initial_pose attributes: {dir(req.initial_pose)}")
            if hasattr(req.initial_pose, 'pose'):
                rospy.loginfo(f"[DEBUG] req.initial_pose.pose: {req.initial_pose.pose}")
                rospy.loginfo(f"[DEBUG] req.initial_pose.pose type: {type(req.initial_pose.pose)}")
                if hasattr(req.initial_pose.pose, 'pose'):
                    rospy.loginfo(f"[DEBUG] req.initial_pose.pose.pose: {req.initial_pose.pose.pose}")
                    rospy.loginfo(f"[DEBUG] req.initial_pose.pose.pose type: {type(req.initial_pose.pose.pose)}")
                    if hasattr(req.initial_pose.pose.pose, 'position'):
                        rospy.loginfo(f"[DEBUG] position: {req.initial_pose.pose.pose.position}")
                    if hasattr(req.initial_pose.pose.pose, 'orientation'):
                        rospy.loginfo(f"[DEBUG] orientation: {req.initial_pose.pose.pose.orientation}")
        else:
            rospy.logwarn(f"[DEBUG] req.initial_pose is None or does not exist!")
            return SetInitialPoseResponse(False, "req.initial_pose is None or does not exist")

        # 将PoseWithCovarianceStamped转换为变换矩阵
        rospy.loginfo(f"[DEBUG] Calling pose_to_mat...")
        initial_pose = pose_to_mat(req.initial_pose)
        rospy.loginfo(f"[DEBUG] pose_to_mat returned: {initial_pose}")

        # allow initialization many times
        if cur_scan is not None:
            rospy.loginfo(f"[DEBUG] cur_scan is available, calling global_localization...")
            result = global_localization(initial_pose)
            if result:
                rospy.loginfo("Initialized successfully")
                initialized = True
                return SetInitialPoseResponse(True, "Initialized successfully")
            else:
                rospy.logwarn("Failed to initialize")
                return SetInitialPoseResponse(False, "Failed to initialize")
        else:
            rospy.logwarn("cur_scan is None, cannot initialize")
            return SetInitialPoseResponse(False, "cur_scan is None")

    except AttributeError as e:
        rospy.logwarn(f"[DEBUG] AttributeError: {str(e)}")
        rospy.logwarn(f"[DEBUG] Error occurred while accessing attributes")
        return SetInitialPoseResponse(False, f"AttributeError: {str(e)}")
    except Exception as e:
        rospy.logwarn(f"[DEBUG] Failed to set initial pose: {str(e)}")
        rospy.logwarn(f"[DEBUG] Exception type: {type(e)}")
        import traceback
        rospy.logwarn(f"[DEBUG] Traceback: {traceback.format_exc()}")
        return SetInitialPoseResponse(False, f"Failed to set initial pose: {str(e)}")

def handle_initialpose_callback(pose_msg):
    global initialized, cur_scan
    initial_pose = pose_to_mat(pose_msg)
    if cur_scan:
        initialized = global_localization(initial_pose)
    else:
        rospy.logwarn('First scan not received!!!!!')

if __name__ == '__main__':
    args, unknown = parse_args()
    keep_localization = args.keep_localization

    MAP_VOXEL_SIZE = 0.4
    SCAN_VOXEL_SIZE = 0.1

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 10

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.90

    # FOV(rad), modify this according to your LiDAR type
    FOV = 6.28

    # The farthest distance(meters) within FOV
    FOV_FAR = 150

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    # pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    # pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    # pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    # 强制使用 /Odometry 话题
    odom_topic = '/Odometry'
    rospy.loginfo(f'Using odometry topic: {odom_topic}')

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber(odom_topic, Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, handle_initialpose_callback, queue_size=1)

    # 添加初始位姿服务
    initialpose_service = rospy.Service('set_initialpose', SetInitialPose, handle_set_initialpose_service)

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/point_cloud_map', PointCloud2))
    tf_publisher_thread = threading.Thread(target=thread_tf_publisher)
    tf_publisher_thread.start()

    # Initialize
    while not initialized and not rospy.is_shutdown():
        # 等待初始位姿
        rospy.logwarn('Waiting for initial pose....')
        rospy.sleep(1)

    if initialized:
        rospy.loginfo('Initial pose received!!!!!!')
        rospy.loginfo('')
        rospy.loginfo('Initialize successfully!!!!!!')
        rospy.loginfo('')
    else:
        rospy.logwarn('Initial pose not received!!!!!!')

    if keep_localization:
        localization_thread = threading.Thread(target=thread_localization)
        localization_thread.start()


    while not rospy.is_shutdown():
        rospy.spin()

    if keep_localization:
        localization_thread.join()
    tf_publisher_thread.join()
    
    rospy.loginfo('Localization Node Exited...')

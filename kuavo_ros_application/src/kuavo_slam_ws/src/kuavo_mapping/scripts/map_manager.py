#!/usr/bin/env python3

import rospy
import subprocess
import os
import signal
import json
import math
import threading
import time
import re
import rosnode
from datetime import datetime
from sqlalchemy import create_engine, Column, Integer, String, Float, DateTime, UniqueConstraint
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, scoped_session
from kuavo_mapping.srv import LoadMap, LoadMapResponse
from kuavo_mapping.srv import GetCurrentMap, GetCurrentMapResponse
from kuavo_mapping.srv import GetAllMaps, GetAllMapsResponse
from kuavo_mapping.srv import TaskPointOperation, TaskPointOperationResponse
from kuavo_mapping.srv import NavigateToTaskPoint, NavigateToTaskPointResponse
from kuavo_mapping.srv import InitialPoseWithTaskPoint, InitialPoseWithTaskPointResponse
from kuavo_mapping.srv import SetInitialPose, SetInitialPoseResponse
from kuavo_mapping.msg import TaskPoint as TaskPointMsg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID, GoalStatusArray
import tf2_ros
import tf2_geometry_msgs
import argparse
import std_msgs.msg

# 创建基类
Base = declarative_base()

# 定义任务点模型
class TaskPoint(Base):
    __tablename__ = 'task_points'
    
    id = Column(Integer, primary_key=True)
    map_name = Column(String)
    name = Column(String)
    pos_x = Column(Float)
    pos_y = Column(Float)
    pos_z = Column(Float)
    orient_x = Column(Float)
    orient_y = Column(Float)
    orient_z = Column(Float)
    orient_w = Column(Float)
    created_at = Column(DateTime, default=datetime.utcnow)
    
    __table_args__ = (
        UniqueConstraint('map_name', 'name', name='uix_map_name_name'),
    )
    
    def to_task_point_msg(self):
        """转换为ROS消息类型"""
        pose = Pose()
        pose.position = Point(self.pos_x, self.pos_y, self.pos_z)
        pose.orientation = Quaternion(self.orient_x, self.orient_y, self.orient_z, self.orient_w)
        return TaskPointMsg(pose=pose, name=self.name)
    
    @classmethod
    def from_task_point_msg(cls, map_name, task_point_msg):
        """从ROS消息类型创建数据库对象"""
        return cls(
            map_name=map_name,
            name=task_point_msg.name,
            pos_x=task_point_msg.pose.position.x,
            pos_y=task_point_msg.pose.position.y,
            pos_z=task_point_msg.pose.position.z,
            orient_x=task_point_msg.pose.orientation.x,
            orient_y=task_point_msg.pose.orientation.y,
            orient_z=task_point_msg.pose.orientation.z,
            orient_w=task_point_msg.pose.orientation.w
        )

def parse_args():
    parser = argparse.ArgumentParser(description='Map Manager')
    parser.add_argument('--map', type=str, default='', help='map file path')
    return parser.parse_known_args()


class MapManager:
    def __init__(self):
        rospy.init_node('map_manager')

        self.pcd_dir = os.path.expanduser('~/maps')
        self.yaml_dir = os.path.expanduser('~/maps')
        self.current_map = ""

        # 并发保护（service 回调可能并发）
        self.map_lock = threading.Lock()
        
        # 存储进程PID
        self.pcd_process = None
        self.map_server_process = None

        # 任务点管理
        self.task_points = {}  # 使用字典存储任务点，key为name
        self.marker_pub = rospy.Publisher('task_points_markers', MarkerArray, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.move_base_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

        # 导航状态
        self.is_navigating = False
        # 订阅导航状态话题
        self.nav_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self._nav_status_callback)
        # 启动定时检测（在初始化时就启动，这样不管怎么发送导航目标都能监控）
        rospy.loginfo("Creating nav check timer on init")
        self.nav_check_timer = rospy.Timer(rospy.Duration(3.0), self._nav_check_callback)
        rospy.loginfo("Nav check timer started on init")

        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 发布服务
        self.load_map_srv = rospy.Service('load_map', LoadMap, self.handle_load_map)
        self.get_current_map_srv = rospy.Service('get_current_map', GetCurrentMap, self.handle_get_current_map)
        self.get_all_maps_srv = rospy.Service('get_all_maps', GetAllMaps, self.handle_get_all_maps)
        self.task_point_srv = rospy.Service('task_point', TaskPointOperation, self.handle_task_point)
        self.navigate_srv = rospy.Service('navigate_to_task_point', NavigateToTaskPoint, self.handle_navigate_to_task_point)
        self.initialpose_with_taskpoint_srv = rospy.Service('initialpose_with_taskpoint', InitialPoseWithTaskPoint, self.handle_initialpose_with_taskpoint)

        rospy.loginfo("MapManager services ready.")

    def init_database(self):
        """初始化SQLAlchemy数据库"""
        map_name = self.current_map if self.current_map else "temp"
        db_path = os.path.join(self.yaml_dir, f'{map_name}_task_points.db')
        
        # 如果是temp数据库且文件存在，则删除它
        if map_name == "temp" and os.path.exists(db_path):
            try:
                os.remove(db_path)
                rospy.loginfo(f"Deleted existing temp database: {db_path}")
            except Exception as e:
                rospy.logwarn(f"Failed to delete temp database: {str(e)}")
        
        rospy.loginfo(f"Database path: {db_path}")
        self.engine = create_engine(f'sqlite:///{db_path}', connect_args={'check_same_thread': False})
        Base.metadata.create_all(self.engine)
        session_factory = sessionmaker(bind=self.engine)
        self.Session = scoped_session(session_factory)

    def get_session(self):
        """获取数据库会话"""
        return self.Session()

    def get_robot_pose(self):
        """获取机器人当前位置"""
        try:
            # 从参数服务器获取机器人基座坐标系
            robot_base_frame = rospy.get_param('/move_base/global_costmap/robot_base_frame', 'base_link')
            
            # 等待最新的TF变换
            self.tf_buffer.lookup_transform('map', robot_base_frame, rospy.Time(0), rospy.Duration(1.0))
            
            # 获取机器人位姿
            transform = self.tf_buffer.lookup_transform('map', robot_base_frame, rospy.Time(0))
            pose = Pose()
            pose.position = transform.transform.translation
            pose.position.z = 0.0  # 确保z坐标为0
            
            # 从四元数中提取yaw角
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            # 从yaw角创建新的四元数（pitch和roll为0）
            pose.orientation = Quaternion(0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))
            
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get robot pose: {str(e)}")
            return None

    def generate_task_point_name(self):
        """生成任务点名称"""
        base_name = "task_point"
        index = 1
        while f"{base_name}_{index}" in self.task_points:
            index += 1
        return f"{base_name}_{index}"

    def load_task_points(self):
        """从数据库加载任务点"""
        map_name = self.current_map if self.current_map else "temp"
        
        self.task_points.clear()
        try:
            session = self.get_session()
            task_points = session.query(TaskPoint)\
                .filter(TaskPoint.map_name == map_name)\
                .order_by(TaskPoint.id)\
                .all()
            
            for tp in task_points:
                self.task_points[tp.name] = tp.to_task_point_msg()
            
            rospy.loginfo(f"Loaded {len(self.task_points)} task points from database")
        except Exception as e:
            rospy.logwarn(f"Failed to load task points: {str(e)}")
        finally:
            session.close()
        
        # ensure the task points are published
        pub_times = 5 
        for i in range(pub_times):
            self.publish_task_points()
            rospy.sleep(0.1)

    def save_task_points(self):
        """保存任务点到数据库"""
        map_name = self.current_map if self.current_map else "temp"
        
        session = self.get_session()
        try:
            # 删除当前地图的所有任务点
            session.query(TaskPoint)\
                .filter(TaskPoint.map_name == map_name)\
                .delete()
            
            # 插入新的任务点
            for point in self.task_points.values():
                task_point = TaskPoint.from_task_point_msg(map_name, point)
                session.add(task_point)
            
            session.commit()
            rospy.loginfo(f"Saved {len(self.task_points)} task points to database")
        except Exception as e:
            rospy.logwarn(f"Failed to save task points: {str(e)}")
            session.rollback()
        finally:
            session.close()

    def publish_task_points(self):
        """发布任务点标记"""
        marker_array = MarkerArray()
        
        # 首先添加删除所有标记的操作
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, (name, point) in enumerate(self.task_points.items()):
            # 文字标记
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.id = i * 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            # 创建新的Pose对象，避免修改原始数据
            text_pose = Pose()
            text_pose.position = Point(point.pose.position.x, point.pose.position.y, 0.6)  # 只在显示时调整高度
            text_pose.orientation = point.pose.orientation
            text_marker.pose = text_pose
            text_marker.scale.z = 0.5
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = name
            marker_array.markers.append(text_marker)
            
        self.marker_pub.publish(marker_array)

    def handle_task_point(self, req):
        """处理任务点操作请求"""
        response = TaskPointOperationResponse()
        
        # 定义操作类型常量
        ADD = 0
        UPDATE = 1
        DELETE = 2
        GET = 3
        
        # 使用temp作为map_name当current_map为空时
        map_name = self.current_map if self.current_map else "temp"
            
        if req.operation == ADD:
            if req.name in self.task_points:
                response.success = False
                response.message = f"Task point '{req.name}' already exists"
                return response
                
            if req.use_robot_current_pose:
                pose = self.get_robot_pose()
                if pose is None:
                    response.success = False
                    response.message = "Failed to get robot pose"
                    return response
            else:
                pose = req.task_point.pose
                # 从四元数中提取yaw角
                q = pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                # 从yaw角创建新的四元数（pitch和roll为0）
                pose.orientation = Quaternion(0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))
                
            # 确保z坐标为0
            pose.position.z = 0.0
            
            # 创建新的TaskPointMsg对象，避免修改原始数据
            task_point = TaskPointMsg()
            task_point.name = req.name
            task_point.pose = pose
            self.task_points[req.name] = task_point
            
            self.save_task_points()
            response.success = True
            response.message = f"Added task point '{req.name}'"
            
        elif req.operation == UPDATE:
            if req.name not in self.task_points:
                response.success = False
                response.message = f"Task point '{req.name}' not found"
                return response
                
            if req.use_robot_current_pose:
                pose = self.get_robot_pose()
                if pose is None:
                    response.success = False
                    response.message = "Failed to get robot pose"
                    return response
            else:
                pose = req.task_point.pose
                # 从四元数中提取yaw角
                q = pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                # 从yaw角创建新的四元数（pitch和roll为0）
                pose.orientation = Quaternion(0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))
                
            # 确保z坐标为0
            pose.position.z = 0.0
            
            # 创建新的TaskPointMsg对象，避免修改原始数据
            task_point = TaskPointMsg()
            task_point.name = req.name
            task_point.pose = pose
            self.task_points[req.name] = task_point
            
            self.save_task_points()
            response.success = True
            response.message = f"Updated task point '{req.name}'"
            
        elif req.operation == DELETE:
            if req.name not in self.task_points:
                response.success = False
                response.message = f"Task point '{req.name}' not found"
                return response
                
            del self.task_points[req.name]
            self.save_task_points()
            response.success = True
            response.message = f"Deleted task point '{req.name}'"
            
        elif req.operation == GET:
            # 创建新的任务点列表，确保z坐标为0
            task_points = []
            for point in self.task_points.values():
                new_point = TaskPointMsg()
                new_point.name = point.name
                new_point.pose = Pose()
                new_point.pose.position = Point(point.pose.position.x, point.pose.position.y, 0.0)
                new_point.pose.orientation = point.pose.orientation
                task_points.append(new_point)
            
            response.success = True
            response.message = "Got all task points"
            response.task_points = task_points
        
        self.publish_task_points()
            
        return response

    def handle_load_map(self, req):
        with self.map_lock:
            raw_name = req.map_name
            map_name = self.sanitize_map_name(raw_name)
            rospy.loginfo(f"Request to load map: raw=[{raw_name}] sanitized=[{map_name}]")

            if not map_name:
                return LoadMapResponse(False, "Empty map name after sanitize")

            pcd_path = os.path.join(self.pcd_dir, map_name + ".pcd")
            yaml_path = os.path.join(self.yaml_dir, map_name + ".yaml")

            if not os.path.isfile(pcd_path) and not os.path.isfile(yaml_path):
                msg = f"Neither PCD nor YAML files found for map '{map_name}'"
                rospy.logwarn(msg)
                return LoadMapResponse(False, msg)

            # 1) 停止旧进程（OS + ROS 双确认）
            self.kill_process(self.pcd_process, node_name="/pcd_to_pointcloud")
            self.kill_process(self.map_server_process, node_name="/map_server")

            self.pcd_process = None
            self.map_server_process = None

            # 2) 启动新进程（即使只有 yaml 也允许，只是跳过 pcd）
            try:
                self.launch_node(map_name)
            except Exception as e:
                rospy.logerr(f"Failed to launch map nodes: {e}")
                return LoadMapResponse(False, str(e))

            # 3) 更新状态
            self.current_map = map_name
            self.init_database()
            self.load_task_points()

            rospy.loginfo(f"Map '{map_name}' loaded successfully")
            return LoadMapResponse(True, f"Map '{map_name}' loaded.")


    def kill_process(self, process, node_name="", timeout=0.1):
        """
        1) 终止 OS 进程并等待退出
        2) 等 ROS graph 中以 node_name 为前缀的节点全部消失
        （例如 /map_server, /map_server_123, /map_server__xxx）
        """

        # ---------- 1. 先杀 OS 进程 ----------
        if process is not None:
            try:
                rospy.loginfo(f"Killing {node_name} (pid={process.pid})")
                process.terminate()

                try:
                    process.wait(timeout=timeout)
                    rospy.loginfo(f"{node_name} process exited")
                except subprocess.TimeoutExpired:
                    rospy.logwarn(f"{node_name} did not exit, force killing")
                    process.kill()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        pass

            except Exception as e:
                rospy.logwarn(f"Error killing {node_name}: {str(e)}")

        # ---------- 2. 等 ROS graph 注销（前缀匹配，关键修复点） ----------
        if not node_name:
            return

        def node_prefix_gone(nodes, prefix):
            """
            只要 ROS graph 里不存在：
            - /prefix
            - /prefix_*
            就认为该类节点已完全消失
            """
            for n in nodes:
                if n == prefix or n.startswith(prefix + "_"):
                    return False
            return True

        start_time = time.time()
        while True:
            try:
                nodes = rosnode.get_node_names()
            except rosnode.ROSNodeIOException:
                nodes = []

            if node_prefix_gone(nodes, node_name):
                rospy.loginfo(f"{node_name} nodes removed from ROS graph")
                return

            if time.time() - start_time > timeout:
                rospy.logwarn(
                    f"Timeout waiting for {node_name} nodes to unregister, "
                    f"remaining nodes: {[n for n in nodes if n.startswith(node_name)]}"
                )
                break

            time.sleep(0.2)

        # ---------- 3. 兜底：强制 rosnode kill 所有前缀匹配节点 ----------
        try:
            kill_list = [
                n for n in rosnode.get_node_names()
                if n == node_name or n.startswith(node_name + "_")
            ]
            if kill_list:
                rospy.logwarn(f"Force rosnode kill: {kill_list}")
                rosnode.kill_nodes(kill_list)
        except Exception as e:
            rospy.logwarn(f"rosnode kill failed for {node_name}: {e}")


    def sanitize_map_name(self, raw: str) -> str:
        """
        - 去掉空格
        - 去掉 .yaml/.pcd 扩展名
        - 把非法字符替换成 _
        - 支持中文字符 (UTF-8)
        这样可以避免 ros::InvalidNameException
        """
        name = (raw or "").strip()
        # 去掉扩展名（支持传 map_manager.yaml / map_manager.pcd）
        name = os.path.splitext(name)[0]
        # 允许英文、数字、下划线、中文 (\u4e00-\u9fff 覆盖常用汉字)
        # 同时过滤掉文件系统不支持的字符: / \\ : * ? " < > |
        name = re.sub(r"[\\/:*?\"<>|]", "_", name)
        return name


    def launch_node(self, map_file):
        # map_file 这里要求是“纯名字”，不要带 .yaml/.pcd
        map_file = self.sanitize_map_name(map_file)

        pcd_path = os.path.join(self.pcd_dir, map_file + ".pcd")
        yaml_path = os.path.join(self.yaml_dir, map_file + ".yaml")

        # 3D 点云地图（有 pcd 才启动）
        if os.path.isfile(pcd_path):
            pcd_cmd = [
                'rosrun', 'pcl_ros', 'pcd_to_pointcloud',
                pcd_path,
                '5',
                '_frame_id:=map',
                'cloud_pcd:=/point_cloud_map',
                '__name:=pcd_to_pointcloud'   # ✅ 固定节点名
            ]
            rospy.loginfo(f"Launching PCD node: {' '.join(pcd_cmd)}")
            self.pcd_process = subprocess.Popen(pcd_cmd)
        else:
            rospy.logwarn(f"PCD file not found, skip pcd_to_pointcloud: {pcd_path}")
            self.pcd_process = None

        # 2D map_server（有 yaml 才启动）
        if os.path.isfile(yaml_path):
            map_cmd = [
                'rosrun', 'map_server', 'map_server',
                yaml_path,
                '__name:=map_server'          # ✅ 固定节点名
            ]
            rospy.loginfo(f"Launching map_server: {' '.join(map_cmd)}")
            self.map_server_process = subprocess.Popen(map_cmd)
        else:
            rospy.logwarn(f"YAML file not found, skip map_server: {yaml_path}")
            self.map_server_process = None



    def handle_get_current_map(self, req):
        return GetCurrentMapResponse(self.current_map)

    def handle_get_all_maps(self, req):
        # 根据目录找所有地图名（去除扩展名）
        pcd_maps = set([f[:-4] for f in os.listdir(self.pcd_dir) if f.endswith('.pcd')])
        yaml_maps = set([f[:-5] for f in os.listdir(self.yaml_dir) if f.endswith('.yaml')])
        all_maps = sorted(list(pcd_maps.union(yaml_maps)))
        return GetAllMapsResponse(all_maps)

    def handle_navigate_to_task_point(self, req):
        """处理导航到任务点的请求"""
        response = NavigateToTaskPointResponse()
        
        if req.task_name not in self.task_points:
            response.success = False
            response.message = f"Task point '{req.task_name}' not found"
            return response
            
        # 获取任务点的位姿
        task_point = self.task_points[req.task_name]
        
        # 创建导航目标消息
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = task_point.pose
        rospy.loginfo(f"Navigating to task point '{task_point.pose}'")
        # 发布导航目标
        self.goal_pub.publish(goal)

        response.success = True
        response.message = f"Navigating to task point '{req.task_name}'"
        return response

    def handle_initialpose_with_taskpoint(self, req):
        """处理来自initialpose_with_taskpoint服务的请求，并调用global_localization的set_initialpose服务"""
        response = InitialPoseWithTaskPointResponse()
        
        # 从请求中提取任务点名称
        task_point_name = req.task_point_name
        
        # 检查任务点是否存在
        if task_point_name in self.task_points:
            # 获取任务点的位姿
            task_point = self.task_points[task_point_name]
            
            # 创建新的PoseWithCovarianceStamped消息
            initialpose_msg = PoseWithCovarianceStamped()
            initialpose_msg.header.frame_id = "map"
            initialpose_msg.header.stamp = rospy.Time.now()
            initialpose_msg.pose.pose = task_point.pose
            
            # # 设置默认的协方差矩阵
            # initialpose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
            #                                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
            #                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            #                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            #                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            #                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
            
            try:
                # 调用global_localization提供的set_initialpose服务
                rospy.wait_for_service('set_initialpose', timeout=5.0)
                set_initialpose_service = rospy.ServiceProxy('set_initialpose', SetInitialPose)
                set_response = set_initialpose_service(initialpose_msg)
                
                if set_response.success:
                    rospy.loginfo(f"Successfully set initial pose using task point '{task_point_name}'")
                    response.success = True
                    response.message = f"Successfully set initial pose using task point '{task_point_name}'"
                else:
                    rospy.logwarn(f"Failed to set initial pose: {set_response.message}")
                    response.success = False
                    response.message = f"Failed to set initial pose: {set_response.message}"
                    
            except rospy.ServiceException as e:
                rospy.logwarn(f"Service call failed: {e}")
                response.success = False
                response.message = f"Service call failed: {e}"
            except rospy.ROSException as e:
                rospy.logwarn(f"Service timeout: {e}")
                response.success = False
                response.message = f"Service timeout: {e}"
        else:
            rospy.logwarn(f"Task point '{task_point_name}' not found in task points")
            response.success = False
            response.message = f"Task point '{task_point_name}' not found"
        
        return response

    def _nav_status_callback(self, msg):
        """导航状态回调 - 根据 /move_base/status 更新 is_navigating 状态"""
        if msg.status_list:
            # 获取最新的状态
            status = msg.status_list[-1].status
            # 0=PENDING, 1=ACTIVE 表示正在导航
            self.is_navigating = (status in [0, 1])
           #rospy.loginfo(f"Navigation status: {status}, is_navigating: {self.is_navigating}")

    def _nav_check_callback(self, event):
        """导航状态检测回调 - 只在导航状态下检测节点"""
        # rospy.loginfo(f"Nav check callback called, is_navigating: {self.is_navigating}")

        # 只在导航状态下检测
        if not self.is_navigating:
            return

        # 检查 /nodelet_manager 节点是否可用
        node_available = self._check_node_available('/nodelet_manager')
       #rospy.loginfo(f"Node /nodelet_manager available: {node_available}")

        if not node_available:
            rospy.logwarn("Node /nodelet_manager not available, canceling navigation")
            # 发送取消导航消息
            cancel_msg = GoalID()
            self.move_base_cancel.publish(cancel_msg)
            # 重置导航状态
            self.is_navigating = False

    def _check_node_available(self, node_name):
        """检查节点是否可用 - 使用 ping 方式"""
        try:
            # 使用 rosnode_ping 来检测节点是否真正响应
            # max_count=1 表示只 ping 一次
            # rosnode_ping 返回的是 bool 值
            result = rosnode.rosnode_ping(node_name, max_count=1)
            return result  # result 是 True 或 False
        except Exception as e:
            rospy.logwarn(f"Error pinging node {node_name}: {e}")
            return False


if __name__ == '__main__':
    try:
        map_manager = MapManager()
        args, unknown = parse_args()
        if args.map == '':
            rospy.logwarn("No default map loaded")
            # 即使没有地图，也要初始化数据库和加载任务点
            map_manager.init_database()
            map_manager.load_task_points()
        else:
            map_manager.current_map = args.map
            map_manager.launch_node(args.map)
            map_manager.init_database()
            map_manager.load_task_points()  # 加载默认地图的任务点

        rospy.spin()
    except rospy.ROSInterruptException:
        map_manager.cleanup()
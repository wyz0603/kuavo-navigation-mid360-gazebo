#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
导航建图服务管理器
提供导航和建图模式切换服务
采用简单的方案：建图和导航完全分离，通过启动/停止launch文件来切换
"""

import rospy
import subprocess
import os
import signal
import time
import yaml
from datetime import datetime
import rospkg
import threading

from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Imu

try:
    from kuavo_mapping.srv import StartMapping, StartMappingResponse
    from kuavo_mapping.srv import SaveMap, SaveMapResponse
    from kuavo_mapping.srv import StopMapping, StopMappingResponse
    from kuavo_mapping.srv import RenameMap, RenameMapResponse
    from kuavo_mapping.srv import EditMap, EditMapResponse
    from kuavo_mapping.srv import DeleteMap, DeleteMapResponse
    from kuavo_mapping.srv import GetCurrentMap, GetCurrentMapResponse
    from kuavo_mapping.srv import TaskPointOperation, TaskPointOperationRequest
    rospy.loginfo("Successfully imported kuavo_mapping services")
except ImportError as e:
    rospy.logerr(f"Failed to import kuavo_mapping services: {e}")
    import sys
    sys.exit(1)


class NavigationServiceManager:
    def __init__(self):
        rospy.init_node('navigation_service_manager', anonymous=False)

        # 状态管理
        self.is_mapping = False
        self.current_map_name = None
        self.is_saving_map = False  # 地图保存状态标志
        self.save_map_lock = threading.Lock()  # 保存操作的锁
        self.last_save_time = 0.0  # 记录上次保存操作的时间戳

        # launch文件进程
        self.navigation_launch_process = None
        self.mapping_launch_process = None

        # 导航launch默认参数
        self.nav_params = {
            'map': rospy.get_param('~default_map', ''),
            'base_local_planner': rospy.get_param('~default_base_local_planner', 'mpc_local_planner'),
            'odom_source': rospy.get_param('~default_odom_source', 'lidar'),
            'crop_box': rospy.get_param('~default_crop_box', 'false'),
            'keep_localization': rospy.get_param('~default_keep_localization', 'false'),
            'rviz': rospy.get_param('~default_rviz', 'false')
        }

        # 获取包路径
        try:
            self.rospack = rospkg.RosPack()
            self.kuavo_mapping_path = self.rospack.get_path('kuavo_mapping')
            self.kuavo_navigation_path = self.rospack.get_path('kuavo_navigation')
        except Exception as e:
            rospy.logwarn(f"rospkg error: {e}")
            # 降级方案：使用相对路径
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.kuavo_mapping_path = os.path.join(current_dir, '..')
            self.kuavo_navigation_path = os.path.join(current_dir, '..', '..', '..', 'kuavo_navigation')

        rospy.loginfo(f"Navigation Service Manager initialized")
        rospy.loginfo(f"kuavo_mapping path: {self.kuavo_mapping_path}")
        rospy.loginfo(f"kuavo_navigation path: {self.kuavo_navigation_path}")

        # 设置服务
        self._setup_services()

        # 启动监控定时器
        self.monitor_timer = rospy.Timer(rospy.Duration(5.0), self._monitor_processes)

        # 默认启动导航模式
        rospy.loginfo("Starting default navigation mode...")
        self._start_navigation_launch()

        rospy.loginfo("Navigation Service Manager is ready")

    def _setup_services(self):
        """设置ROS服务"""
        rospy.Service('/start_mapping_service', StartMapping, self.start_mapping)
        rospy.Service('/save_map_service', SaveMap, self.save_map)
        rospy.Service('/stop_mapping_service', StopMapping, self.stop_mapping)
        rospy.Service('/rename_map_service', RenameMap, self.rename_map)
        rospy.Service('/edit_map_service', EditMap, self.edit_map)
        rospy.Service('/delete_map_service', DeleteMap, self.delete_map)
        rospy.Service('/get_mapping_status', Trigger, self.get_mapping_status)

        rospy.loginfo("Navigation services registered:")
        rospy.loginfo("  - /start_mapping_service")
        rospy.loginfo("  - /save_map_service")
        rospy.loginfo("  - /stop_mapping_service")
        rospy.loginfo("  - /rename_map_service")
        rospy.loginfo("  - /edit_map_service")
        rospy.loginfo("  - /delete_map_service")
        rospy.loginfo("  - /get_mapping_status")

    def start_mapping(self, req):
        """启动建图服务"""
        response = StartMappingResponse()

        try:
            if self.is_mapping:
                response.success = False
                response.message = f"建图已在进行中: {self.current_map_name}"
                return response

            # 获取参数
            map_name = getattr(req, 'map_name', f"map_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}")
            lidar_type = getattr(req, 'lidar_type', "livox")

            if not map_name:
                response.success = False
                response.message = "地图名称不能为空"
                return response

            rospy.loginfo(f"Starting mapping: {map_name}, lidar_type: {lidar_type}")

            # 1. 停止导航launch
            rospy.loginfo("Stopping navigation launch...")
            self._stop_navigation_launch()

            # 等待导航节点完全退出，确保雷达驱动已经释放资源
            rospy.loginfo("Waiting for all navigation nodes to exit...")
            self._wait_for_navigation_nodes_exit()

            # 清理残留的 fast-lio 进程（特别是 laserMapping）
            self._cleanup_residual_processes()

            # 2. 启动建图launch
            rospy.loginfo("Starting mapping launch...")
            success = self._start_mapping_launch(lidar_type)

            if not success:
                response.success = False
                response.message = "启动建图launch失败"
                # 尝试恢复导航
                rospy.logwarn("Mapping launch failed, trying to restart navigation...")
                self._start_navigation_launch()
                return response

            # 3. 检查IMU数据
            if not self._wait_for_imu_ready():
                response.success = False
                response.message = "IMU数据异常，无法启动建图"
                rospy.logerr("IMU data is invalid, stopping mapping launch...")
                self._stop_mapping_launch()
                self._start_navigation_launch()
                return response

            # 4. 更新状态
            self.current_map_name = map_name
            self.is_mapping = True

            response.success = True
            response.message = f"建图启动成功: {map_name}"
            response.map_name = map_name

            rospy.loginfo(f"Mapping started successfully: {map_name}")

        except Exception as e:
            response.success = False
            response.message = f"启动建图错误: {str(e)}"
            rospy.logerr(f"Error in start_mapping: {str(e)}")

        return response

    def stop_mapping(self, req):
        """停止建图服务"""
        response = StopMappingResponse()

        try:
            if not self.is_mapping:
                response.success = False
                response.message = "当前没有进行建图"
                return response

            # 检查是否正在保存地图
            if self.is_saving_map:
                rospy.logwarn("Map save is in progress, waiting for it to complete...")
                rospy.loginfo("Please wait, stopping mapping will continue after save completes...")

                # 等待保存完成（最多等待120秒）
                wait_timeout = 120
                wait_start = time.time()

                while self.is_saving_map:
                    if time.time() - wait_start > wait_timeout:
                        response.success = False
                        response.message = "等待地图保存超时，请重试"
                        rospy.logerr("Timeout waiting for map save to complete")
                        return response
                    rospy.loginfo(f"Waiting for map save to complete... ({int(time.time() - wait_start)}s)")
                    time.sleep(1)

                rospy.loginfo("Map save completed, proceeding to stop mapping")

            rospy.loginfo("Stopping mapping...")

            # 1. 停止建图launch
            rospy.loginfo("Stopping mapping launch...")
            self._stop_mapping_launch()

            # 等待建图节点完全退出，确保雷达驱动已经释放资源
            rospy.loginfo("Waiting for all mapping nodes to exit...")
            self._wait_for_mapping_nodes_exit()

            # 清理残留的 fast-lio 进程（特别是 laserMapping）
            self._cleanup_residual_processes()

            # 2. 启动导航launch
            rospy.loginfo("Starting navigation launch...")
            self._start_navigation_launch()

            # 3. 更新状态
            map_name = self.current_map_name
            self.current_map_name = None
            self.is_mapping = False

            response.success = True
            response.message = f"建图已停止: {map_name}"

            rospy.loginfo(f"Mapping stopped successfully, navigation restarted")

        except Exception as e:
            response.success = False
            response.message = f"停止建图错误: {str(e)}"
            rospy.logerr(f"Error in stop_mapping: {str(e)}")

        return response

    def save_map(self, req):
        """保存地图服务"""
        response = SaveMapResponse()

        # 检查是否正在保存
        if self.is_saving_map:
            response.success = False
            response.message = "地图正在保存中，请稍后再试"
            rospy.logwarn("Map save already in progress, ignoring duplicate request")
            return response

        # 获取锁
        if not self.save_map_lock.acquire(blocking=False):
            response.success = False
            response.message = "地图正在保存中，请稍后再试"
            rospy.logwarn("Map save lock is held, cannot save now")
            return response

        try:
            self.is_saving_map = True
            rospy.loginfo("Acquired save map lock, starting save...")

            map_name = getattr(req, 'map_name', self.current_map_name)
            rospy.loginfo(f"Saving map: {map_name}")

            success = self._save_map_internal(map_name)

            response.success = success
            response.message = "Map saved successfully" if success else "Map save failed"
            if success:
                response.map_path = map_name
                response.yaml_path = f"{map_name}.yaml"
                response.pgm_path = f"{map_name}.pgm"
                response.pcd_path = f"{map_name}.pcd"

            rospy.loginfo(f"Map saving {'successful' if success else 'failed'}")

        except Exception as e:
            response.success = False
            response.message = f"Map save error: {str(e)}"
            rospy.logerr(f"Error saving map: {str(e)}")

        finally:
            # 清理残留的 fast-lio 进程（特别是 laserMapping）
            self._cleanup_residual_processes()

            # 启动导航launch
            rospy.loginfo("Starting navigation launch...")
            self._start_navigation_launch()
            # 3. 更新状态
            # 释放锁和状态
            self.is_saving_map = False
            self.last_save_time = time.time()  # 记录保存操作完成的时间
            self.save_map_lock.release()
            rospy.loginfo("Released save map lock")

        return response

    def rename_map(self, req):
        """重命名地图服务"""
        response = RenameMapResponse()

        try:
            old_name = getattr(req, 'old_name', '')
            new_name = getattr(req, 'new_name', '')

            if self.is_mapping:
                response.success = False
                response.message = "不能重命名当前正在使用的地图"
                return response

            if not old_name or not new_name:
                response.success = False
                response.message = "原地图名称和新地图名称不能为空"
                return response

            if old_name == new_name:
                response.success = False
                response.message = "新地图名称不能与原地图名称相同"
                return response

            rospy.loginfo(f"Renaming map: {old_name} -> {new_name}")

            success = self._rename_map_files(old_name, new_name)

            if success:
                # 更新任务点数据库
                self._update_task_points_database(old_name, new_name)

                response.success = True
                response.message = f"地图重命名成功: {old_name} -> {new_name}"
                rospy.loginfo(f"Map renamed successfully")
            else:
                response.success = False
                response.message = f"地图重命名失败: {old_name} -> {new_name}"
                rospy.logerr(f"Map rename failed")

        except Exception as e:
            response.success = False
            response.message = f"重命名地图错误: {str(e)}"
            rospy.logerr(f"Error in rename_map: {str(e)}")

        return response

    def delete_map(self, req):
        """Delete map service"""
        response = DeleteMapResponse()

        try:
            map_name = getattr(req, 'map_name', '')

            if not map_name:
                response.success = False
                response.message = "Map name cannot be empty"
                return response

            # Check if currently mapping this map
            if self.is_mapping and self.current_map_name == map_name:
                response.success = False
                response.message = "Cannot delete map currently being mapped"
                rospy.logwarn(f"Cannot delete map {map_name}: mapping in progress")
                return response

            # Get the actual current loaded map dynamically
            try:
                rospy.wait_for_service('/get_current_map', timeout=2.0)
                get_current_map = rospy.ServiceProxy('/get_current_map', GetCurrentMap)
                current_map_result = get_current_map()

                if current_map_result.current_map:
                    current_loaded_map = current_map_result.current_map
                    rospy.loginfo(f"Currently loaded map: {current_loaded_map}")

                    if map_name == current_loaded_map:
                        response.success = False
                        response.message = "Cannot delete map currently in use for navigation"
                        rospy.logwarn(f"Cannot delete map {map_name}: currently loaded as navigation map")
                        return response
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logwarn(f"Failed to get current map: {e}, proceeding with delete check")

            rospy.loginfo(f"Deleting map: {map_name}")

            success = self._delete_map_files(map_name)

            if success:
                self._delete_task_points_database(map_name)

                response.success = True
                response.message = f"Map deleted successfully: {map_name}"
                rospy.loginfo(f"Map deleted successfully")
            else:
                response.success = False
                response.message = f"Failed to delete map: {map_name}"
                rospy.logerr(f"Map delete failed")

        except Exception as e:
            response.success = False
            response.message = f"Error deleting map: {str(e)}"
            rospy.logerr(f"Error in delete_map: {str(e)}")

        return response

    def edit_map(self, req):
        """编辑地图服务"""
        response = EditMapResponse()

        try:
            map_name = getattr(req, 'map_name', '')
            points = getattr(req, 'points', [])
            operation = getattr(req, 'operation', '')

            rospy.loginfo(f"Edit map: {map_name}, operation: {operation}")

            if not map_name:
                response.success = False
                response.message = "地图名称不能为空"
                return response

            if not points or len(points) != 8:
                response.success = False
                response.message = "需要提供四个点的坐标 (x1,y1, x2,y2, x3,y3, x4,y4)"
                return response

            if operation not in ['fill', 'clear']:
                response.success = False
                response.message = "操作类型必须是 'fill' 或 'clear'"
                return response

            # 查找地图文件
            map_file = self._find_map_file(map_name)
            if not map_file:
                response.success = False
                response.message = f"找不到地图文件: {map_name}"
                return response

            # 读取地图配置
            with open(map_file, 'r', encoding='utf-8') as f:
                map_config = yaml.safe_load(f)

            # 获取PGM路径
            pgm_path = map_config.get('image', '')
            if not os.path.isabs(pgm_path):
                pgm_path = os.path.join(os.path.dirname(map_file), pgm_path)

            if not os.path.exists(pgm_path):
                response.success = False
                response.message = f"找不到PGM文件: {pgm_path}"
                return response

            # 处理地图编辑
            image_data = self._process_map_edit(pgm_path, points, operation, map_config)

            if image_data:
                response.success = True
                response.message = "地图编辑成功"
                response.image_data = image_data
                rospy.loginfo(f"Map edit completed successfully")
            else:
                response.success = False
                response.message = "地图编辑失败"

        except Exception as e:
            response.success = False
            response.message = f"地图编辑错误: {str(e)}"
            rospy.logerr(f"Error in edit_map: {str(e)}")

        return response

    def get_mapping_status(self, req):
        """获取建图状态"""
        response = TriggerResponse()

        try:
            # 计算活跃进程数（用于向后兼容）
            active_processes = 0
            if self.is_mapping and self.mapping_launch_process:
                active_processes = 1 if self.mapping_launch_process.poll() is None else 0

            # 使用JSON格式
            import json
            status_info = {
                "is_mapping": self.is_mapping,
                "is_saving_map": self.is_saving_map,
                "current_map_name": self.current_map_name if self.current_map_name else '',
                "active_processes": active_processes,
                "navigation_launch_running": self.navigation_launch_process is not None and self.navigation_launch_process.poll() is None,
                "mapping_launch_running": self.mapping_launch_process is not None and self.mapping_launch_process.poll() is None
            }

            response.success = True
            response.message = json.dumps(status_info, ensure_ascii=False)

            rospy.loginfo(f"Mapping status: {response.message}")

        except Exception as e:
            response.success = False
            response.message = json.dumps({"error": str(e)}, ensure_ascii=False)
            rospy.logerr(f"Error getting status: {str(e)}")

        return response

    def _start_navigation_launch(self):
        """启动导航launch"""
        try:
            launch_file = os.path.join(self.kuavo_navigation_path, 'launch', 'kuavo_navigation.launch')

            if not os.path.exists(launch_file):
                rospy.logerr(f"Navigation launch file not found: {launch_file}")
                return False

            rospy.loginfo(f"Starting navigation launch: {launch_file}")
            rospy.loginfo(f"Navigation parameters: {self.nav_params}")

            # 构建launch命令，传入参数
            cmd = [
                'roslaunch',
                'kuavo_navigation',
                'kuavo_navigation.launch',
                f'map:={self.nav_params["map"]}',
                f'rviz:={self.nav_params["rviz"]}'
            ]

            rospy.loginfo(f"Full navigation launch command: {' '.join(cmd)}")

            self.navigation_launch_process = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
                cwd=os.path.expanduser("~"),  # 设置工作目录为用户home目录
                preexec_fn=os.setsid
            )

            rospy.loginfo(f"Navigation launch started with PID: {self.navigation_launch_process.pid}")
            time.sleep(3)  # 等待节点初始化

            return True

        except Exception as e:
            rospy.logerr(f"Failed to start navigation launch: {e}")
            return False

    def _stop_navigation_launch(self):
        """停止导航launch"""
        if self.navigation_launch_process is None:
            return

        try:
            if self.navigation_launch_process.poll() is None:
                rospy.loginfo(f"Stopping navigation launch (PID: {self.navigation_launch_process.pid})...")
                try:
                    pgid = os.getpgid(self.navigation_launch_process.pid)
                    os.killpg(pgid, signal.SIGTERM)
                except (ProcessLookupError, OSError) as e:
                    rospy.logwarn(f"Failed to send SIGTERM to process group: {e}")

                # 等待进程结束
                try:
                    self.navigation_launch_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    rospy.logwarn("Navigation launch did not stop gracefully, using SIGKILL")
                    try:
                        pgid = os.getpgid(self.navigation_launch_process.pid)
                        os.killpg(pgid, signal.SIGKILL)
                        self.navigation_launch_process.wait()
                    except (ProcessLookupError, OSError) as e:
                        rospy.logwarn(f"Failed to send SIGKILL to process group: {e}")

                rospy.loginfo("Navigation launch stopped")
        except (ProcessLookupError, OSError) as e:
            rospy.logwarn(f"Navigation launch process error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error stopping navigation launch: {e}")

        self.navigation_launch_process = None

    def _start_mapping_launch(self, lidar_type):
        """启动建图launch"""
        try:
            launch_file = os.path.join(self.kuavo_mapping_path, 'launch', 'build_map.launch')

            if not os.path.exists(launch_file):
                rospy.logerr(f"Mapping launch file not found: {launch_file}")
                return False

            rospy.loginfo(f"Starting mapping launch: {launch_file} with lidar_type={lidar_type}")

            cmd = [
                'roslaunch',
                'kuavo_mapping',
                'build_map.launch',
                f'lidar_type:={lidar_type}'
            ]

            rospy.loginfo(f"Full mapping launch command: {' '.join(cmd)}")

            self.mapping_launch_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=os.path.expanduser("~"),  # 设置工作目录为用户home目录
                preexec_fn=os.setsid
            )

            rospy.loginfo(f"Mapping launch started with PID: {self.mapping_launch_process.pid}")
            time.sleep(5)  # 等待节点初始化

            return True

        except Exception as e:
            rospy.logerr(f"Failed to start mapping launch: {e}")
            return False

    def _stop_mapping_launch(self):
        """停止建图launch"""
        if self.mapping_launch_process is None:
            return

        try:
            if self.mapping_launch_process.poll() is None:
                rospy.loginfo(f"Stopping mapping launch (PID: {self.mapping_launch_process.pid})...")

                # 检查最近是否有保存操作
                time_since_last_save = time.time() - self.last_save_time
                SAVE_COOLDOWN = 35.0  # 保存操作冷却时间（秒），应大于 _save_map_internal 中的超时时间

                if time_since_last_save > SAVE_COOLDOWN:
                    # 超过冷却时间，可以安全地发送 SIGINT 触发 PCD 保存
                    rospy.loginfo(f"No recent save operation ({time_since_last_save:.1f}s ago), sending SIGINT to laserMapping")
                    try:
                        subprocess.run(['pkill', '-SIGINT', '-f', 'laserMapping'], timeout=2)
                        rospy.loginfo("Sent SIGINT to laserMapping")
                        time.sleep(2)
                    except:
                        pass
                else:
                    # 在冷却时间内，说明刚刚完成保存操作，避免重复发送 SIGINT
                    rospy.loginfo(f"Recent save operation ({time_since_last_save:.1f}s ago), skipping SIGINT to avoid conflict")

                # 先优雅停止 map_manager 节点，让它有时间清理资源
                rospy.loginfo("Stopping map_manager node gracefully...")
                try:
                    # 使用 rosnode kill 优雅停止 map_manager
                    subprocess.run(['rosnode', 'kill', '/map_manager'], timeout=5)
                    rospy.loginfo("Sent kill command to map_manager")
                    time.sleep(2)  # 等待 map_manager 清理资源
                except Exception as e:
                    rospy.logwarn(f"Failed to kill map_manager gracefully: {e}")

                # 然后停止整个launch
                rospy.loginfo("Stopping mapping launch process group...")
                try:
                    pgid = os.getpgid(self.mapping_launch_process.pid)
                    os.killpg(pgid, signal.SIGTERM)
                except (ProcessLookupError, OSError) as e:
                    rospy.logwarn(f"Failed to send SIGTERM to process group: {e}")

                # 等待进程结束
                try:
                    self.mapping_launch_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    rospy.logwarn("Mapping launch did not stop gracefully, using SIGKILL")
                    try:
                        pgid = os.getpgid(self.mapping_launch_process.pid)
                        os.killpg(pgid, signal.SIGKILL)
                        self.mapping_launch_process.wait()
                    except (ProcessLookupError, OSError) as e:
                        rospy.logwarn(f"Failed to send SIGKILL to process group: {e}")

                rospy.loginfo("Mapping launch stopped")
        except (ProcessLookupError, OSError) as e:
            rospy.logwarn(f"Mapping launch process error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error stopping mapping launch: {e}")

        self.mapping_launch_process = None

    def _wait_for_mapping_nodes_exit(self, timeout=15):
        """等待建图相关节点完全退出"""
        rospy.loginfo("Waiting for mapping nodes to exit...")

        # 建图相关的节点列表
        mapping_nodes = [
            '/map_manager',
            '/laserMapping',
            '/octomap_server',
            '/octomap_server_grid',
            '/pointcloud_to_laserscan'
        ]

        start_time = time.time()
        check_interval = 0.5  # 每0.5秒检查一次

        while time.time() - start_time < timeout:
            try:
                # 获取当前运行的节点列表
                result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=3)
                running_nodes = result.stdout

                # 检查是否还有建图相关的节点在运行
                remaining_nodes = [node for node in mapping_nodes if node in running_nodes]

                if not remaining_nodes:
                    rospy.loginfo("All mapping nodes have exited")
                    return True
                else:
                    rospy.loginfo(f"Waiting for nodes to exit: {remaining_nodes}")

            except Exception as e:
                rospy.logwarn(f"Error checking node status: {e}")

            time.sleep(check_interval)

        # 超时后显示警告，但继续执行
        try:
            result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=3)
            remaining_nodes = [node for node in mapping_nodes if node in result.stdout]
            if remaining_nodes:
                rospy.logwarn(f"Timeout waiting for mapping nodes to exit. Remaining: {remaining_nodes}")
            else:
                rospy.loginfo("All mapping nodes exited after timeout check")
        except:
            pass

        return True

    def _wait_for_navigation_nodes_exit(self, timeout=15):
        """等待导航相关节点完全退出"""
        rospy.loginfo("Waiting for navigation nodes to exit...")

        # 导航相关的节点列表
        # 注意：map_manager 不在列表中，因为它在导航和建图模式都需要
        navigation_nodes = [
            '/move_base',
            '/global_localization',
            '/global_planner',
            '/local_planner'
        ]

        start_time = time.time()
        check_interval = 0.5  # 每0.5秒检查一次

        while time.time() - start_time < timeout:
            try:
                # 获取当前运行的节点列表
                result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=3)
                running_nodes = result.stdout

                # 检查是否还有导航相关的节点在运行
                remaining_nodes = [node for node in navigation_nodes if node in running_nodes]

                if not remaining_nodes:
                    rospy.loginfo("All navigation nodes have exited")
                    return True
                else:
                    rospy.loginfo(f"Waiting for nodes to exit: {remaining_nodes}")

            except Exception as e:
                rospy.logwarn(f"Error checking node status: {e}")

            time.sleep(check_interval)

        # 超时后显示警告，但继续执行
        try:
            result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True, timeout=3)
            remaining_nodes = [node for node in navigation_nodes if node in result.stdout]
            if remaining_nodes:
                rospy.logwarn(f"Timeout waiting for navigation nodes to exit. Remaining: {remaining_nodes}")
            else:
                rospy.loginfo("All navigation nodes exited after timeout check")
        except:
            pass

        return True

    def _cleanup_residual_processes(self):
        """清理残留的 fast-lio 相关进程"""
        rospy.loginfo("Cleaning up residual fast-lio processes...")

        # 需要清理的进程名称列表
        process_names = ['laserMapping', 'laserOdometry']

        for process_name in process_names:
            try:
                # 使用 pgrep 查找进程
                result = subprocess.run(['pgrep', '-f', process_name], capture_output=True)

                if result.returncode == 0:
                    pids = result.stdout.decode().strip().split('\n')
                    rospy.loginfo(f"Found residual {process_name} processes: {pids}")

                    # 先尝试优雅终止（SIGTERM）
                    for pid in pids:
                        if pid:
                            try:
                                subprocess.run(['kill', '-TERM', pid], timeout=2)
                                rospy.loginfo(f"Sent SIGTERM to {process_name} (PID: {pid})")
                            except:
                                pass

                    # 等待 2 秒
                    time.sleep(2)

                    # 如果进程还在，强制杀死（SIGKILL）
                    result = subprocess.run(['pgrep', '-f', process_name], capture_output=True)
                    if result.returncode == 0:
                        pids = result.stdout.decode().strip().split('\n')
                        for pid in pids:
                            if pid:
                                try:
                                    subprocess.run(['kill', '-KILL', pid], timeout=2)
                                    rospy.loginfo(f"Sent SIGKILL to {process_name} (PID: {pid})")
                                except:
                                    pass

                    # 最终检查
                    result = subprocess.run(['pgrep', '-f', process_name], capture_output=True)
                    if result.returncode == 0:
                        rospy.logwarn(f"Failed to kill all {process_name} processes")
                    else:
                        rospy.loginfo(f"All {process_name} processes cleaned up")

            except Exception as e:
                rospy.logwarn(f"Error cleaning up {process_name}: {e}")

    def _save_map_internal(self, map_name):
        """内部地图保存实现"""
        try:
            rospy.loginfo(f"Saving map with name: {map_name}")

            # 创建目录
            maps_dir = os.path.expanduser("~/maps")
            pcd_dir = os.path.expanduser("~/pcd")
            os.makedirs(maps_dir, exist_ok=True)
            os.makedirs(pcd_dir, exist_ok=True)

            map_base = os.path.join(maps_dir, map_name)

            # 1. 保存2D地图
            rospy.loginfo("Saving 2D map...")
            save_cmd = ["rosrun", "map_server", "map_saver", "-f", map_base]
            result = subprocess.run(save_cmd, capture_output=True, text=True, timeout=60)

            if result.returncode != 0:
                rospy.logerr(f"2D map save failed: {result.stderr}")
                return False
            rospy.loginfo("2D map saved successfully")

            # 2. 修复YAML文件中的nan值
            yaml_file = f"{map_base}.yaml"
            if os.path.exists(yaml_file):
                try:
                    with open(yaml_file, 'r', encoding='utf-8') as f:
                        content = f.read()
                    if 'nan' in content:
                        content = content.replace('nan', '0.0')
                        with open(yaml_file, 'w', encoding='utf-8') as f:
                            f.write(content)
                        rospy.loginfo("Fixed 'nan' values in YAML file")
                except Exception as e:
                    rospy.logwarn(f"Failed to fix YAML file: {e}")

            # 3. 获取并保存PCD文件
            try:
                rospack = rospkg.RosPack()
                fast_lio_path = rospack.get_path('fast_lio')
                pcd_source_path = os.path.join(fast_lio_path, "PCD", "scans.pcd")

                rospy.loginfo(f"Looking for PCD file at: {pcd_source_path}")

                # 触发任务点保存
                try:
                    rospy.wait_for_service('/task_point', timeout=3.0)
                    from kuavo_mapping.srv import TaskPointOperation
                    task_service = rospy.ServiceProxy('/task_point', TaskPointOperation)
                    dummy_result = task_service(TaskPointOperationRequest.GET, "_dummy_save_trigger", 0.0, 0.0, 0.0)
                    rospy.loginfo("Triggered task point save")
                except Exception as e:
                    rospy.logwarn(f"Could not trigger task point save: {e}")

                # 发送 SIGINT 给 laserMapping 触发 PCD 保存
                try:
                    result = subprocess.run(['pgrep', '-f', 'laserMapping'], capture_output=True)
                    if result.returncode == 0:
                        rospy.loginfo("Sending SIGINT to laserMapping to trigger PCD save...")
                        subprocess.run(['pkill', '-SIGINT', '-f', 'laserMapping'])
                        # 不要在这里等待，下面统一等待 PCD 文件生成
                except Exception as e:
                    rospy.logwarn(f"Error triggering laserMapping: {e}")

                # 等待PCD文件生成（最多等待30秒）
                # 注意：这里需要等待 laserMapping 完成 PCD 写入，所以需要更长的超时时间
                timeout = 30
                pcd_check_interval = 1  # 每秒检查一次

                rospy.loginfo("Waiting for PCD file to be generated...")

                while not os.path.exists(pcd_source_path) and timeout > 0:
                    rospy.loginfo(f"Waiting for PCD file... {timeout}s remaining")
                    time.sleep(pcd_check_interval)
                    timeout -= pcd_check_interval

                if os.path.exists(pcd_source_path):
                    rospy.loginfo("PCD file generated successfully")
                else:
                    rospy.logwarn(f"PCD file not found after waiting: {pcd_source_path}")

                if os.path.exists(pcd_source_path):
                    # 复制PCD文件
                    import shutil
                    temp_pcd_path = os.path.join(maps_dir, f"temp_{map_name}.pcd")
                    target_pcd_path = os.path.join(maps_dir, f"{map_name}.pcd")

                    shutil.copy2(pcd_source_path, temp_pcd_path)
                    time.sleep(1)

                    # 降采样PCD文件
                    downsample_cmd = [
                        "rosrun", "kuavo_mapping", "downsample_pcd.py",
                        "--pcd", temp_pcd_path,
                        "--output_file", target_pcd_path
                    ]

                    result = subprocess.run(downsample_cmd, capture_output=True, text=True)

                    if result.returncode == 0 and os.path.exists(target_pcd_path):
                        rospy.loginfo(f"PCD file saved successfully: {target_pcd_path}")
                        try:
                            os.remove(temp_pcd_path)
                        except:
                            pass
                    else:
                        rospy.logwarn(f"PCD downsample failed, copying original file")
                        shutil.copy2(temp_pcd_path, target_pcd_path)
                else:
                    rospy.logwarn(f"PCD file not found: {pcd_source_path}")

            except Exception as e:
                rospy.logwarn(f"Error saving PCD file: {e}")

            # 4. 处理任务点数据库
            source_db_path = os.path.join(maps_dir, f"{map_name}_task_points.db")

            if os.path.exists(source_db_path):
                rospy.loginfo(f"Found task points database: {source_db_path}")
            else:
                temp_db_path = os.path.join(maps_dir, "temp_task_points.db")
                if os.path.exists(temp_db_path):
                    try:
                        os.rename(temp_db_path, source_db_path)
                        rospy.loginfo(f"Renamed temp database to: {source_db_path}")

                        # 更新数据库
                        script_dir = os.path.dirname(os.path.abspath(__file__))
                        update_script = os.path.join(script_dir, "update_task_points_db.py")

                        if os.path.exists(update_script):
                            result = subprocess.run(
                                ["python3", update_script, source_db_path, map_name],
                                capture_output=True, text=True
                            )
                            if result.returncode == 0:
                                rospy.loginfo("Database updated successfully")
                    except Exception as e:
                        rospy.logwarn(f"Error processing database: {e}")

            rospy.loginfo(f"Map saved successfully: {map_name}")
            return True

        except Exception as e:
            rospy.logerr(f"Internal save map error: {e}")
            return False

    def _rename_map_files(self, old_name, new_name):
        """重命名地图相关文件"""
        try:
            maps_dir = os.path.expanduser("~/maps")
            extensions = ['.yaml', '.pgm', '.pcd']
            renamed_files = []

            for ext in extensions:
                old_file = os.path.join(maps_dir, f"{old_name}{ext}")
                new_file = os.path.join(maps_dir, f"{new_name}{ext}")

                if os.path.exists(old_file):
                    os.rename(old_file, new_file)
                    renamed_files.append(new_file)
                    rospy.loginfo(f"Renamed file: {old_file} -> {new_file}")

            # 更新YAML文件中的image路径
            yaml_file = os.path.join(maps_dir, f"{new_name}.yaml")
            if os.path.exists(yaml_file):
                try:
                    with open(yaml_file, 'r', encoding='utf-8') as f:
                        map_config = yaml.safe_load(f)

                    if 'image' in map_config:
                        new_pgm_path = os.path.join(maps_dir, f"{new_name}.pgm")
                        if os.path.exists(new_pgm_path):
                            old_image_path = map_config['image']
                            map_config['image'] = new_pgm_path
                            rospy.loginfo(f"Updated YAML image path")

                    with open(yaml_file, 'w', encoding='utf-8') as f:
                        yaml.dump(map_config, f, default_flow_style=False, allow_unicode=True)
                except Exception as yaml_e:
                    rospy.logerr(f"Error updating YAML file: {str(yaml_e)}")

            return len(renamed_files) > 0

        except Exception as e:
            rospy.logerr(f"Error renaming map files: {str(e)}")
            return False

    def _update_task_points_database(self, old_name, new_name):
        """更新任务点数据库"""
        try:
            maps_dir = os.path.expanduser("~/maps")
            possible_db_files = [
                f"{old_name}_task_points.db",
                f"temp_task_points.db"
            ]

            for db_file in possible_db_files:
                db_path = os.path.join(maps_dir, db_file)
                if os.path.exists(db_path):
                    new_db_file = f"{new_name}_task_points.db"
                    new_db_path = os.path.join(maps_dir, new_db_file)

                    if db_file != new_db_file:
                        os.rename(db_path, new_db_path)

                    # 更新数据库内容
                    script_dir = os.path.dirname(os.path.abspath(__file__))
                    update_script = os.path.join(script_dir, "update_task_points_db.py")

                    if os.path.exists(update_script):
                        result = subprocess.run(
                            ["python3", update_script, new_db_path, new_name],
                            capture_output=True, text=True
                        )
                        if result.returncode == 0:
                            rospy.loginfo(f"Database updated successfully")
                            return True

            return True

        except Exception as e:
            rospy.logerr(f"Error updating database: {str(e)}")
            return False

    def _delete_map_files(self, map_name):
        """删除地图相关文件"""
        try:
            maps_dir = os.path.expanduser("~/maps")
            extensions = ['.yaml', '.pgm', '.pcd']
            deleted_files = []

            for ext in extensions:
                file_path = os.path.join(maps_dir, f"{map_name}{ext}")

                if os.path.exists(file_path):
                    os.remove(file_path)
                    deleted_files.append(file_path)
                    rospy.loginfo(f"Deleted file: {file_path}")

            return len(deleted_files) > 0

        except Exception as e:
            rospy.logerr(f"Error deleting map files: {str(e)}")
            return False

    def _delete_task_points_database(self, map_name):
        """删除任务点数据库"""
        try:
            maps_dir = os.path.expanduser("~/maps")
            db_file = f"{map_name}_task_points.db"
            db_path = os.path.join(maps_dir, db_file)

            if os.path.exists(db_path):
                os.remove(db_path)
                rospy.loginfo(f"Deleted task points database: {db_path}")
                return True

            return True

        except Exception as e:
            rospy.logerr(f"Error deleting database: {str(e)}")
            return False

    def _find_map_file(self, map_name):
        """查找地图文件"""
        possible_paths = [
            os.path.expanduser(f"~/maps/{map_name}.yaml"),
            f"/root/maps/{map_name}.yaml",
            os.path.expanduser(f"~/.ros/maps/{map_name}.yaml"),
        ]

        for path in possible_paths:
            if os.path.exists(path):
                return path
        return None

    def _process_map_edit(self, pgm_path, points, operation, map_config):
        """处理地图编辑操作"""
        try:
            from PIL import Image, ImageDraw
            import base64
            import io

            img = Image.open(pgm_path)
            original_mode = img.mode

            if img.mode != 'RGB':
                img = img.convert('RGB')

            # 提取4个点：左上、左下、右上、右下 -> 重新排序为顺时针
            p0 = (int(points[0]), int(points[1]))   # 左上
            p1 = (int(points[2]), int(points[3]))   # 左下
            p2 = (int(points[4]), int(points[5]))   # 右上
            p3 = (int(points[6]), int(points[7]))   # 右下

            image_points = [p0, p1, p3, p2]  # 顺时针：左上、左下、右下、右上

            # 确保坐标在图像范围内
            for i in range(len(image_points)):
                pixel_x, pixel_y = image_points[i]
                pixel_x = max(0, min(pixel_x, img.width - 1))
                pixel_y = max(0, min(pixel_y, img.height - 1))
                image_points[i] = (pixel_x, pixel_y)

            draw = ImageDraw.Draw(img)

            if len(image_points) == 4:
                if operation == 'fill':
                    draw.polygon(image_points, fill=(0, 0, 0))
                else:
                    draw.polygon(image_points, fill=(255, 255, 255))

                if original_mode != 'RGB':
                    img = img.convert(original_mode)

                img.save(pgm_path)

                # 生成base64图片数据
                display_img = img.convert('RGB')
                buffer = io.BytesIO()
                display_img.save(buffer, format='PNG')
                img_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')

                return img_base64

            return None

        except Exception as e:
            rospy.logerr(f"Error processing map edit: {e}")
            return None

    def _is_valid_imu_data(self, imu_msg):
        """检查IMU数据是否有效"""
        acc = imu_msg.linear_acceleration
        gyr = imu_msg.angular_velocity

        # 检查是否为0向量
        if (abs(acc.x) < 1e-10 and abs(acc.y) < 1e-10 and abs(acc.z) < 1e-10 and
            abs(gyr.x) < 1e-10 and abs(gyr.y) < 1e-10 and abs(gyr.z) < 1e-10):
            return False

        # 检查NaN或无穷大
        import numpy as np
        if any(np.isnan([acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z])) or \
           any(np.isinf([acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z])):
            return False

        # 检查加速度是否合理
        acc_magnitude = np.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
        if acc_magnitude < 0.1 or acc_magnitude > 50.0:
            return False

        # 检查角速度是否合理
        gyr_magnitude = np.sqrt(gyr.x**2 + gyr.y**2 + gyr.z**2)
        if gyr_magnitude > 50.0:
            return False

        return True

    def _wait_for_imu_ready(self):
        """等待IMU数据正常"""
        rospy.loginfo("Waiting for IMU data...")

        max_wait_time = 60
        wait_interval = 1
        start_time = time.time()

        while time.time() - start_time < max_wait_time:
            try:
                imu_data = rospy.wait_for_message('/livox/imu', Imu, timeout=2.0)

                if self._is_valid_imu_data(imu_data):
                    import numpy as np
                    acc = imu_data.linear_acceleration
                    gyr = imu_data.angular_velocity
                    acc_mag = np.sqrt(acc.x**2 + acc.y**2 + acc.z**2)
                    gyr_mag = np.sqrt(gyr.x**2 + gyr.y**2 + gyr.z**2)
                    rospy.loginfo(f"✓ IMU data valid! Acc: {acc_mag:.3f}m/s², Gyro: {gyr_mag:.3f}rad/s")
                    return True
                else:
                    rospy.logwarn("✗ IMU data invalid, waiting...")

            except rospy.ROSException:
                rospy.logwarn("Waiting for IMU data...")

            time.sleep(wait_interval)

        rospy.logerr("IMU data timeout!")
        return False

    def _monitor_processes(self, event):
        """监控launch进程状态"""
        # 检查导航launch
        if self.navigation_launch_process is not None:
            if self.navigation_launch_process.poll() is not None:
                rospy.logwarn("Navigation launch process exited unexpectedly")
                self.navigation_launch_process = None

        # 检查建图launch
        if self.mapping_launch_process is not None:
            if self.mapping_launch_process.poll() is not None:
                rospy.logwarn("Mapping launch process exited unexpectedly")
                self.mapping_launch_process = None
                if self.is_mapping:
                    rospy.logwarn("Mapping launch exited while is_mapping=True, resetting state")
                    self.is_mapping = False
                    self.current_map_name = None

    def cleanup(self):
        """清理资源"""
        rospy.loginfo("Cleaning up navigation service manager...")

        self._stop_mapping_launch()
        self._stop_navigation_launch()

        self.is_mapping = False
        self.current_map_name = None

        rospy.loginfo("Navigation service manager cleaned up")


def main():
    try:
        manager = NavigationServiceManager()
        rospy.loginfo("Navigation Service Manager ready")

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation Service Manager interrupted")
    except Exception as e:
        rospy.logerr(f"Navigation Service Manager error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        if 'manager' in locals():
            manager.cleanup()


if __name__ == '__main__':
    main()

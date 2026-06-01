import math
from copy import deepcopy
import time
import os 
import rospkg

from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.stage import add_reference_to_stage
from scipy.spatial.transform import Rotation

from nio import ROOT_PATH
from nio.agent import Kuavo
from nio.camera import Sensor
import rospy
from std_msgs.msg import Float32
from kuavo_msgs.msg import jointCmd
from kuavo_msgs.msg import sensorsData
from std_srvs.srv import SetBool, SetBoolResponse  # 添加服务消息类型导入

ASSET = ROOT_PATH / "assets"

# SIM_CFG = {"decimation": 1, "physics_dt": 1 / 400}
SIM_CFG = {"decimation": 15, "physics_dt": 1 / 1000}

SENSOR_CFG = {"prim_path": "/World/Kuavo"}

"""
    基础环境
"""
class Env:
    def __init__(self, isaac_robot_version, use_camera_ros_topic_flag, use_point_cloud_flag, scene_index, robot_scene_point_index):
        # 存储ROS参数
        self._scene_index = scene_index
        self._isaac_robot_version = isaac_robot_version
        self._use_camera_ros_topic_flag = use_camera_ros_topic_flag
        self._use_point_cloud_flag = use_point_cloud_flag
        self._robot_scene_point_index = robot_scene_point_index
        """
            是否开启深度相机点云功能
            0 : 关闭
            1 : 开启
        """
        if self._use_point_cloud_flag == 0:
            self.USE_POINT_CLOUD_FLAG_BOOL = False
        elif self._use_point_cloud_flag == 1:
            self.USE_POINT_CLOUD_FLAG_BOOL = True

        """
            40 - 4代短手短脚
            45 - 4pro长手 
        """
        self.AGENT_CFG = {
                "init_pos": [-0.00593, 0, 0.815],
                "init_roll": 0,
                "init_pitch": 0.0,
                "init_yaw": 0,
        }
        if self._isaac_robot_version == 40 or self._isaac_robot_version == 43:
            self.AGENT_CFG = {
                "init_pos": [-0.00593, 0, 0.78],
                "init_roll": 0,
                "init_pitch": 0.0,
                "init_yaw": 0,
            }
        """
            0 - 车企检测
            1 - 比赛场景
            2 - 默认地板场景
        """
        if self._scene_index == 0:
            self.SCENE_CFG = {
                "warehouse": {"position": [0, 0, 0], "orientation": [1, 0, 0, 0]},
                "car": {"position": [0.8283, 2.54758, 0.0], "orientation": [0, 0, -90]},
            }
        elif self._scene_index == 1:
            # 长腿 | 机器人
            self.SCENE_CFG = {
                "point_1": {"position": [11.8283, -8.7758, 0.0], "orientation": [0, 0, -180]}, # 沙地前
                "point_2": {"position": [2.3283, 9.458, -0.051], "orientation": [0, 0, -90]},  # 导航场景
                "point_3": {"position": [9.5283, 8.4758, -0.052], "orientation": [0, 0, 0]},   # 搬运场景
                "point_4": {"position": [0.2283, 8.7758, 0.0], "orientation": [0, 0, 0]},      # 上斜坡前
                "point_5": {"position": [4.9083, -8.7758, 0.0], "orientation": [0, 0, -180]},  # 上楼梯前
            }
            # 短腿 | 机器人
            if self._isaac_robot_version == 40 or self._isaac_robot_version == 43:
                self.SCENE_CFG = {
                    "point_1": {"position": [11.8283, -8.7758, 0.0], "orientation": [0, 0, -180]}, # 沙地前
                    "point_2": {"position": [2.3283, 9.458, -0.044], "orientation": [0, 0, -90]},  # 导航场景
                    "point_3": {"position": [9.5283, 8.4758, -0.045], "orientation": [0, 0, 0]},   # 搬运场景
                    "point_4": {"position": [0.2283, 8.7758, 0.0], "orientation": [0, 0, 0]},      # 上斜坡前
                    "point_5": {"position": [4.9083, -8.7758, 0.0], "orientation": [0, 0, -180]},  # 上楼梯前
                }
        elif self._scene_index == 2:
            # 长腿 | 机器人
            self.SCENE_CFG = {
                "point_1": {"position": [0.39, 0.0, 0.0], "orientation": [0, 0, 0]}, # 沙地前
            }
            # 短腿 | 机器人
            if self._isaac_robot_version == 40 or self._isaac_robot_version == 43:
                self.SCENE_CFG = {
                    "point_1": {"position": [0.39, 0.0, 0.0], "orientation": [0, 0, 0]}, # 沙地前
                }
        elif self._scene_index == 3:
            # 长腿 | 机器人
            self.SCENE_CFG = {
                "point_1": {"position": [0.39, 0.0, 0.0], "orientation": [0, 0, 0]}, # 沙地前
            }
            # 短腿 | 机器人
            if self._isaac_robot_version == 40 or self._isaac_robot_version == 43:
                self.SCENE_CFG = {
                    "point_1": {"position": [0.39, 0.0, 0.0], "orientation": [0, 0, 0]}, # 沙地前
                }

        """
            是否开启相机ROS topic发布功能
            0 : 关闭
            1 : 开启
        """
        if self._use_camera_ros_topic_flag == 0:
            self.enable_robot_camera = False
        elif self._use_camera_ros_topic_flag == 1:
            self.enable_robot_camera = True

        # 添加rospack
        self.rospack = rospkg.RosPack()
        self.warehouse = None
        self.car = None
        self.agent = None
        self.rsd455 = None
        self._load_scene()
        self._load_agent()
        self._load_sensor()
        self.sim_cfg = deepcopy(SIM_CFG)
        self.decimation = self.sim_cfg["decimation"]
        self.sim = SimulationContext(
            backend="numpy",
            physics_dt=self.sim_cfg["physics_dt"],
            rendering_dt=self.sim_cfg["physics_dt"] * self.sim_cfg["decimation"],
        )
        self.imu_data = None
        self.joint_data = None
        self.action = None
        
        # 添加用于计算加速度的变量
        self.last_joint_velocities = None
        self.last_time = None
        
        self.robot_sensor_data_pub = rospy.Publisher('/sensors_data_raw', sensorsData, queue_size=10)
        self.robot_joint_cmd_sub = rospy.Subscriber('/joint_cmd', jointCmd, self.joint_cmd_callback)
        
        # 添加仿真启动服务
        self.sim_running = True
        self.sim_start_srv = rospy.Service('sim_start', SetBool, self.sim_start_callback)
                    
    def reset(self):
        self.sim.reset()
        self.agent.initialize()
        self.rsd455.initialize()

    def joint_cmd_callback(self, msg):
        """将jointCmd消息转换为action字典格式

        action:  {
        
        'arms': {
            'ctrl_mode': 'velocity', 
            'joint_values': [0.01509438010286756, 0.015377446995305593, 0.0034525364617042863, -0.0009044164478106839, 0.0001279102113479343, -0.0001131726509593715, 0.023064709528283073, 0.02322650023520489, 2.094678484505175e-05, -2.2767468949554025e-05, 0.004178697550915565, -0.004201821757199927, -0.0007678927510557545, -0.0007781176865619842]}, 
            
        'legs': {
            'ctrl_mode': 'effort', 
            'joint_values': [-2.547679918973073, 2.6440293256604406, -0.2590461228134409, 0.39808970601301596, 5.5078389323387, 4.970989263532492, -20.697055551442972, -18.349446762674294, 14.295794209413375, 12.987065913521176, -0.24207946291522914, -0.2384792461664086]}
            
        }

        action格式:
        {
            "arms": {
                "ctrl_mode": "position",
                "joint_values": [...],
                "stiffness": [...],  # kp values
                "dampings": [...]    # kd values
            },
            "legs": {
            "ctrl_mode": "effort",
                "joint_values": [...],
                "stiffness": [...],
                "dampings": [...]
            }
        }
        """
        # msg数据
        joint_cmd_tau = msg.tau
        joint_cmd_v = msg.joint_v
        joint_cmd_q = msg.joint_q

        # 构建关键控制数据
        action = {
            "arms": {
                "ctrl_mode": "velocity",
                "joint_values": None,
            },
            "legs": {
                "ctrl_mode": "effort",
                "joint_values": None,
            }
        }
        
        # 腿部关节力矩赋值优化
        joint_cmd_leg_tau = [0.0] * 12
        for i in range(6):
            # 左腿力矩 (0,2,4,6,8,10)
            joint_cmd_leg_tau[i*2] = joint_cmd_tau[i]
            # 右腿力矩 (1,3,5,7,9,11)
            joint_cmd_leg_tau[i*2+1] = joint_cmd_tau[i+6]

        action["legs"]["joint_values"] = joint_cmd_leg_tau

        # 手部关节速度赋值优化
        joint_cmd_arm_v = [0.0] * 14
        for i in range(7):
            # 左手速度 (0,2,4,6,8,10,12)
            joint_cmd_arm_v[i*2] = joint_cmd_v[i+12]
            # 右手速度 (1,3,5,7,9,11,13)
            joint_cmd_arm_v[i*2+1] = joint_cmd_v[i+19]

        action["arms"]["joint_values"] = joint_cmd_arm_v
        
        # 赋值
        self.action = action
    
    def update_sensor_data(self):
        """
            发布/sensor_data_raw
            --- IMU Data --- :  
            {'time': 0.010000000707805157, 
            'physics_step': 11.0, 
            'lin_acc': array([  359.4783 ,   616.86053, -2586.8992 ], dtype=float32), 
            'ang_vel': array([ 1.5214992e-03, -7.1768576e-05,  1.6091872e-03], dtype=float32), 
            'orientation': array([ 0.99997777, -0.00509261,  0.00163787,  0.00398086], dtype=float32)}
        """
        self.imu_data = self.rsd455.imu_sensor.get_current_frame(read_gravity=True) # 获取数据
        self.joint_data = self.agent.get_obs() # 获取关节数据
        # print( " --- IMU Data --- : ", self.imu_data)
        
        # 数据组合
        sensor_data = sensorsData()
        
        # 填充header和sensor_time
        current_time = rospy.Time.from_sec(float(self.imu_data["time"]))
        sensor_data.header.stamp = current_time
        sensor_data.header.frame_id = "world"  # 设置适当的frame_id
        sensor_data.sensor_time = current_time
        
        # init data
        q_leg = self.joint_data["joint_state"]["legs_positions"].tolist()
        dq_leg = self.joint_data["joint_state"]["legs_velocities"].tolist()
        q_arm = self.joint_data["joint_state"]["arms_positions"].tolist()
        dq_arm = self.joint_data["joint_state"]["arms_velocities"].tolist()
        current_leg = self.joint_data["joint_state"]["legs_applied_effort"].tolist()
        current_arm = self.joint_data["joint_state"]["arms_applied_effort"].tolist()

        time = float(self.imu_data["time"])
        physics_step = float(self.imu_data["physics_step"])
        linear_acceleration = self.imu_data["lin_acc"].tolist()
        angular_velocity = self.imu_data["ang_vel"].tolist()
        orientation = self.imu_data["orientation"].tolist()

        # 关节数据赋值优化
        # 初始化数组
        sensor_data.joint_data.joint_q = [0.0] * 28
        sensor_data.joint_data.joint_v = [0.0] * 28
        sensor_data.joint_data.joint_vd = [0.0] * 28
        sensor_data.joint_data.joint_torque = [0.0] * 28
        
        # 腿部数据赋值（0-11）
        for i in range(6):
            # 左腿数据 (0-5)
            sensor_data.joint_data.joint_q[i] = q_leg[i*2]
            sensor_data.joint_data.joint_v[i] = dq_leg[i*2]
            sensor_data.joint_data.joint_torque[i] = current_leg[i*2]
            
            # 右腿数据 (6-11)
            sensor_data.joint_data.joint_q[i+6] = q_leg[i*2+1]
            sensor_data.joint_data.joint_v[i+6] = dq_leg[i*2+1]
            sensor_data.joint_data.joint_torque[i+6] = current_leg[i*2+1]
        
        # 手臂数据赋值（12-25）
        for i in range(7):
            # 左手数据 (12-18)
            sensor_data.joint_data.joint_q[i+12] = q_arm[i*2]
            sensor_data.joint_data.joint_v[i+12] = dq_arm[i*2]
            sensor_data.joint_data.joint_torque[i+12] = current_arm[i*2]
            
            # 右手数据 (19-25)
            sensor_data.joint_data.joint_q[i+19] = q_arm[i*2+1]
            sensor_data.joint_data.joint_v[i+19] = dq_arm[i*2+1]
            sensor_data.joint_data.joint_torque[i+19] = current_arm[i*2+1]
        
        # 获取头部数据
        if "head_positions" in self.joint_data["joint_state"]:
            head_positions = self.joint_data["joint_state"]["head_positions"].tolist()
            head_velocities = self.joint_data["joint_state"]["head_velocities"].tolist()
            head_applied_effort = self.joint_data["joint_state"]["head_applied_effort"].tolist()
            
            # 头部数据 (26-27)
            for i in range(2):
                sensor_data.joint_data.joint_q[26+i] = head_positions[i]
                sensor_data.joint_data.joint_v[26+i] = head_velocities[i]
                sensor_data.joint_data.joint_torque[26+i] = head_applied_effort[i]
        else:
            # 如果没有头部数据，设置为0
            for i in range(26, 28):
                sensor_data.joint_data.joint_q[i] = 0.0
                sensor_data.joint_data.joint_v[i] = 0.0
                sensor_data.joint_data.joint_torque[i] = 0.0

        # IMU数据
        sensor_data.imu_data.gyro.x = angular_velocity[0]  # ang_vel
        sensor_data.imu_data.gyro.y = angular_velocity[1]  # ang_vel
        sensor_data.imu_data.gyro.z = angular_velocity[2]  # ang_vel
        sensor_data.imu_data.acc.x = linear_acceleration[0]  # lin_acc
        sensor_data.imu_data.acc.y = linear_acceleration[1]  # lin_acc
        sensor_data.imu_data.acc.z = linear_acceleration[2]  # lin_acc
        sensor_data.imu_data.quat.w = orientation[0]  # 旋转矩阵
        sensor_data.imu_data.quat.x = orientation[1]  # 旋转矩阵
        sensor_data.imu_data.quat.y = orientation[2]  # 旋转矩阵
        sensor_data.imu_data.quat.z = orientation[3]  # 旋转矩阵

        # 计算关节加速度
        current_time = float(self.imu_data["time"])
        if self.last_joint_velocities is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                # 计算腿部关节加速度
                for i in range(12):
                    sensor_data.joint_data.joint_vd[i] = (sensor_data.joint_data.joint_v[i] - self.last_joint_velocities[i]) / dt
                
                # 计算手臂关节加速度
                for i in range(12, 26):
                    sensor_data.joint_data.joint_vd[i] = (sensor_data.joint_data.joint_v[i] - self.last_joint_velocities[i]) / dt
                
                # 计算头部关节加速度
                if "head_positions" in self.joint_data["joint_state"]:
                    for i in range(26, 28):
                        sensor_data.joint_data.joint_vd[i] = (sensor_data.joint_data.joint_v[i] - self.last_joint_velocities[i]) / dt
                else:
                    # 如果没有头部数据，加速度保持为0
                    sensor_data.joint_data.joint_vd[26] = 0.0
                    sensor_data.joint_data.joint_vd[27] = 0.0
        
        # 更新上一时刻的速度和时间
        self.last_joint_velocities = sensor_data.joint_data.joint_v.copy()
        self.last_time = current_time
        
        # 发布数据
        self.robot_sensor_data_pub.publish(sensor_data)
        
    def step(self):
        # 添加仿真状态检查
        if not self.sim_running:
            return

        # 发布数据 
        if self.enable_robot_camera:
            self.rsd455.get_obs()
        
        # 渲染此时下发cmd和反馈state
        for _ in range(self.decimation):
            # 下发控制
            if self.action is not None:
                self.agent.apply_action(self.action)            
            # 真正环境交互
            self.sim.step(render=False) 
            # IMU 传感器数据 | 机器人关节数据
            self.update_sensor_data()
        self.sim.render() # 同步渲染

    def get_sensor(self):
        return self.rsd455

    def get_agent(self):
        return self.agent

    def _load_scene(self):
        cfg = deepcopy(self.SCENE_CFG)
        if self._scene_index == 0:
            add_reference_to_stage(
                usd_path=str(ASSET / "car/car.usd"), prim_path="/World/Scene/car"
            )
            self.car = XFormPrim("/World/Scene/car")
            initial_quat = Rotation.from_euler(
                "xyz", cfg["car"]["orientation"], degrees=True
            ).as_quat()[[3, 0, 1, 2]]
            self.car.set_world_pose(cfg["car"]["position"], initial_quat)
            self.sphere1 = XFormPrim("/World/Scene/car/Sphere1")
        elif self._scene_index == 1:
            add_reference_to_stage(
                usd_path=str(ASSET / "scene/scene.usd"), prim_path="/World"
            )
            self.car = XFormPrim("/World/Scene")
            
            # 根据ROBOT_SCENE_POINT_INDEX选择对应的点位
            point_key = f"point_{self._robot_scene_point_index}"
            if point_key not in cfg:
                print(f"Warning: {point_key} not found in SCENE_CFG, using point_1 as default")
                point_key = "point_1"
            
            selected_point = cfg[point_key]
            initial_quat = Rotation.from_euler(
                "xyz", selected_point["orientation"], degrees=True
            ).as_quat()[[3, 0, 1, 2]]
            self.car.set_world_pose(selected_point["position"], initial_quat)
        elif self._scene_index == 2:
            add_reference_to_stage(
                usd_path=str(ASSET / "flat_road.usd"), prim_path="/World"
            )
            self.car = XFormPrim("/World/FlatGrid")
            
            # 根据ROBOT_SCENE_POINT_INDEX选择对应的点位
            point_key = f"point_{self._robot_scene_point_index}"
            if point_key not in cfg:
                print(f"Warning: {point_key} not found in SCENE_CFG, using point_1 as default")
                point_key = "point_1"
            
            selected_point = cfg[point_key]
            initial_quat = Rotation.from_euler(
                "xyz", selected_point["orientation"], degrees=True
            ).as_quat()[[3, 0, 1, 2]]
            self.car.set_world_pose(selected_point["position"], initial_quat)
        elif self._scene_index == 3:
            add_reference_to_stage(
                usd_path=str(ASSET / "flat_stairClimb.usd"), prim_path="/World"
            )
            self.car = XFormPrim("/World/FlatGrid")
            
            # 根据ROBOT_SCENE_POINT_INDEX选择对应的点位
            point_key = f"point_{self._robot_scene_point_index}"
            if point_key not in cfg:
                print(f"Warning: {point_key} not found in SCENE_CFG, using point_1 as default")
                point_key = "point_1"
            
            selected_point = cfg[point_key]
            initial_quat = Rotation.from_euler(
                "xyz", selected_point["orientation"], degrees=True
            ).as_quat()[[3, 0, 1, 2]]
            self.car.set_world_pose(selected_point["position"], initial_quat)

    def _load_agent(self):
        agent_cfg = deepcopy(self.AGENT_CFG)
        roll = math.radians(agent_cfg["init_roll"])
        pitch = math.radians(agent_cfg["init_pitch"])
        yaw = math.radians(agent_cfg["init_yaw"])
        initial_quat = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()[
            [3, 0, 1, 2]
        ]
        
        # 从ROS包中获取USD文件路径
        try:
            kuavo_assets_path = self.rospack.get_path('kuavo_assets')
            usd_path = os.path.join(kuavo_assets_path, 
                                  f'models/biped_s{self._isaac_robot_version}/urdf/biped_s{self._isaac_robot_version}.usd')
            if not os.path.exists(usd_path):
                rospy.logwarn(f"USD file not found at {usd_path}, falling back to default path")
                usd_path = str(ASSET / f"biped_s{self._isaac_robot_version}.usd")
        except rospkg.ResourceNotFound:
            rospy.logwarn("kuavo_assets package not found, falling back to default path")
            usd_path = str(ASSET / f"biped_s{self._isaac_robot_version}.usd")
        
        self.agent = Kuavo(usd_path)
        self.agent.set_world_pose(agent_cfg["init_pos"], initial_quat)

    def _load_sensor(self):
        snsor_cfg = deepcopy(SENSOR_CFG)
        self.rsd455 = Sensor(
            usd_path=str(ASSET / "rsd455.usd"), prim_path=snsor_cfg["prim_path"],
            enable_pointcloud_flag=self.USE_POINT_CLOUD_FLAG_BOOL
        )

    def shutdown(self) -> None:
        self.sim.stop()

    def sim_start_callback(self, req):
        """
        仿真启动服务的回调函数
        Args:
            req: SetBool请求，data字段为True表示启动仿真，False表示停止仿真
        Returns:
            SetBoolResponse: 服务响应
        """
        response = SetBoolResponse()
        
        self.sim_running = req.data

        if req.data:
            rospy.loginfo("Simulation started")
        else:
            rospy.loginfo("Simulation stopped")
        
        response.success = True
        response.message = "Simulation control successful"
        
        return response

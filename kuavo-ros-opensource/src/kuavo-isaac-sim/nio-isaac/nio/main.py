import copy
import json
import time

from nio.env import Env
from nio.tcp import PersistentTcpClient, json2bin

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
# pylint: disable=W0105
"""
action = {
            "arms":{"ctrl_mode": "position",
                    "joint_values": Optional[List, numpy.ndarray, torch.Tensor],
                    "stiffness": Optional[List, numpy.ndarray, torch.Tensor],
                    "dampings": Optional[List, numpy.ndarray, torch.Tensor],
                    },
            "legs": {"ctrl_mode": "effort",
                    "joint_values": Optional[List, numpy.ndarray, torch.Tensor],
                    "stiffness": Optional[List, numpy.ndarray, torch.Tensor],
                    "dampings": Optional[List, numpy.ndarray, torch.Tensor],
                    }
        }
"""

"""
obs_agent:  

{'joint_state': 

{'arms_positions': array([ 6.7899339e-02,  6.6581585e-02, -1.4145547e-02,  1.1513305e-02,
        1.9962389e-04, -2.5846387e-04,  3.5964666e-07,  3.5432970e-07,
        2.7999916e-04, -3.2667772e-04,  6.9805950e-02,  6.8463989e-02,
        8.8377640e-04, -1.9650348e-03], dtype=float32), 
        
'arms_velocities': array([ 7.6536715e-02,  7.6023042e-02, -2.7579060e-03,  1.7340450e-02,
        8.3563675e-05,  1.0103192e-03,  8.2413724e-04, -6.6661090e-04,
       -6.3904811e-04,  4.4197246e-04,  6.2549159e-02,  6.3240990e-02,
        3.9022765e-03, -2.3816221e-03], dtype=float32), 
        
'arms_applied_effort': array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
      dtype=float32), 
      
'legs_positions': array([-0.02000966,  0.01999262, -0.00154832,  0.00158663, -0.65413886,
       -0.65397155,  0.955847  ,  0.9557412 , -0.24546012, -0.24552198,
        0.01998793, -0.01977446], dtype=float32), 
        
'legs_velocities': array([ 0.00621787,  0.01130222,  0.00022094, -0.01070856,  0.06646577,
        0.06809103, -0.07139311, -0.07885165,  0.0892537 ,  0.09069658,
        0.0034664 ,  0.01848834], dtype=float32), 
        
'legs_applied_effort': array([-0.02000695,  0.01999509, -0.00155124,  0.0015864 , -0.6541672 , -0.6540032 ,  
                               0.95580333,  0.9557031 , -0.2455988 , -0.24566151, 0.0199839 , -0.01978234 ], dtype=float32)}, 

'body_state': 

{'world_pos': array([-0.00651233,  0.00161495,  0.7819493 ], dtype=float32), 
'world_orient': array([ 9.9960482e-01, -2.7147911e-05, -2.8098553e-02,  8.7645662e-04],
      dtype=float32), 

'linear_velocities': array([-0.0587876 ,  0.00157661,  0.00219552], dtype=float32), 
'angular_velocities': array([-0.00217248, -0.08555341, -0.0009418 ], dtype=float32)}, 
'stiffness': array([500., 500.,  50.,  50., 500., 500.,  50.,  50., 500., 500.,  50.,
        50., 500., 500.,  50.,  50., 500., 500.,  50.,  50., 500., 500.,
        50.,  50.,  50.,  50.], dtype=float32), 
'dampings': array([50., 50.,  2.,  2., 50., 50.,  2.,  2., 50., 50.,  2.,  2., 50.,
       50.,  2.,  2., 50., 50.,  2.,  2., 50., 50.,  2.,  2.,  2.,  2.],
      dtype=float32)}
"""
rospy.init_node('kuavo_isaac_sim_controller', anonymous=True)

# 获取参数
isaac_robot_version = rospy.get_param('/isaac_robot_version', default=45)
use_camera_ros_topic_flag = rospy.get_param('/use_camera_ros_topic_flag', default=0)
use_point_cloud_flag = rospy.get_param('/use_point_cloud_flag', default=0)
robot_scene_point_index = rospy.get_param('/robot_scene_point_index', default=1)
scene_index = rospy.get_param('/scene_index', default=3)

# 打印参数值
rospy.loginfo(f"Isaac Robot Version: {isaac_robot_version}")
rospy.loginfo(f"Use Camera ROS Topic Flag: {use_camera_ros_topic_flag}")
rospy.loginfo(f"Use Point Cloud Flag: {use_point_cloud_flag}")
rospy.loginfo(f"Scene Index: {scene_index}")
rospy.loginfo(f"Robot Scene Point Index: {robot_scene_point_index}")

class BipedCtrlClient(PersistentTcpClient):
    def get_command(self, msg):
        data_bin = json2bin(msg)
        return json.loads(self.send(data_bin).decode("ascii"))


class KuavoController:
    def __init__(self):
        self.msg = None
        self.client = None
        self.env = None
        self.debug_counter = 0  # 添加调试计数器
        
        # 初始化ROS节点（如果还没初始化）
        if not rospy.get_node_uri():
            rospy.init_node('kuavo_isaac_sim_controller', anonymous=True)
        
        # 创建ROS发布器
        self.tau_pub = rospy.Publisher('/kuavo_isaac_sim/joint_cmd', JointState, queue_size=10)
        
        # 创建关节名称列表
        self.joint_names = (
            # 腿部关节 - 左右交替
            ['leg_l1_joint', 'leg_r1_joint',
             'leg_l2_joint', 'leg_r2_joint',
             'leg_l3_joint', 'leg_r3_joint',
             'leg_l4_joint', 'leg_r4_joint',
             'leg_l5_joint', 'leg_r5_joint',
             'leg_l6_joint', 'leg_r6_joint',
             # 手臂关节 - 左右交替
             'zarm_l1_joint', 'zarm_r1_joint',
             'zarm_l2_joint', 'zarm_r2_joint',
             'zarm_l3_joint', 'zarm_r3_joint',
             'zarm_l4_joint', 'zarm_r4_joint',
             'zarm_l5_joint', 'zarm_r5_joint',
             'zarm_l6_joint', 'zarm_r6_joint',
             'zarm_l7_joint', 'zarm_r7_joint']
        )

    def publish_joint_tau(self, action):
        """发布关节数据，根据控制模式发布力矩或位置"""
        if action is None:
            return
        
        # 创建JointState消息
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self.joint_names
        
        # 初始化空列表，长度与关节数量相同
        effort_data = [0.0] * len(self.joint_names)
        position_data = [0.0] * len(self.joint_names)
        
        # 处理腿部数据（前12个关节）
        leg_values = action['legs']['joint_values']
        if action['legs']['ctrl_mode'] == 'effort':
            effort_data[:12] = leg_values
        else:
            position_data[:12] = leg_values
        
        # 处理手臂数据（后14个关节）
        arm_values = action['arms']['joint_values']
        if action['arms']['ctrl_mode'] == 'effort':
            effort_data[12:] = arm_values
        else:
            position_data[12:] = arm_values
        
        # 填充消息
        joint_state_msg.effort = effort_data
        joint_state_msg.position = position_data
        
        # 调试输出（可选）
        if self.debug_counter % 500 == 0:  # 每500次打印一次
            # rospy.loginfo(f"Arms control mode: {action['arms']['ctrl_mode']}")
            # rospy.loginfo(f"Legs control mode: {action['legs']['ctrl_mode']}")
            if action['arms']['ctrl_mode'] == 'position':
                # rospy.loginfo(f"Arms position values: {arm_values[:5]}...")  # 只打印前5个值
                pass
            else:
                # rospy.loginfo(f"Arms effort values: {arm_values[:5]}...")
                pass
        
        # 发布消息
        self.tau_pub.publish(joint_state_msg)

    def initialize_socket(self):
        try:
            self.client = BipedCtrlClient(ip="0.0.0.0", port=8800)
        except Exception as e:
            self.client = None
            raise ValueError("Please run C++ to open the socket first") from e

    def reconnect(self, env):
        print("waiting for server...\r")
        while self.client is None:
            try:
                self.initialize_socket()
            except Exception:  # pylint: disable=broad-exception-caught
                time.sleep(0.05)
            env.step()
        print("connected to server!")

    def get_cmd(self, obs, sensor_data=None, imu_data_sensor=None):
        """
        get the command from the controller.
        """
        obs_agent = copy.deepcopy(obs)
        # print("obs_agent: ", obs_agent)

        q_leg = obs_agent["joint_state"]["legs_positions"]    # 腿部关节角度
        dq_leg = obs_agent["joint_state"]["legs_velocities"]  # 腿部关节速度
        current_leg = obs_agent["joint_state"]["legs_applied_effort"]  # 腿部关节力矩

        q_arm = obs_agent["joint_state"]["arms_positions"]    # 手部关节角度 
        dq_arm = obs_agent["joint_state"]["arms_velocities"]  # 手部关节速度
        current_arm = obs_agent["joint_state"]["arms_applied_effort"]  # 手部关节力矩

        p_wb = obs_agent["body_state"]["world_pos"]           # 机器人在世界位置下的position
        quat_wb = obs_agent["body_state"]["world_orient"]     # 机器人在世界位置下的orientation
        v_wb = obs_agent["body_state"]["linear_velocities"]   # 机器人在世界位置下的线速度
        w_wb = obs_agent["body_state"]["angular_velocities"]  # 机器人在世界位置下的角速度

        # 获取IMU数据并转换为列表
        imu_data = imu_data_sensor["imu"]
        
        self.msg = {
            "q_leg": q_leg.tolist(),
            "dq_leg": dq_leg.tolist(),
            "q_arm": q_arm.tolist(),
            "dq_arm": dq_arm.tolist(),
            "current_leg": current_leg.tolist(),
            "current_arm": current_arm.tolist(),
            "p_wb": p_wb.tolist(),
            "quat_wb": quat_wb.tolist(),
            "v_wb": v_wb.tolist(),
            "w_wb": w_wb.tolist(),
            # 添加IMU数据
            "imu": {
                "time": float(imu_data["time"]),  # 仿真时间戳
                "physics_step": float(imu_data["physics_step"]), # 仿真步数 
                "linear_acceleration": imu_data["linear_acceleration"].tolist(), # IMU线性加速度
                "angular_velocity": imu_data["angular_velocity"].tolist(),       # IMU角速度
                "orientation": imu_data["orientation"].tolist()                  # IMU方向四元数
            }
        }
        # 从服务端当中获取tau力矩控制命令数据
        cmd = self.client.get_command(self.msg)
        
        # 发布力矩数据
        self.publish_joint_tau(cmd)
        
        return cmd


def main():
    env = Env(isaac_robot_version, use_camera_ros_topic_flag, use_point_cloud_flag, scene_index, robot_scene_point_index)
    # Call env.reset() to init simulation
    env.reset()
    kuavo = env.get_agent()
    sensor = env.get_sensor()
    print("=================")
    env.reset()

    print(kuavo.get_arm_idx())
    print(kuavo.get_leg_idx())
    
    # 腿部index
    print("leg_l1_joint: ", kuavo.get_joint_index("leg_l1_joint"))
    print("leg_l2_joint: ", kuavo.get_joint_index("leg_l2_joint"))
    print("leg_l3_joint: ", kuavo.get_joint_index("leg_l3_joint"))
    print("leg_l4_joint: ", kuavo.get_joint_index("leg_l4_joint"))
    print("leg_l5_joint: ", kuavo.get_joint_index("leg_l5_joint"))
    print("leg_l6_joint: ", kuavo.get_joint_index("leg_l6_joint"))
    print("leg_r1_joint: ", kuavo.get_joint_index("leg_r1_joint"))
    print("leg_r2_joint: ", kuavo.get_joint_index("leg_r2_joint"))
    print("leg_r3_joint: ", kuavo.get_joint_index("leg_r3_joint"))
    print("leg_r4_joint: ", kuavo.get_joint_index("leg_r4_joint"))
    print("leg_r5_joint: ", kuavo.get_joint_index("leg_r5_joint"))
    print("leg_r6_joint: ", kuavo.get_joint_index("leg_r6_joint"))
    
    # 手部index
    print("zarm_l1_joint: ", kuavo.get_joint_index("zarm_l1_joint"))
    print("zarm_l2_joint: ", kuavo.get_joint_index("zarm_l2_joint"))
    print("zarm_l3_joint: ", kuavo.get_joint_index("zarm_l3_joint"))
    print("zarm_l4_joint: ", kuavo.get_joint_index("zarm_l4_joint"))
    print("zarm_l5_joint: ", kuavo.get_joint_index("zarm_l5_joint"))
    print("zarm_l6_joint: ", kuavo.get_joint_index("zarm_l6_joint"))
    print("zarm_l7_joint: ", kuavo.get_joint_index("zarm_l7_joint"))
    print("zarm_r1_joint: ", kuavo.get_joint_index("zarm_r1_joint"))
    print("zarm_r2_joint: ", kuavo.get_joint_index("zarm_r2_joint"))
    print("zarm_r3_joint: ", kuavo.get_joint_index("zarm_r3_joint"))
    print("zarm_r4_joint: ", kuavo.get_joint_index("zarm_r4_joint"))
    print("zarm_r5_joint: ", kuavo.get_joint_index("zarm_r5_joint"))
    print("zarm_r6_joint: ", kuavo.get_joint_index("zarm_r6_joint"))
    print("zarm_r7_joint: ", kuavo.get_joint_index("zarm_r7_joint"))
    
    # # 头部index
    # print("zhead_1_joint: ", kuavo.get_joint_index("zhead_1_joint")) # 4 
    # print("zhead_2_joint: ", kuavo.get_joint_index("zhead_2_joint")) # 9

    # # 只获取总质量
    # total_mass = kuavo.get_total_mass()
    # 获取总质量并打印详细信息
    total_mass = kuavo.get_total_mass(verbose=True)
    print("Total mass: ", total_mass)
    
    action = None

    while True:
        # 机器人控制层step
        env.step()
        
if __name__ == "__main__":
    main()

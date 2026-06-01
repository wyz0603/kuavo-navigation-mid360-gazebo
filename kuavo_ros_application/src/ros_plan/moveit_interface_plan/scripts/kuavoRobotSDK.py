#!/usr/bin/env python
"""机器人基础控制类

本模块封装了机器人的基本控制功能，包括行走、手臂控制、姿态控制等。

功能描述：
- 设置机器人基本行走状态
- 控制手臂的运动和控制模式
- 控制机器人的姿态模式
"""
import rospy
import time

# robot control
from dynamic_biped.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from dynamic_biped.srv import srvChangePhases, srvChangePhasesRequest, srvChangePhasesResponse
from dynamic_biped.srv import srvchangeCtlMode, srvchangeCtlModeRequest, srvchangeCtlModeResponse  # 0 位置 1 速度 2 步数
from dynamic_biped.srv import srvClearPositionCMD, srvClearPositionCMDRequest, srvClearPositionCMDResponse
from dynamic_biped.srv import changeAMBACCtrlMode, changeAMBACCtrlModeRequest, changeAMBACCtrlModeResponse
from dynamic_biped.srv import srvClearPositionCMD, srvClearPositionCMDRequest, srvClearPositionCMDResponse
from dynamic_biped.srv import srvChangeJoller, srvChangeJollerRequest, srvChangeJollerResponse
from dynamic_biped.srv import controlEndHand, controlEndHandRequest, controlEndHandResponse

from dynamic_biped.msg import walkCommand       
from sensor_msgs.msg import JointState
# from dynamic_biped.msg import robotQVTau         # 全身关节的位置、速度、力矩，前四个为躯干
from dynamic_biped.msg import robotArmInfo
from dynamic_biped.msg import robotTorsoState    # 躯干的旋转角度、旋转速度、旋转加速度 / 质心的旋转角度、旋转速度、旋转加速度
from dynamic_biped.msg import robotPhase         # 机器人目前的主状态，子状态

class kuavo:
    """ 构造机器人基础类 """

    def __init__(self, name):
        self._name = name
        
        # 手臂关节数(默认为14个)
        self.arm_num = 14 

        # 默认的手臂归中数据
        self.arm_recenter_joint_list = [20,0,0,-30,0,0,0,20,0,0,-30,0,0,0]

        # 成员变量用于保存数据
        self.latest_walk_speed = None
        self.latest_QVTau = None
        self.latest_Torso_centroid_State = None
        self.latest_Phase = None
        self.latest_mainPhase = None
        self.latest_subPhase = None

        # msg Pub 
        self._walk_speed_pub = rospy.Publisher("/walkCommand", walkCommand, queue_size=10)
        self._arm_traj_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

        # msg Sub
        self._robotQVTau_sub = rospy.Subscriber("/robot_q_v_tau", robotArmInfo, self.get_robotQVTau_callback)
        self._robotTorsoState_sub = rospy.Subscriber("/robot_torso_state", robotTorsoState, self.get_robotTorsoState_callback)
        self._robotPhase_sub = rospy.Subscriber("/leju_robot_phase", robotPhase, self.get_robotPhase_callback)
        self._walk_speed_sub = rospy.Subscriber("/walkCommand", walkCommand, self.get_walk_speed_callback)

        # service client
        self._arm_ctrl_mode_client = rospy.ServiceProxy("/change_arm_ctrl_mode", changeArmCtrlMode)
        self._change_phases_client = rospy.ServiceProxy("/setPhase", srvChangePhases)
        self._robot_ctrl_mode_client = rospy.ServiceProxy("/change_ctl_mode", srvchangeCtlMode)
        self._robot_AMBAC_ctrl_mode_client = rospy.ServiceProxy("/change_AMBAC_ctrl_mode", changeAMBACCtrlMode)
        self._robot_clear_position_client = rospy.ServiceProxy("/clear_position_cmd", srvClearPositionCMD)
        self._robot_change_joller_client = rospy.ServiceProxy("/change_joller_position", srvChangeJoller)
        self._robot_controlEndHand_client = rospy.ServiceProxy("/control_end_hand", controlEndHand)

    """ ROS函数 """
    def get_walk_speed_callback(self, msg):
        self.latest_walk_speed = msg

    def get_robotQVTau_callback(self, msg):
        self.latest_QVTau = msg
    
    def get_robotTorsoState_callback(self, msg):
        self.latest_Torso_centroid_State = msg

    def get_robotPhase_callback(self, msg):
        self.latest_mainPhase = msg.mainPhase
        self.latest_subPhase = msg.subPhase

    def pub_kuavo_walk_step_command(self, ctl_mode:int, step_num:float, x:float, y:float, yaw:float):
        walk_speed_msg = walkCommand()

        walk_speed_msg.mode = ctl_mode    
        walk_speed_msg.values = [step_num, x, y, yaw]  

        self._walk_speed_pub.publish(walk_speed_msg)

    def pub_kuavo_walk_speed(self, ctl_mode:int, x:float, y:float, yaw:float):
        walk_speed_msg = walkCommand()

        walk_speed_msg.mode = ctl_mode    
        walk_speed_msg.values = [x, y, yaw]  

        self._walk_speed_pub.publish(walk_speed_msg)

    def pub_kuavo_arm_traj(self, traj_jointstate):
        arm_traj_msg = JointState()

        arm_traj_msg.position = traj_jointstate.position
        self._arm_traj_pub.publish(arm_traj_msg)

    def srvControl_end_hand(self, left_hand_position, right_hand_position)->bool:
        try:
            request = controlEndHandRequest()
            request.left_hand_position = left_hand_position
            request.right_hand_position = right_hand_position

            response = self._robot_controlEndHand_client(request)
            return response.result

        except rospy.ServiceException as e:
            rospy.logerr(f"srvControl_end_hand Service call failed: {e}")
            return False

    def srv_changeArmCtrlMode(self, control_mode:bool)->bool:
        try:
            request = changeArmCtrlModeRequest()
            request.control_mode = control_mode

            response = self._arm_ctrl_mode_client(request)

            return response.result

        except rospy.ServiceException as e:
            rospy.logerr(f"changeArmCtrlMode Service call failed: {e}")
            return False

    def srv_srvChangePhases(self, master_id:int, state_req:str, sub_state:str)->int:
        try:
            # 客户端发送request
            request = srvChangePhasesRequest()
            request.masterID = master_id
            request.stateReq = state_req
            request.subState = sub_state

            # 客户端接收response
            response = self._change_phases_client(request) 

            return response.stateRes

        except rospy.ServiceException as e:
            rospy.logerr(f"srvChangePhases Service call failed: {e}")
            return 0  
    
    def srv_ChangeJoller_call(self, _left_pos:int, _right_pos:int)->bool:
        try:
            # 客户端发送request
            request = srvChangeJollerRequest()
            request.l_pos = _left_pos
            request.r_pos = _right_pos

            # 客户端接收
            response = self._robot_change_joller_client(request)
            
            return response.result
        
        except rospy.ServiceException as e:
            rospy.logerr(f"srv_ChangeJoller_call Service call failed: {e}")
            return 0   
    
    def srv_ChangectlMode(self, _master_id:int, _control_mode:int)->int:
        try:
            # 客户端发送request
            request = srvchangeCtlModeRequest()
            request.masterID = _master_id
            request.control_mode = _control_mode

            # 客户端接收response
            response = self._robot_ctrl_mode_client(request) 

            return response.control_mode

        except rospy.ServiceException as e:
            rospy.logerr(f"srv_ChangectlMode Service call failed: {e}")
            return 0          
    
    """ 功能函数 """
    def get_QVTau(self):
        """ 获取全身关节的位置、速度、力矩，前四个为躯干 """
        if self.latest_QVTau is not None:
            return self.latest_QVTau
        else:
            rospy.logwarn("No QVTau data available.")

    def get_Torso_centroid_State(self):
        """ 获取躯干/质心的旋转角度、旋转速度、旋转加速度 """
        if self.latest_Torso_centroid_State is not None:
            return self.latest_Torso_centroid_State
        else:
            rospy.logwarn("No Torso_centroid_State data available.")

    def get_Phase(self):
        """ 获取机器人目前的主状态，子状态 """
        if self.latest_Phase is not None:
            return self.latest_Phase
        else:
            rospy.logwarn("No Phase data available.")

    def get_walk_speedCommand(self):
        """ 获取机器人行走command """
        if self.latest_walk_speed is not None:
            return self.latest_walk_speed
        else:
            rospy.logwarn("No walk speed data available.")

    def set_arm_traj_position(self, joint_positions:list):
        """ 设置机器人行走状态时的速度
        :param joint_positions: list 最终关节的位置
        """
        if len(joint_positions) == self.arm_num:
            arm_traj_msg = JointState()
            arm_traj_msg.position = joint_positions

            self.pub_kuavo_arm_traj(arm_traj_msg)
        else:
            rospy.logerr("Invalid number of joint positions provided.")
    
    def set_walk_speed(self, control_mode:int, v_x:float, v_y:float, v_yaw:float, v_step=0):
        """ 设置机器人行走状态时的速度
        :param control_mode: int # mode: 0->PositionCommand | 1->VelocityCommand | 2->stepCommand
        :param v_x: float, 
        :param v_y: float, 
        :param v_yaw: float,
        """
        if control_mode == 1:
            # 速度模式控制
            print("....VelocityCommand  ", " v_x : ", v_x , " v_y : ", v_y ," v_yaw : ", v_yaw)
            self.pub_kuavo_walk_speed(control_mode, v_x, v_y, v_yaw)
        elif control_mode == 2:
            # 步态模式控制 
            print("....stepCommand v_step: ", v_step, " v_x : ", v_x , " v_y : ", v_y ," v_yaw : ", v_yaw)
            self.pub_kuavo_walk_step_command(control_mode, v_step, v_x, v_y, v_yaw)
        
    
    def set_robot_Phases(self, p_master_id:int, p_state_req:str, p_sub_state:str)->int:
        """ 设置机器人此时的模式 
        :param master_id: uint8, masterID
        :param state_req: string, stateReq
        :param sub_state: string, subState
        :return: int16, stateRes
        """
        result = self.srv_srvChangePhases(p_master_id, p_state_req, p_sub_state)
        rospy.loginfo(f"Service call /setPhase call: {result}")
        return result

    def set_end_control(self, left_hand_position, right_hand_position):
        """ 设置机器人的夹爪的状态
        :param master_id: uint8, masterID
        :param state_req: string, stateReq
        :param sub_state: string, subState
        :return: int16, stateRes
        """
        result = self.srvControl_end_hand(left_hand_position, right_hand_position)
        # rospy.loginfo(f"Service call /control_end_hand call: {result}")
        return result

    def set_robot_joller_position(self, l_position:int, r_position:int)->bool:
        """ 设置末端机械爪的 左右位置
        :param l_position: int32, 左边机械爪的位置 
        :param r_position: int32, 右边机械爪的位置    
        """
        result = self.srv_ChangeJoller_call(l_position, r_position)
        rospy.loginfo(f"Service call /change_joller_position call: {result}")
        return result
    
    def set_robot_arm_ctl_mode(self, p_control_mode:bool)->bool:
        """ 切换手臂规划模式 
        :param control_mode: bool, 控制模式
        :return: bool, 服务调用结果 
        """
        result = self.srv_changeArmCtrlMode(p_control_mode)
        rospy.loginfo(f"Service call /change_arm_ctrl_mode call: {result}")
        return result
    
    def set_robot_arm_recenter(self)->bool:
        """切换手臂为归中模式
        """
        try:
            self.set_arm_traj_position(self.arm_recenter_joint_list)
            rospy.loginfo("set_robot_arm_recenter success!")
        except Exception as e:
            print(f"An error occurred: {e}")
            rospy.loginfo("set_robot_arm_recenter fail!")

    def set_robot_status_ctl_mode(self, p_master_id:int, _control_mode:str)->bool:
        """切换机器人行走时的控制模式(在行走状态开始前进行设置)
        0 - Position 位置控制模式
        1 - Velocity 速度控制模式
        2 - Step     步数控制模式
        """
        # init control cmd
        if _control_mode == "Position":
            control_cmd = 0 
        elif _control_mode == "Velocity":
            control_cmd = 1
        elif _control_mode == "Step":
            control_cmd = 2
        
        try:
            self.srv_ChangectlMode(p_master_id, control_cmd)
        except Exception as e:
            print(f"An error occurred: {e}")
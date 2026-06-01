import sys
import os
import time

current_path = os.path.dirname(os.path.abspath(__file__))
library_path = os.path.join(current_path, '../lib/python-can-3.3.4')
# my_path = os.path.join(current_path, '../lib/python-can-3.3.4/can/interfaces/bmcan')
# sys.path.append(my_path) 
sys.path.append(library_path) 
project_root = os.path.dirname(current_path)
sys.path.append(project_root) 
# print("--------------------------------------")
# print(f"library_path{library_path}")
# print(f"project_root{project_root}")
# current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
# new_library_path = "../lib/python-can-3.3.4/can/interfaces/bmcan"
# print(f"current_ld_library_path{current_ld_library_path}")
# os.environ["LD_LIBRARY_PATH"] = f"{library_path}:{current_ld_library_path}:{new_library_path}"
# temp = os.environ["LD_LIBRARY_PATH"]
# print(f"os.environ:{temp}")
sys.path.insert(0,library_path)
import can
from can.bus import BusState


class BMCANTools:

    def __init__(self):
        self.dev = None
        self.dev_info = {
            "dev_type": "bmcan",
            "dev_channel": 0,
            "bit_rate": 1000000,
            "data_rate": 1000000,
            "terminal_res": True,
            "timeout": 1,
        }
    def open_canbus(self):
        try:
            self.dev = can.interface.Bus(
                bustype=self.dev_info["dev_type"],
                channel=self.dev_info["dev_channel"],
                bitrate=self.dev_info["bit_rate"],
                data_bitrate=self.dev_info["data_rate"],
                tres=self.dev_info["terminal_res"],
            )
            if self.dev.state == BusState.ACTIVE:
                return True
            else:
                return False

        except Exception as exc:
            exc_msg = str(exc)
            # exc_msg = "try erroy"
        return exc_msg
    def close_canbus(self):
        try:
            self.dev.shutdown()
            return True

        except Exception as exc:
            exc_msg = str(exc)
        return exc_msg


class NiMServos:

    def __init__(self):
        try:
            print("初始化USB-CAN设备")
            self.cantool=BMCANTools()
        except Exception as exc:
            exc_msg = str(exc)
            print("初始化USB-CAN设备失败")
            return exc_msg
        self.CiA402OperateMode = {
            "PP" : 0x01,#轮廓位置模式(PP)
            "VM" : 0x02,#速度模式(VM)
            "PV" : 0x03,#轮廓速度模式(PV)
            "PT" : 0x04,#轮廓转矩模式(PT)
            "HM" : 0x06,#原点回归模式(HM)
            "IP" : 0x07,#插补模式(IP)
            "CSP" : 0x08,#循环同步位置模式(CSP)
            "CSV" : 0x09,#循环同步速度模式(CSV)
            "CST" : 0x0A,#循环同步转矩模式(CST)
        }
        self.ControlMode = {
            "CiA402Mode" : 0x00,            #/*!< CiA402 模式 */
            "NiMotionPositionMode" : 0x01,  #/*!< NiMotion 位置模式 */
            "NiMotionVelocityMode" : 0x02,  #/*!< NiMotion 速度模式 */
            "NiMotionTorqueMode"   : 0x03,  #/*!< NiMotion 转矩模式 */
            "NiMotionOpenloopMode" : 0x04   #/*!< NiMotion 开环模式 */
         }
        self.TriggerMotorMode = {
            "abs_pos_delay_update"      : 0x1F, #绝对位置运动,非立即更新
            "abs_pos_immediate_update"  : 0x3F, #绝对位置运动,立即更新
            "pos_delay_update"          : 0x5F, #相对位置运动,非立即更新
            "pos_immediate_update"      : 0x6F  #相对位子运动,立即更新
        }
        self.CiA402ParameterTypeDef = {
            'target_position'   : 0,       #/*!< 目标位置 */
            'target_speed'      : 0,       #/*!< 目标速度 */
            'target_torque'     : 0,       #/*!< 目标转矩 */
            'acceleration'      : 0,       #/*!< 加速度单位 */
            'acceleration_time' : 0,       #/*!< 加速度时间单位 */
            'deceleration'      : 0,       #/*!< 减速度单位 */
            'deceleration_time' : 0,       #/*!< 减速度时间单位 */
            'max_acceleration'  : 0,       #/*!< 加速度单位 */
            'max_deceleration'  : 0,       #/*!< 减速度单位 */
            'motion_profile_type' : 0,     #/*!< 轮廓斜坡类型 */
            'torque_ramp_type'  : 0,       #/*!< 转矩斜坡类型 */
            'torque_ramp'       : 0,       #/*!< 转矩斜坡 */
            'time_constant'     : 0,       #/*!< 时间常数 */
            'time_exponent'     : 0,       #/*!< 时间指数 */
            'velocity_window'   : 0,       #/*!< 速度到达阈值 */
            'velocity_window_time' : 0,    #/*!< 速度到达时间窗口 */
            'velocity_threshold'   : 0,    #/*!< 零度到达阈值 */
            'velocity_threshold_time': 0,  #/*!< 零度到达时间窗口  */
            'max_torque'   : 0,            #/*!< 最大转矩 */
            'max_current'  : 0,            #/*!< 最大电流 */
            'max_current_dur_time'   : 0,  #/*!< 最大电流持续时间 */
        }

    def sendCANopenMessage(self,cob_id,command_specifier,index,sub_index,data):
        '''
        * @brief Send CANopen Message
        * @param cob_id 通信对象编号
        * @param command_specifier 命令符
        * @param index 索引
        * @param sub_index 子索引
        * @param data 数据
        * @note
        '''
        tx_msg = can.Message(
            arbitration_id=cob_id,
            is_extended_id=False,
            dlc=0x08,
            data = [
                command_specifier,
                index & 0xFF,
                (index >> 8) & 0xFF,
                sub_index,
                data & 0xFF,
                (data >> 8) & 0xFF,
                (data >>16) & 0xFF,
                (data >>24) & 0xFF,
            ]
        )
        self.cantool.dev.send(tx_msg,self.cantool.dev_info["timeout"])


    def receiveCANopenMessage(self):
        """
        * @brief Receive CANopen Message
        """
        msg = self.cantool.dev.recv(self.cantool.dev_info["timeout"])
        return msg


    def sendCiA402ControlWord(self,cob_id,control_word):
        """
        * @brief Send CiA402 Control Word
        * @param cob_id 通信对象编号
        * @param control_word 控制字
        * @note
        """
        self.sendCANopenMessage(cob_id,0x2B,0x6040,0x00,control_word)


    def prepareMotor(self,cob_id):
        """
        * @brief Prepare Motor
        * @param cob_id 通信对象编号
        * @note(
        """
        self.sendCiA402ControlWord(cob_id,0x06)

    
    def disableMotor(self,cob_id):
        """
        * @brief Disable Motor
        * @param cob_id 通信对象编号
        * @note
        """
        self.sendCiA402ControlWord(cob_id,0x07)
    

    def enableMotor(self,cob_id,operate_mode):
        """
        * @brief Disable Motor
        * @param cob_id 通信对象编号
        * @note
        """
        if self.CiA402OperateMode[operate_mode] in (
            0x01,#轮廓位置模式(PP)
            0x03,#轮廓速度模式(PV)
            0x04,#轮廓转矩模式(PT)
            0x06,#原点回归模式（HM）
            0x08,#循环同步位置模式（CSP）
            0x09,#循环同步速度模式
            0x0A,
            ):
            self.sendCiA402ControlWord(cob_id,0x0f)

        elif self.CiA402OperateMode[operate_mode] in (
             0x02,#速度模式(VM)
            ):
            self.sendCiA402ControlWord(cob_id,0x7f)

        elif self.CiA402OperateMode[operate_mode] in (
             0x07,#"IP"
            ):
            self.sendCiA402ControlWord(cob_id,0x1f)
    

    def enableMotorFirstTime(self,cob_id,operate_mode):
        """
        * @brief Enable Motor First Time
        * @param cob_id 通信对象编号
        * @note 首次使能电机,必须先依次给控制字6040h写入 0x6、0x7、0xF,后续断使能写入 0x7,使能则写入 0xF,
        """
        self.prepareMotor(cob_id)
        # time.sleep(0.01)
        self.disableMotor(cob_id)
        # time.sleep(0.01)
        self.enableMotor(cob_id,operate_mode)

    def configureMotorControlMode(self,cob_id,control_mode):
        """
        * @brief Configure Motor Control Mode
        * @param cob_id 通信对象编号
        * @param control_mode 控制模式， CiA402 模式和 NiMotion 模式（包括 NiMotion 位置模式,NiMotion 速度模式、NiMotion 转矩模式、NiMotion 开环模式）
        * @note 切换模式时，电机要处于失能状态
        """
        self.disableMotor(cob_id)
        # time.sleep(0.01)
        self.sendCANopenMessage(cob_id,0x2B,0x2002,0x01,control_mode)

    
    def configureMotorCiA402OperateMode(self,cob_id,operate_mode): 
        """
        * @brief  Configure Motor CiA402 Operate Mode
        * @param cob_id 通信对象编号
        * @param operate_mode CiA402运动模式（包括 轮廓位置模式(PP) 速度模式(VM) 轮廓速度模式(PV) 轮廓转矩模式(PT) 原点回归模式(HM) 插补模式(IP) 循环同步位置模式(CSP) 循环同步速度模式(CSV) 循环同步转矩模式(CST)）
        * @note 切换模式时，电机要处于失能状态
        """
        self.disableMotor(cob_id)
        self.sendCANopenMessage(cob_id,0x2F,0x6060,0x00,self.CiA402OperateMode[operate_mode])


    def configureMotorCiA402Parameter(self,cob_id, operate_mode):
        """
        * @brief Configure Motor CiA402 Parameter
        * @param cob_id 通信对象编号
        * @param operate_mode CiA402运动模式
        * @param parameter 参数结构体
        """
        if self.CiA402OperateMode[operate_mode] == 0x01:#轮廓位置模式(PP)
            self.sendCANopenMessage(cob_id, 0x23, 0x607A, 0x00, self.CiA402ParameterTypeDef["target_position"])
            self.sendCANopenMessage(cob_id, 0x23, 0x6081, 0x00, self.CiA402ParameterTypeDef["target_speed"])
            self.sendCANopenMessage(cob_id, 0x23, 0x6083, 0x00, self.CiA402ParameterTypeDef['acceleration'])
            self.sendCANopenMessage(cob_id, 0x23, 0x6084, 0x00, self.CiA402ParameterTypeDef['deceleration'])
            self.sendCANopenMessage(cob_id, 0x23, 0x60C5, 0x00, self.CiA402ParameterTypeDef['max_acceleration'])
            self.sendCANopenMessage(cob_id, 0x23, 0x60C6, 0x00, self.CiA402ParameterTypeDef['max_deceleration'])
        elif self.CiA402OperateMode[operate_mode] == 0x02:#速度模式(VM)
            self.sendCANopenMessage(cob_id, 0x23, 0x6048, 0x01, self.CiA402ParameterTypeDef['acceleration'])
            self.sendCANopenMessage(cob_id, 0x2B, 0x6048, 0x02, 0x01)            
            self.sendCANopenMessage(cob_id, 0x23, 0x6049, 0x01, self.CiA402ParameterTypeDef['deceleration'])    
            self.sendCANopenMessage(cob_id, 0x2B, 0x6049, 0x02, 0x01)        
        elif self.CiA402OperateMode[operate_mode] == 0x03:#轮廓速度模式(PV)
            self.sendCANopenMessage(cob_id, 0x23, 0x6083, 0x00, self.CiA402ParameterTypeDef['acceleration'])
            self.sendCANopenMessage(cob_id, 0x23, 0x6084, 0x00, self.CiA402ParameterTypeDef['deceleration'])
        elif self.CiA402OperateMode[operate_mode] == 0x04:#轮廓转矩模式(PT)
            self.sendCANopenMessage(cob_id, 0x23, 0x6088, 0x00, self.CiA402ParameterTypeDef['torque_ramp_type'])
            self.sendCANopenMessage(cob_id, 0x23, 0x6087, 0x00, self.CiA402ParameterTypeDef['torque_ramp'])   
        elif self.CiA402OperateMode[operate_mode] == 0x07:#轮廓转矩模式(IP)
            self.sendCANopenMessage(cob_id, 0x2F, 0x60C2, 0x01, self.CiA402ParameterTypeDef['time_constant'])
            self.sendCANopenMessage(cob_id, 0x2F, 0x60C2, 0x02, self.CiA402ParameterTypeDef['time_exponent'])

    
    def triggerMotorOperation(self,cob_id, tigger_mode):
        """
        * @brief Trigger Motor Operation
        * @param cob_id 通信对象编号
        * @param tigger_mode 电机触发模式(1.相对位置,非立即更新 2.相对位置,立即更新 3绝对位置,非立即更新 4绝对位置,立即更新)
        * @note 
        """
        self.sendCANopenMessage(cob_id, 0x2B, 0x6040, 0x00, self.TriggerMotorMode[tigger_mode])



    def initializeNiMServosMotor(self,device_id,control_mode="CiA402Mode",operate_mode = "PP",bSDO=True):
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 初始化CiA402ParameterTypeDef参数
        print(f"[Loginfo]:初始化设备{device_id}CiA402ParameterTypeDef")
        self.CiA402ParameterTypeDef["target_speed"] = 0
        self.CiA402ParameterTypeDef["acceleration"] = 500000
        self.CiA402ParameterTypeDef["deceleration"] = 5000000
        self.CiA402ParameterTypeDef["max_acceleration"] = 5000000
        self.CiA402ParameterTypeDef["max_deceleration"] = 5000000
        self.CiA402ParameterTypeDef["motion_profile_type"] = 0
        self.CiA402ParameterTypeDef["velocity_window"] = 500
        self.CiA402ParameterTypeDef["velocity_window_time"] = 2000
        self.CiA402ParameterTypeDef["velocity_threshold"] = 20000
        self.CiA402ParameterTypeDef["velocity_threshold_time"] = 1000
        self.CiA402ParameterTypeDef["max_torque"] = 2540
        self.CiA402ParameterTypeDef["max_current"] = 2340
        self.CiA402ParameterTypeDef["max_current_dur_time"] = 150
        # 配置模式
        print(f"[Loginfo]:初始化设备{cob_id}配置模式")
        self.configureMotorControlMode(cob_id,self.ControlMode[control_mode])
        self.configureMotorCiA402OperateMode(cob_id,operate_mode)
        # 设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,operate_mode)
        print(f"[Loginfo]:初始化设备{cob_id}设置控制字使能电机")


    def nimservos_PP_Init(self,device_id,postion,speed,acceleration,deceleration,tigger_mode="abs_pos_delay_update",bSDO=True):
        """
        * @brief NiMServos Motor PP Mode Init 轮廓位置模式(PP)初始化
        * @param device_id 通信对象编号(不包含功能码)
        * @param postion 初始化位置
        * @param speed 目标速度
        * @param acceleration 目标加速度
        * @param deceleration 目标减速度                        
        * @param tigger_mode 电机触发模式(1.相对位置,非立即更新 2.相对位置,立即更新 3绝对位置,非立即更新 4绝对位置,立即更新) 
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                        
        * @note 此模式主要用于点对点定位应用。此模式下，上位机给目标位置(绝对或者相对)、位置曲线的速
                度、加减速及减速度，伺服内部的轨迹发生器将根据设置生成目标位置曲线指令，驱动器内部完成
                位置控制，速度控制，转矩控制
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"PP")
        # 参数配置
        self.CiA402ParameterTypeDef["target_position"] = postion
        self.CiA402ParameterTypeDef["target_speed"] = speed
        self.CiA402ParameterTypeDef["acceleration"] = acceleration
        self.CiA402ParameterTypeDef["deceleration"] = deceleration
        self.configureMotorCiA402Parameter(cob_id,"PP")
        # 设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"PP")
        # 触发电机运行
        self.triggerMotorOperation(cob_id,tigger_mode)


    def nimservos_PP_UpdatePos(self,device_id,postion,tigger_mode="abs_pos_delay_update",bSDO=True):
        """
        * @brief NiMServos Motor PP Mode Init 轮廓位置模式(PP)下更新目标位置
        * @param device_id 通信对象编号(不包含功能码)
        * @param postion 目标位置                    
        * @param tigger_mode 电机触发模式(1.相对位置,非立即更新 2.相对位置,立即更新 3绝对位置,非立即更新 4绝对位置,立即更新) 
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                        
        * @note 此模式主要用于点对点定位应用。此模式下，上位机给目标位置(绝对或者相对)、位置曲线的速
                度、加减速及减速度，伺服内部的轨迹发生器将根据设置生成目标位置曲线指令，驱动器内部完成
                位置控制，速度控制，转矩控制
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 参数配置(更新目标位置)
        self.CiA402ParameterTypeDef["target_position"] = postion
        self.configureMotorCiA402Parameter(cob_id,"PP")
        # 设置控制字使能电机
        # self.prepareMotor(cob_id)
        # self.disableMotor(cob_id)
        self.enableMotor(cob_id,"PP")
        # 触发电机运行
        self.triggerMotorOperation(cob_id,tigger_mode)


    def nimservos_VM_Init(self,device_id,speed=0,acceleration=500,deceleration=500,bSDO=True):
        """
        * @brief NiMServos Motor VM Mode Init 速度模式初始化
        * @param device_id 通信对象编号(不包含功能码)
        * @param speed 初始化速度(默认为零)                   
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                      
        * @note 
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"VM")
        # 参数配置
        self.sendCANopenMessage(cob_id,0x2B,0x6042,0x00,speed)
        self.CiA402ParameterTypeDef["acceleration"] = acceleration
        self.CiA402ParameterTypeDef["deceleration"] = deceleration        
        self.configureMotorCiA402Parameter(cob_id,"VM")
        # 写控制字
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"VM")
        # 设置目标速度,电机运行
        self.sendCANopenMessage(cob_id,0x2B,0x6042,0x00,speed)


    def nimservos_VM_UpdateSpeed(self,device_id,speed,bSDO=True):
        """
        * @brief NiMServos Motor VM Mode 更新速度模式下的目标转速
        * @param device_id 通信对象编号(不包含功能码)
        * @param speed 目标速度                   
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                      
        * @note 
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 设置目标速度,电机运行(更新目标速度)
        self.sendCANopenMessage(cob_id,0x2B,0x6042,0x00,speed)


    def nimservos_PV_Init(self,device_id,acceleration,deceleration,speed=0,bSDO=True):
        """
        * @brief NiMServos Motor PV Mode Init轮廓速度模式(PV)初始化
        * @param device_id 通信对象编号(不包含功能码)
        * @param speed 目标速度(默认为零)
        * @param acceleration 目标加速度
        * @param deceleration 目标减速度                        
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 此模式下，上位控制器将目标速度、加速度、减速度发送给伺服驱动器，速度、转矩调节由伺
                服内部执行。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"PV")
        # 参数配置
        self.CiA402ParameterTypeDef["acceleration"] = acceleration
        self.CiA402ParameterTypeDef["acceleration"] = deceleration
        self.configureMotorCiA402Parameter(cob_id,"PV")
        # 设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"PV")
        # 设置目标速度,电机运行
        self.sendCANopenMessage(cob_id,0x23,0x60FF,00,speed)


    def nimservos_PV_UpdateSpeed(self,device_id,speed,bSDO=True):
        """
        * @brief NiMServos Motor PV Mode 更新轮廓速度模式(PV)下的目标转速
        * @param device_id 通信对象编号(不包含功能码)
        * @param speed 目标速度                     
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 此模式下，上位控制器将目标速度、加速度、减速度发送给伺服驱动器，速度、转矩调节由伺
                服内部执行。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 设置目标速度,电机运行(更新目标转速)
        self.sendCANopenMessage(cob_id,0x23,0x60FF,00,speed)


    def nimservos_PT_Init(self,device_id,torque_ramp,torque_ramp_type,torque=0,bSDO=True):
        """
        * @brief NiMServos Motor PT Mode Init 廓转矩模式(PT)初始化
        * @param device_id 通信对象编号(不包含功目标减速度码)
        * @param torque 目标力矩(默认为零)
        * @param torque_ramp 目标加速度
        * @param torque_ramp_type 斜坡类型 0-斜坡 2-无斜坡                        
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 此模式下，上位控制器将目标转矩 6071h、转矩斜坡常数 6087h 发送给伺服驱动器，转矩调节
                由伺服内部执行，当速度达到限幅值将进入调速阶段。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"PT")
        # 参数配置
        self.CiA402ParameterTypeDef["torque_ramp_type"] = torque_ramp_type
        self.CiA402ParameterTypeDef["torque_ramp"] = torque_ramp
        self.configureMotorCiA402Parameter(cob_id,"PT")
        # 设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"PT")
        # 设置目标速度,电机运行
        self.sendCANopenMessage(cob_id,0x2B,0x6071,00,torque)


    def nimservos_PT_Update(self,device_id,torque,bSDO=True):
        """
        * @brief NiMServos Motor PT Mode 更新廓转矩模式(PT)下的目标转扭矩
        * @param device_id 通信对象编号(不包含功目标减速度码)
        * @param torque 目标扭矩                       
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 此模式下，上位控制器将目标转矩 6071h、转矩斜坡常数 6087h 发送给伺服驱动器，转矩调节
                由伺服内部执行，当速度达到限幅值将进入调速阶段。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 设置目标速度,电机运行
        self.sendCANopenMessage(cob_id,0x2B,0x6071,00,torque)


    def nimservos_HM(self,device_id,valve_switch_speed,origin_signal_speed,home_acc,bSDO=True):
        """
        * @brief NiMServos Motor HM Mode 原点回归模式(HM)
        * @param device_id 通信对象编号(不包含功目标减速度码)
        * @param valve_switch_speed 限位开关速度
        * @param origin_signal_speed 原点信号速度
        * @param home_acc 回零加速度                 
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 原点回归模式是用于从目前的位置移动到设备的原点位置。在运动过程中，最大加速度，最小
                减速度，最大速度，最小速度等都考虑在内。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"HM")
        # 设置端子
        self.sendCANopenMessage(cob_id,0x2B,0x2003,0x03,0x0f)
        self.sendCANopenMessage(cob_id,0x2B,0x2003,0x04,0x00)
        # 回零方式
        self.sendCANopenMessage(cob_id,0x2F,0x6098,0x00,0x11)
        # 设置寻找限位开关速度和寻找原点信号速度
        self.sendCANopenMessage(cob_id,0x23,0x6099,0x01,valve_switch_speed)
        self.sendCANopenMessage(cob_id,0x23,0x6099,0x01,origin_signal_speed)
        # 设置回零加速度
        self.sendCANopenMessage(cob_id,0x23,0x609A,0x00,home_acc)
        # 设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"HM")
        # 触发电机运行(电机使能后)


    def nimservos_IP(self,device_id,position,time_constant,time_exponent,bSDO=True):
        """
        * @brief NiMServos Motor IP Mode 插补模式(IP)
        * @param device_id 通信对象编号(不包含功能码)
        * @param position 斜坡类型 0-斜坡 2-无斜坡 
        * @param time_constant 插补时间常数
        * @param time_exponent 插补时间指数
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                        
        * @note 插值位置模式用于同步多个轴。为此，高级控制器执行斜坡和路径计算，并将轴在特定时间所处
                的相应需求位置传递给控制器。控制器在这些中间位置点之间插入。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"IP")
        # 设置插补时间
        self.CiA402ParameterTypeDef["time_constant"] = time_constant
        self.CiA402ParameterTypeDef["time_exponent"] = time_exponent
        self.configureMotorCiA402Parameter(cob_id,"IP")
        # 设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"IP")
        # 触发电机运行(电机使能后)
        # 上位机按照插补周期写插补位置 60C1h:01h(只支持绝对位置指令，用户单位)
        self.sendCANopenMessage(cob_id,0x23,0x60C1,0x01,position)


    def nimservos_CSP_Init(self,device_id,position,bSDO=True):
        """
        * @brief NiMServos Motor CSP Mode 循环同步位置模式(CSP)初始化
        * @param device_id 通信对象编号(不包含功能码)
        * @param position 初始化位置                  
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 在该模式下，将以固定的时间间隔(以下称为"循环")通过现场总线向控制器发送绝对的位置预设
                值。此时，控制器不再计算斜坡，它仅遵循预设值。目标位置通过 PDO 进行传输，控制器会立即对
                其做出反应。Controlword 中的位 4 无需设定（不同于轮廓位置模式）。目标预设值是绝对值，因此
                与每个循环被发送的次数无关。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"CSP")
        self.sendCANopenMessage(cob_id, 0x23, 0x6080, 0x00, 4294967)
        #设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"CSP")
        # 按照同步周期发送目标位置 607A(只支持绝对位置指令，用户单位)
        self.sendCANopenMessage(cob_id,0x23,0x607A,0x00,position)


    def nimservos_CSP_UpdatePos(self,device_id,position,bSDO=True):
        """
        * @brief NiMServos Motor CSP Mode 更新循环同步位置模式(CSP)下的目标位置
        * @param device_id 通信对象编号(不包含功能码)
        * @param position 目标位置                  
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 在该模式下，将以固定的时间间隔(以下称为"循环")通过现场总线向控制器发送绝对的位置预设
                值。此时，控制器不再计算斜坡，它仅遵循预设值。目标位置通过 PDO 进行传输，控制器会立即对
                其做出反应。Controlword 中的位 4 无需设定（不同于轮廓位置模式）。目标预设值是绝对值，因此
                与每个循环被发送的次数无关。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 按照同步周期发送目标位置 607A(只支持绝对位置指令，用户单位)
        self.sendCANopenMessage(cob_id,0x23,0x607A,0x00,position)



    def nimservos_CSV_Init(self,device_id,speed=0,bSDO=True):
        """
        * @brief NiMServos Motor CSV Mode 循环同步速度置模式(CSV)初始化
        * @param device_id 通信对象编号(不包含功能码)
        * @param speed 初始化速度 (用户单位/s)            
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 周期同步速度模式下，上位控制器将计算好的目标速度 60FFh周期性同步的发送给伺服驱动器,
                速度、转矩调节由伺服内部执行。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"CSV")
        # 设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"CSV")
        # 上位机按照同步周期发送目标速度 60FFh (用户单位/s)
        self.sendCANopenMessage(cob_id,0x23,0x60FF,0x00,speed)


    def nimservos_CSV_UpdateSpeed(self,device_id,speed,bSDO=True):
        """
        * @brief NiMServos Motor CSV Mode 循环同步速度置模式(CSV)
        * @param device_id 通信对象编号(不包含功能码)
        * @param speed 目标速度 (用户单位/s)            
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                       
        * @note 周期同步速度模式下，上位控制器将计算好的目标速度 60FFh周期性同步的发送给伺服驱动器,
                速度、转矩调节由伺服内部执行。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 上位机按照同步周期发送目标速度 60FFh (用户单位/s)
        self.sendCANopenMessage(cob_id,0x23,0x60FF,0x00,speed)


    def nimservos_CST_Init(self,device_id,torque=0,bSDO=True):
        """
        * @brief NiMServos Motor CST Mode 循环同步力矩置模式(CST)初始化
        * @param device_id 通信对象编号(不包含功能码)
        * @param torque 初始化力矩(默认为零) 6071h (单位 0.1%)                     
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                        
        * @note 此模式下，上位控制器将计算好的目标转矩 6071h 周期性同步的发送给伺服驱动器，转矩调节
                由伺服内部执行。当速度达到限幅值后将进入调速阶段。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 配置模式
        self.configureMotorControlMode(cob_id,self.ControlMode["CiA402Mode"])
        self.configureMotorCiA402OperateMode(cob_id,"CST")
        #设置控制字使能电机
        self.prepareMotor(cob_id)
        self.disableMotor(cob_id)
        self.enableMotor(cob_id,"CST")
        # 上位机按照同步周期发送目标转矩 6071h (单位 0.1%)
        self.sendCANopenMessage(cob_id,0x2B,0x6071,0x00,torque) 


    def nimservos_CST_UpdateTorque(self,device_id,torque,bSDO=True):
        """
        * @brief NiMServos Motor CST Mode 更新循环同步力矩置模式(CST)下的目标转矩
        * @param device_id 通信对象编号(不包含功能码)
        * @param torque 目标转矩 6071h (单位 0.1%)                     
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯                        
        * @note 此模式下，上位控制器将计算好的目标转矩 6071h 周期性同步的发送给伺服驱动器，转矩调节
                由伺服内部执行。当速度达到限幅值后将进入调速阶段。
        """
        # 设置功能码
        if bSDO == True :
            cob_id = 0x600 + device_id
        else:
            pass
        # 上位机按照同步周期发送目标转矩 6071h (单位 0.1%)(更新目标转矩)
        self.sendCANopenMessage(cob_id,0x2B,0x6071,0x00,torque)


    def nimservos_ConfigureDX1(self,device_id,bDI=True,bitpositive=True,bSDO=True):
        """
        * @brief NiMServos Motor Configure DX1
        * @param device_id 通信对象编号(不包含功能码)
        * @param bDI 输入输出模式选择 True:将DX1设置为输入模式,False:将DX1设置为输出模式
        * @param bitpositive 低/高位有效位 True:高位有效 False:低位有效
        * @param bSDO SDO/PDO通讯  True:使用SDO通讯 False:使用PDO通讯
        """ 
        # 设置功能码
        if bSDO == True :
            cob_id = device_id + 0x600
        else:
            pass
        # 设置输入/输出模式
        if bDI == True:
            self.sendCANopenMessage(cob_id, 0x2B, 0x2004, 0x01, 0x00)
        else:
            self.sendCANopenMessage(cob_id, 0x2B, 0x2004, 0x01, 0x01)
        rx_msg = self.cantool.dev.recv(self.cantool.dev_info["timeout"])
        print(rx_msg)
        # 设置有效电平
        if bitpositive == True :
            self.sendCANopenMessage(cob_id, 0x2B, 0x2004, 0x02, 0x01)
        else:
            self.sendCANopenMessage(cob_id, 0x2B, 0x2004, 0x02, 0x00)
        rx_msg = self.cantool.dev.recv(self.cantool.dev_info["timeout"])
        print(rx_msg)


    def nimservos_SetDX1_Out(self,device_id,positive=False,bSDO=True):
        """
        * @brief NiMServos Motor Set DX1 Out 设置DX1输出高低电平
        * @param device_id 通讯对象编号(不包含功能码)
        * @param positive 高低电平输出 True:输出高电平 False: 输出低电平
        * @param bSDO SDO/PDO通讯 True:使用SDO通讯 Fasle: 使用PDO通讯
        """
        # 设置功能码
        if bSDO == True:
            cob_id = device_id + 0x600
        else:
            pass
        # 设置DX1为输出模式,并且设置高电平有效 
        self.nimservos_ConfigureDX1(cob_id, bDI=False,bitpositive=True)
        # 输出电平
        if positive == True :
            self.sendCANopenMessage(cob_id, 0x2B, 0x2031, 0x02, 0x01)
        else:
            self.sendCANopenMessage(cob_id, 0x2B, 0x2031, 0x02, 0x00)

    def nimservos_readSpeed(self, device_id, bSDO=True):
        """
        * @brief NiMservos Read Speed
        * @param device_id 设备id号(不包含功能码)
        * @param bSDO 布尔值 bSDO = true使用sdo通信
        * @note 
        """
        # 设置功能码
        if bSDO == True:
            cob_id = device_id + 0x600
        else:
            return
        # 发起读取速度请求
        self.sendCANopenMessage(cob_id, 0x40, 0x200B, 0x02, 0x00)
        # 监听应答
        rx_msg = self.receiveCANopenMessage()
        # print(f"[MESSAGE]: {rx_msg}")
        if rx_msg != None:
            if (rx_msg.arbitration_id == 0x580+device_id and rx_msg.data[0] != 0x60):
                # print(f"[MESSAGE]:节点{device_id}接受到SDO服务器应答")
                if (rx_msg.data[0] == 0x80):
                    print(f"[ERROR]:SDO应答异常响应")
                elif(rx_msg.data[1]+(rx_msg.data[2]<<8) == 0x200B and rx_msg.data[3] == 0x02):
                    data = (rx_msg.data[4]) + (rx_msg.data[5]<<8) + (rx_msg.data[6]<<16) + (rx_msg.data[7]<<24)
                    msg = dict()
                    msg["device_id"] = device_id
                    msg["velocity"] = data
                    return msg
        return None


    def nimservos_readAbsolutePos(self, device_id, bSDO=True):
        """
        * @brief NiMservos Read 绝对编码器角度 
        * @param device_id 设备id号(不包含功能码)
        * @param bSDO 布尔值 bSDO = true使用sdo通信
        * @note 默认一圈 编码器增量10000
        """
        # 设置功能码
        if bSDO == True:
            cob_id = device_id + 0x600
        else:
            pass
        # 发起读取绝对位置请求
        self.sendCANopenMessage(cob_id, 0x40, 0x6063,  0x00, 0x00)
        # 监听应答
        rx_msg = self.receiveCANopenMessage()
        # print(f"[MESSAGE]: {rx_msg}")
        #
        if rx_msg != None:
            if (rx_msg.arbitration_id == 0x580+device_id and rx_msg.data[0] != 0x60):
            # print(f"[MEESSAGE]:节点{device_id}接受到SDO服务器应答")
                if (rx_msg.data[0] == 0x80):
                    print(f"[ERROR]:SDO应答异常响应")
                elif(rx_msg.data[1]+(rx_msg.data[2]<<8) == 0x6063 and rx_msg.data[3] == 0x00):
                    data = (rx_msg.data[4]) + (rx_msg.data[5]<<8) + (rx_msg.data[6]<<16) + (rx_msg.data[7]<<24)
                    msg = dict()
                    msg["device_id"] = device_id
                    msg["position"] = data
                    return msg
         
        return None
    
    def nimservos_setDeviceID(self,device_id, set_id, bSDO=True):
        """
        * @brief NiMServos Set Device ID 更改设备id
        * @param device_id 设备id号(不包含功能码)
        * @param set_id 要更换的设备id号(不包含功能码)
        * @note 在PMMP60电机更改出现更改失败的问题, 在PMM60电机中更改成功
        """
        # 设置功能码
        if bSDO == True:
            cob_id = device_id + 0x600
        else:
            pass
        # 设置控制字使能电机
        self.disableMotor(cob_id)
        # 设置node id
        self.sendCANopenMessage(cob_id, 0x2B, 0x200C, 0x02, set_id)
        # 设置控制字电机准备机
        self.prepareMotor(cob_id)
        # 保存参数
        self.sendCANopenMessage(cob_id, 0x23, 0x1010, 0x01, 0x65766173)#保存到用户区

         
        
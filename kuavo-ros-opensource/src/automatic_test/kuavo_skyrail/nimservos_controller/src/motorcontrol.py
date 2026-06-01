#!/usr/bin/python3
import rospy 
import time
import sys
import os
from std_msgs.msg import String
current_path = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_path)
sys.path.append(project_root)
from drivers.src.nimservosSDK import NiMServos
from nimservos_controller.msg import motor_cmd , motor_state
import threading

BEGIN_LINEAR_X = 200000000
BEGIN_LINEAR_Y = 200000000

class NimMotor:
    def __init__(
            self,
            id = 0,
            controlmode = None,
            targetposition = 0,
        ) -> None:

        # 电机基础信息
        self.device_id = id 

        # 电机控制模式
        self.control_mode = controlmode

        # 电机的目标控制信息
        self.target_velocity= 0
        self.target_position = targetposition
        
        # 电机的传感器信息
        self.sensor_velocity = 0
        self.sensor_position = 0
        self.sensor_temp = 0

        # 电机状态信息
        self.state = None 


class NimMotorNode(NimMotor):

    def __init__(self):

        # 创建电机
        self.motor_left = NimMotor(id=1,controlmode="CSV",targetposition=BEGIN_LINEAR_X)
        self.motor_right= NimMotor(id=2,controlmode="CSV",targetposition=BEGIN_LINEAR_X)
        self.motor_car  = NimMotor(id=3,controlmode="CSV",targetposition=BEGIN_LINEAR_Y)

        self.motor_list = []
        self.motor_list.append(self.motor_left)
        self.motor_list.append(self.motor_right)
        self.motor_list.append(self.motor_car)

        # 创建NimServos Tool
        self.nimservos_tool = NiMServos()

        # 创建节点
        rospy.init_node('NimMotorNode', anonymous=True)

        # 创建电机状态信息发布者
        self.publisher = rospy.Publisher("nimmotor_state",motor_state,queue_size=10)
        
        # 创建电机控制信息订阅者
        self.subscriber = rospy.Subscriber("nimmotor_control",motor_cmd,self.recv_callback,queue_size=10)

        # 创建一个定时器
        self.timer = rospy.Timer(rospy.Duration(1),self.timer_callback)

        # 创建线程更新电机状态
        self.motor_updateVel_thread = threading.Thread(target=self.motor_updateVel)
        self.motor_updatePos_thread = threading.Thread(target=self.motor_updatePos)
    

    def recv_callback(self,msg):

        rospy.loginfo(f"Received Cmd message:{msg}")

        # self.motor_left.target_position = msg.linear_x + self.motor_left.target_position
        # self.motor_right.target_position = msg.linear_x + self.motor_right.target_position
        # self.motor_car.target_position = msg.linear_y + self.motor_car.target_position

        self.motor_left.target_velocity = msg.linear_y 
        self.motor_right.target_velocity = msg.linear_y
        self.motor_car.target_velocity = msg.linear_x
        

    def timer_callback(self,event):

        rospy.logdebug("nimmotor node pubish test")
        for motor in self.motor_list:
            msg = motor_state()
            # # 电机基本参数
            msg.device_id  = motor.device_id
            # 电机控制模式
            msg.control_mode = motor.control_mode
            # 电机状态参数
            msg.state = "None"
            # 电机传感器参数
            msg.sensor_position = motor.sensor_position 
            msg.sensor_velocity = motor.sensor_velocity
            msg.sensor_temp     = motor.sensor_temp
            self.publisher.publish(msg)

    
    def motor_updateVel(self,t=0.01):
        
        while not rospy.is_shutdown():
            self.nimservos_tool.receiveCANopenMessage()
            time.sleep(0.001)


    def motor_updatePos(self,t=0.01):

        while not rospy.is_shutdown():
            for i in range(1,4):
                data = self.nimservos_tool.nimservos_readAbsolutePos(i)
                if data != None:
                    rospy.loginfo(data)
                    if data["device_id"] == 1:
                        self.motor_left.sensor_position = data["position"]
                    elif data["device_id"] == 2:
                        self.motor_right.sensor_position = data["position"]
                    elif data["device_id"] == 3:
                        self.motor_car.sensor_position = data["position"]
                else :
                    pass
            for i in range(1,4):
                data = self.nimservos_tool.nimservos_readSpeed(i)
                if data != None:
                    rospy.loginfo(data)
                    if (-2147483648 <= data["velocity"] and data["velocity"] <= 2147483647):
                        if data["device_id"] == 1:
                            self.motor_left.sensor_velocity = data["velocity"] 
                        elif data["device_id"] == 2:
                            self.motor_right.sensor_velocity = data["velocity"]
                        elif data["device_id"] == 3:
                            self.motor_car.sensor_velocity = data["velocity"]
                    else :
                        pass

            time.sleep(t)



    def run(self):

        # 开启前先关闭usb-can
        self.nimservos_tool.cantool.close_canbus()
        # 开启usb-can
        can_open_status = self.nimservos_tool.cantool.open_canbus()
        rospy.loginfo(can_open_status)
        if can_open_status:

            rospy.loginfo("can open successes")
            # nimservosMotor初始化
            for motor in self.motor_list:
                self.nimservos_tool.initializeNiMServosMotor(motor.device_id)

            # 初始化为CSP模式
            # for motor in self.motor_list:
            #     self.nimservos_tool.nimservos_CSP_Init(
            #         device_id=motor.device_id,
            #         position=motor.target_position,
            #     )

            # 初始化为pp模式
            # self.nimservos_tool.nimservos_PP_Init(        
            #     device_id=1,
            #     postion=BEGIN_LINEAR_X,
            #     speed=80000,
            #     acceleration=40000,
            #     deceleration=40000
            # )

            # 初始化为VM模式
            # for motor in self.motor_list:
            #     self.nimservos_tool.nimservos_VM_Init(
            #         device_id=motor.device_id,
            #         speed=motor.target_velocity,
            #     )

            # 初始化为PV模式
            # for motor in self.motor_list:
            #     self.nimservos_tool.nimservos_PV_Init(
            #         device_id=motor.device_id,
            #         speed=motor.target_velocity,
            #         acceleration=500,
            #         deceleration=500
            #     )
            
            # 初始化为CSV模式
            for motor in self.motor_list:
                self.nimservos_tool.nimservos_CSV_Init(
                    device_id=motor.device_id,
                    speed=motor.target_velocity,                   
                )
            # 开启速度更新线程
            # self.motor_updateVel_thread.start()
            # 开启位置更新线程  
            # self.motor_updatePos_thread.start()

            while not rospy.is_shutdown():
                self.nimservos_tool.nimservos_CSV_UpdateSpeed(3, -self.motor_car.target_velocity)
                self.nimservos_tool.nimservos_CSV_UpdateSpeed(2, -self.motor_right.target_velocity)
                self.nimservos_tool.nimservos_CSV_UpdateSpeed(1, -self.motor_left.target_velocity)
                
                # for motor in self.motor_list:
                #     self.nimservos_tool.nimservos_CSV_UpdateSpeed(
                #         device_id=motor.device_id
                #         speed=-motor.target_velocitys
                #     )

                # string_ = self.nimservos_tool.receiveCANopenMessage()
                # print(string_)
                time.sleep(0.00001) 

            # 程序结束关闭电机 
            for motor in self.motor_list:
                self.nimservos_tool.disableMotor(0x600+motor.device_id)

            # 关闭can转usb       
            self.nimservos_tool.cantool.close_canbus()




if __name__ == '__main__':

    try:
        node  = NimMotorNode()
        node.run()
    except rospy.ROSException:
        pass
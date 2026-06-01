#!/usr/bin/python3
import rospy
import math
import time
import os
import sys
import tty
from nlink_parser.msg import LinktrackNodeframe2
from nlink_parser.msg import LinktrackAnchorframe0
from nimservos_controller.msg import motor_cmd
from std_msgs.msg  import String
from pynput.keyboard import Key,Listener
from pid import PIDController
from user_pkg.msg import piddebug, uwbdebug
import threading
import tkinter as tk
import signal 

current_path = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_path)
sys.path.append(project_root)

MOVE_SPEED = 30000

class UWBDataTypedef(): 

    def __init__(self,id):
        # 标签id 
        self.id = id 
        # 标签上一时刻坐标
        self.uwb_last_x = 0
        self.uwb_last_y = 0
        # 标签当前时刻坐标
        self.uwb_cur_x = 0
        self.uwb_cur_y = 0

        self.uwb_move_x = 0
        self.uwb_move_y = 0
        self.uwb_on_x   = 0
        self.uwb_on_y   = 0

        # 标签坐标列表
        self.uwb_poslists = [[],[]]
        self.uwb_poslist_length = 25
        # 标签目标坐标
        # self.uwb_target_x = 0
        # self.uwb_target_y = 0
    
        
class UserNode():

    def __init__(self):

        # 初始化节点
        rospy.init_node(name="user",anonymous=True)

        # 初始化轨道小车标签和机器人小车标签
        self.uwb_car = UWBDataTypedef(4)
        self.uwb_robot = UWBDataTypedef(5)

        # uwb订阅者
        # self.uwb_sub = rospy.Subscriber("/nlink_linktrack_nodeframe2", LinktrackNodeframe2, self.uwb_callback)
        self.uwb_sub = rospy.Subscriber("/nlink_linktrack_anchorframe0", LinktrackAnchorframe0, self.uwb_callback)

        # 创建uwb调试曲线发布者
        self.uwbdebug_pub = rospy.Publisher("/uwb_debug", uwbdebug, queue_size=10)
        
        # 天轨小车和机器人标签x,y方向的差值
        self.move_x_distance = 0# 小车方向移动距离
        self.move_y_distance = 0# 横梁方向移动距离

        # 创建pid控制器
        self.pidcontroller_x = PIDController()
        self.pidcontroller_y = PIDController()

        # 创建pid调试曲线发布者
        self.piddebug_pub = rospy.Publisher("/pid_debug",piddebug, queue_size=10)

        # 创建nimservos电机控制指令发布者
        self.nimservos_pub = rospy.Publisher("/nimmotor_control", motor_cmd, queue_size=10)

        # 创建键盘UI控制小窗口
        self.control_mode = "键盘控制模式"

        self.root = tk.Tk()
        self.master = self.root
        self.master.title("Key Listener")

        self.control_mode_label = tk.Label(self.root, text="键盘控制模式", font=("Helvetica", 16))
        self.control_mode_label.pack(pady=20)

        self.control_tip_label = tk.Label(self.root, text="WSAD 控制天轨前后左右移动", font=("Helvetica", 16))
        self.control_tip_label.pack(pady=20)

        self.globel_label = tk.Label(self.root, text="模式切换:o->键盘控制  p->uwb控制", font=("Helvetica", 10))
        self.globel_label.pack(pady=20)

        self.key_status_label = tk.Label(self.root, text="No key pressed", font=("Helvetica", 14))
        self.key_status_label.pack(pady=20)

        self.master.bind("<KeyPress>", self.on_key_press)
        self.master.bind("<KeyRelease>", self.on_key_release)

        # 创建线程
        self.nimservos_cmdpub_thread = threading.Thread(target=self.nimservor_cmdpub)
        

    def uwb_callback(self, data):

        # rospy.loginfo(data)
        # rospy.loginfo(data.nodes[0].id)
        for node in data.nodes:
            if node.id == self.uwb_car.id:
                # rospy.loginfo(node)
                tmp_x = node.pos_3d[0]
                tmp_y = node.pos_3d[1]

                self.uwb_car.uwb_cur_y = tmp_y
                self.uwb_car.uwb_cur_x = tmp_x
                # print("y\r\n",tmp_y)
                # print("x\r\n",tmp_x)

                # print("y\r\n",self.uwb_car.uwb_cur_x)
                # print("x\r\n",self.uwb_car.uwb_cur_x)
                # 滑动平均滤波
                # self.uwb_car.uwb_poslists[0].insert(0,tmp_x)
                # self.uwb_car.uwb_poslists[1].insert(0,tmp_y)

                # data_count = len(self.uwb_car.uwb_poslists[0]) 

                # if data_count > self.uwb_car.uwb_poslist_length:
                #     self.uwb_car.uwb_poslists[0].pop()
                #     self.uwb_car.uwb_poslists[1].pop()
                #     self.uwb_car.uwb_cur_x = sum(self.uwb_car.uwb_poslists[0])/self.uwb_car.uwb_poslist_length
                #     self.uwb_car.uwb_cur_y = sum(self.uwb_car.uwb_poslists[1])/self.uwb_car.uwb_poslist_length
                # else:
                #     self.uwb_car.uwb_cur_x = sum(self.uwb_car.uwb_poslists[0])/ data_count
                #     self.uwb_car.uwb_cur_y = sum(self.uwb_car.uwb_poslists[1])/ data_count
                # # 限幅
                # # if abs(self.uwb_car.uwb_cur_y-self.uwb_car.uwb_last_y)<0.02:
                # #     self.uwb_car.uwb_cur_y = self.uwb_car.uwb_last_y
                if abs(self.uwb_car.uwb_move_y - self.uwb_car.uwb_cur_y) > 0.5:
                    if (self.uwb_car.uwb_cur_y >= self.uwb_car.uwb_move_y):
                        self.uwb_car.uwb_on_y = 1
                    else:
                        self.uwb_car.uwb_on_y = 2
                    print("y 触发前",self.uwb_car.uwb_move_y)
                    self.uwb_car.uwb_move_y = self.uwb_car.uwb_cur_y
                    print("y 触发后",self.uwb_car.uwb_move_y)
                    self.uwb_car.uwb_on_y = 1
                if abs(self.uwb_car.uwb_move_x - self.uwb_car.uwb_cur_x) > 0.5:
                    if (self.uwb_car.uwb_cur_x >= self.uwb_car.uwb_move_x):
                        self.uwb_car.uwb_on_x = 1
                    else:
                        self.uwb_car.uwb_on_x = 2
                    print("x 触发前",self.uwb_car.uwb_move_x)
                    self.uwb_car.uwb_move_x = self.uwb_car.uwb_cur_x
                    #self.uwb_car.uwb_on_x = 1
                    print("x 触发后",self.uwb_car.uwb_move_x)


                self.uwb_car.uwb_last_y = self.uwb_car.uwb_cur_y
                self.uwb_car.uwb_last_x = self.uwb_car.uwb_cur_x


            elif node.id == self.uwb_robot.id:
                # rospy.loginfo(node)
                tmp_x = node.pos_3d[0]
                tmp_y = node.pos_3d[1]
                # 滑动平均滤波
                self.uwb_robot.uwb_poslists[0].insert(0,tmp_x)
                self.uwb_robot.uwb_poslists[1].insert(0,tmp_y)

                data_count = len(self.uwb_robot.uwb_poslists[0]) 

                if data_count > self.uwb_car.uwb_poslist_length:
                    self.uwb_robot.uwb_poslists[0].pop()
                    self.uwb_robot.uwb_poslists[1].pop()
                    self.uwb_robot.uwb_cur_x = sum(self.uwb_robot.uwb_poslists[0])/self.uwb_robot.uwb_poslist_length
                    self.uwb_robot.uwb_cur_y = sum(self.uwb_robot.uwb_poslists[1])/self.uwb_robot.uwb_poslist_length
                else:
                    self.uwb_robot.uwb_cur_x = sum(self.uwb_robot.uwb_poslists[0])/ data_count
                    self.uwb_robot.uwb_cur_y = sum(self.uwb_robot.uwb_poslists[1])/ data_count
                # 限幅
                # if abs(self.uwb_robot.uwb_cur_y-self.uwb_robot.uwb_last_y) < 0.02:
                #     self.uwb_robot.uwb_cur_y = self.uwb_robot.uwb_last_y

                self.uwb_robot.uwb_last_y = self.uwb_robot.uwb_cur_y
                self.uwb_robot.uwb_last_x = self.uwb_robot.uwb_cur_x
                

    def nimservos_cmd_send(self):

        msg = motor_cmd()
        msg.linear_x = self.move_x_distance
        msg.linear_y = self.move_y_distance
        self.nimservos_pub.publish(msg)

    def piddebug_msg_send(self,objectname,pidcontroller):

        msg = piddebug()
        # msg.target = self.uwb_robot.uwb_cur_y
        # msg.target = self.uwb_robot.uwb_cur_y
        # msg.rev = self.uwb_car.uwb_cur_y
        msg.objectname = objectname
        msg.target = pidcontroller.target
        msg.rev = pidcontroller.rev
        msg.kp = pidcontroller.kp
        msg.ki = pidcontroller.ki
        msg.kd = pidcontroller.kd
        msg.p_out = pidcontroller.p_out
        msg.i_out = pidcontroller.i_out
        msg.d_out = pidcontroller.d_out
        msg.pid_out = pidcontroller.pid_out
        self.piddebug_pub.publish(msg)
    
    def uwbdebug_msg_send(self):
        
        msg =uwbdebug()
        msg.car_uwb_cur_x = self.uwb_car.uwb_cur_x
        msg.car_uwb_cur_y = self.uwb_car.uwb_cur_y
        msg.robot_uwb_cur_x = self.uwb_robot.uwb_cur_x
        msg.robot_uwb_cur_y = self.uwb_robot.uwb_cur_y
        self.uwbdebug_pub.publish(msg)
        

    def on_key_press(self, event):
        key = event.keysym
        rospy.loginfo(f"Key {key} pressed")
        self.key_status_label.config(text=f"Key '{key}' pressed")
        
        if key == "w" or key == "W":
            self.move_y_distance =  MOVE_SPEED
        elif key == "s" or key == "S":
            self.move_y_distance = -MOVE_SPEED

        if key == "a" or key == "A":
            self.move_x_distance =  MOVE_SPEED
        elif key == "d" or key == "D":
            self.move_x_distance = -MOVE_SPEED

        if key == "space":
            self.move_x_distance = 0
            self.move_y_distance = 0
        
        if key == "o" or key == "O":
            self.control_mode = "键盘控制模式"
            self.control_mode_label.config(text=self.control_mode)
            self.control_tip_label.config(text="WSAD 控制天轨前后左右移动")
        elif key == "p" or key == "P":
            self.control_mode = "UWB追踪模式"
            self.control_mode_label.config(text=self.control_mode)
            self.control_tip_label.config(text=f"robot id:{self.uwb_robot.id} car id:{self.uwb_car.id}")      

        # self.nimservos_cmd_send()
        # self.uwbdebug_msg_send()      
        
    def on_key_release(self, event):
        key = event.keysym
        rospy.loginfo(f"Key {key} released")
        self.key_status_label.config(text=f"Key '{key}' released")

        # if key == "w" or key == "W":
        #     self.move_y_distance =  0
        # elif key == "s" or key == "S":
        #     self.move_y_distance = 0

        # if key == "a" or key == "A":
        #     self.move_x_distance =  0
        # elif key == "d" or key == "D":
        #     self.move_x_distance = 0

        # self.nimservos_cmd_send()

    def nimservor_cmdpub(self):

        while not rospy.is_shutdown():
             
            # rospy.loginfo("threading test")
            self.uwbdebug_msg_send()
            if self.control_mode == "键盘控制模式":
                # rospy.loginfo("键盘控制模式")
                self.nimservos_cmd_send()

            elif self.control_mode == "UWB追踪模式":
                if self.uwb_car.uwb_on_y == 0:
                    self.move_y_distance = 0
                elif self.uwb_car.uwb_on_y == 1:
                    self.uwb_car.uwb_on_y = 0
                    self.move_y_distance = MOVE_SPEED
                    print("我y前走了")
                elif self.uwb_car.uwb_on_y == 2:
                    self.move_y_distance = -MOVE_SPEED 
                    print("我y后走了")

                if self.uwb_car.uwb_on_x == 0:
                    self.move_x_distance = 0                    
                elif self.uwb_car.uwb_on_x == 1:
                    self.uwb_car.uwb_on_x = 0
                    self.move_x_distance = -MOVE_SPEED
                    print("我x前走了")
                elif self.uwb_car.uwb_on_x == 2:
                    self.uwb_car.uwb_on_x = 0
                    self.move_x_distance =  MOVE_SPEED
                    print("我x后走了")
                                    
                for i in range(100000):
                    self.nimservos_cmd_send()

                # # rospy.loginfo("UWB追踪模式")
                # # 设置pid参数
                # self.pidcontroller_y.pid_config(kp=150000,ki=0,kd=500)
                # self.pidcontroller_x.pid_config(kp=150000,ki=0,kd=500)
                # # pid运算
                # out_y = self.pidcontroller_y.pid_calcul(self.uwb_robot.uwb_cur_y,self.uwb_car.uwb_cur_y)
                # out_x = self.pidcontroller_x.pid_calcul(self.uwb_robot.uwb_cur_x,self.uwb_car.uwb_cur_x)

                # self.move_y_distance = int(out_y)
                # self.move_x_distance = int(out_x)

                # self.piddebug_msg_send("天轨y轴", self.pidcontroller_y)
                # # self.piddebug_msg_send("天轨x轴", self.pidcontroller_x)
                

                

            
            rospy.sleep(0.001)


        
def signal_handler(sig, frame):
    rospy.loginfo("Ctrl+C detected, shutting down...")
    sys.exit(0)  # Exit the program gracefully


    
if __name__ == '__main__':

    try:

        user = UserNode()
        user.nimservos_cmdpub_thread.start()

        signal.signal(signal.SIGINT, signal_handler)
        user.root.mainloop()
        
        # while not rospy.is_shutdown():

        #     # user.pidcontroller_y.pid_config(100000,0,0)
        #     # out = user.pidcontroller_y.pid_calcul(user.uwb_robot.uwb_cur_y,user.uwb_car.uwb_cur_y)
        #     # rospy.loginfo(out)
        #     # user.piddebug_msg_send()

        #     # msg = motor_cmd()
        #     # msg.linear_x = 0
        #     # msg.linear_y = int(out)
        #     # user.nimservos_pub.publish(msg)

        #     time.sleep(0.001)
            
    except rospy.ROSException as e:
        rospy.loginfo(e)
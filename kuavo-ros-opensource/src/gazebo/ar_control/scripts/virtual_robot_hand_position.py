# from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
import time
import rospy
from kuavo_msgs.msg import robotHandPosition  # 确保已创建此消息类型

class RobotHandController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('kuavo_hand_controller', anonymous=True)
        # 创建订阅者，接收手部位置命令
        self.hand_pos_sub = rospy.Subscriber('/control_robot_hand_position', 
                                            robotHandPosition, 
                                            self.hand_position_callback)
        print("Hand position controller initialized")
        
    def hand_position_callback(self, msg):
        # 处理接收到的手部位置消息
        print(f"Received hand position command: {msg}")
        # 这里添加控制机器人手部的代码
        # 例如: self.robot.set_hand_position(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)
        # 具体实现取决于KuavoRobot API和robotHandPosition消息的具体定义

def main():
    # Initialize SDK
    # 初始化手部控制器
    hand_controller = RobotHandController()

    # 保持节点运行，等待手部位置命令
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
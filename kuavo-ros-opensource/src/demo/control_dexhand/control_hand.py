#!/usr/bin/env python3
import sys
import time
import rospy
from kuavo_msgs.msg import robotHandPosition

def control_hand():
    # Initialize ROS node
    rospy.init_node('hand_control_demo', anonymous=True)
    
    # Create publisher for hand control
    pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
    while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    # Create message
    msg = robotHandPosition()    
    while not rospy.is_shutdown():
        try:
            
            # Update both hands
            print("\033[33mDual hand --> [OPEN]\033[0m")
            msg.left_hand_position = [0] * 6  
            msg.right_hand_position = [0] * 6  
            pub.publish(msg)
            time.sleep(1.5)

            print("\033[32mDual hand --> [CLOSE]\033[0m")
            msg.left_hand_position = [50, 80, 80, 80, 80, 80]
            msg.right_hand_position = [50, 80, 80, 80, 80, 80]
            pub.publish(msg)
            time.sleep(1.5)

        except KeyboardInterrupt:
            print("\nCtrl+C detected, exiting...")
            sys.exit(0)

if __name__ == '__main__':
    try:
        control_hand()
    except rospy.ROSInterruptException:
        pass

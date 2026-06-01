#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
from kuavo_msgs.msg import robotHandPosition
from sensor_msgs.msg import JointState
hand_state = None

def hand_state_callback(msg):
    global hand_state
    hand_state = msg
    # Print hand joint positions
    if hand_state is not None:
        print("Left hand positions:", hand_state.position[:6])
        print("Right hand positions:", hand_state.position[6:12])

def control_robot_hand_position(left_pos, right_pos):
    """
    Control both hands' positions
    Args:
        left_pos: List of 6 values (0-100) for left hand finger positions
        right_pos: List of 6 values (0-100) for right hand finger positions
    """
    msg = robotHandPosition()
    msg.left_hand_position = left_pos
    msg.right_hand_position = right_pos
    hand_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('dexhand_control_node')
    
    # Publisher for hand commands
    hand_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
    while hand_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for hand_pub subscriber...")
        rospy.sleep(0.1)
    
    # Subscriber for hand state
    hand_state_sub = rospy.Subscriber('/dexhand/state', JointState, hand_state_callback)
    
    # Wait for connections
    time.sleep(1)
    
    # Loop opening and closing hands
    option_open = False
    try:
        while not rospy.is_shutdown():
            if option_open:
                print("Opening hands")
                control_robot_hand_position([0]*6, [0]*6)
                option_open = False
            else:
                print("Closing hands") 
                control_robot_hand_position([100]*6, [100]*6)
                option_open = True
            time.sleep(1.5)
    except KeyboardInterrupt:
        print("\nShutting down dexhand control node...")

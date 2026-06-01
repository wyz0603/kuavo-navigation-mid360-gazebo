#!/usr/bin/env python3

# This script is designed to plot the touch state data from a dexterous hand using ROS.
# It subscribes to the '/dexhand/touch_state' topic to receive data about the normal forces
# applied to each finger of both the left and right hands. The data is visualized in real-time
# using matplotlib, with separate plots for each finger's forces on both hands.

import rospy
from kuavo_msgs.msg import dexhandTouchState

import matplotlib.pyplot as plt
import signal
import sys

class TouchStatePlotter:
    def __init__(self):
        self.fig, self.axs = plt.subplots(5, 2, figsize=(10, 15))
        self.left_hand_data = [[[0] * 100 for _ in range(3)] for _ in range(5)]
        self.right_hand_data = [[[0] * 100 for _ in range(3)] for _ in range(5)]
        self.lines = []
        self.lines2 = []
        
        for i in range(5):
            self.lines.append([
                self.axs[i, 0].plot(range(100), self.left_hand_data[i][j], label=f'Left Hand Finger {i+1} Force {j+1}')[0]
                for j in range(3)
            ])
            self.axs[i, 0].set_ylim(-10, 250)
            self.axs[i, 0].legend()
            self.axs[i, 0].set_title('normal_force')
            
            self.lines2.append([
                self.axs[i, 1].plot(range(100), self.right_hand_data[i][j], label=f'Right Hand Finger {i+1} Force {j+1}')[0]
                for j in range(3)
            ])
            self.axs[i, 1].set_ylim(-10, 250)
            self.axs[i, 1].legend()
            self.axs[i, 1].set_title('normal_force')
        
        plt.tight_layout()
        plt.ion()
        plt.show()

    def callback(self, data):
        for i in range(5):
            self.left_hand_data[i][0].append(data.left_hand[i].normal_force1) # 法向力1
            self.left_hand_data[i][0].pop(0)
            self.left_hand_data[i][1].append(data.left_hand[i].normal_force2) # 法向力2
            self.left_hand_data[i][1].pop(0)
            self.left_hand_data[i][2].append(data.left_hand[i].normal_force3) # 法向力3
            self.left_hand_data[i][2].pop(0)
            
            self.right_hand_data[i][0].append(data.right_hand[i].normal_force1) # 法向力1
            self.right_hand_data[i][0].pop(0)
            self.right_hand_data[i][1].append(data.right_hand[i].normal_force2) # 法向力2
            self.right_hand_data[i][1].pop(0)
            self.right_hand_data[i][2].append(data.right_hand[i].normal_force3) # 法向力3
            self.right_hand_data[i][2].pop(0)

    def update_plot(self):
        for i in range(5):
            for j in range(3):
                self.lines[i][j].set_ydata(self.left_hand_data[i][j])
                self.lines2[i][j].set_ydata(self.right_hand_data[i][j])
        if plt.fignum_exists(self.fig.number) == False:
            print('Figure closed, exiting...')
            rospy.signal_shutdown('Figure closed')
            return
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def listener():
    rospy.init_node('touch_state_plotter', anonymous=True)
    plotter = TouchStatePlotter()
    rospy.Subscriber('/dexhand/touch_state', dexhandTouchState, plotter.callback)
    
    rate = rospy.Rate(5)  # 5 Hz
    while not rospy.is_shutdown():
        plotter.update_plot()
        rate.sleep()

def signal_handler(sig, frame):
    print('Exiting safely...')
    plt.close('all')  # Close all matplotlib windows
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)  # Handle termination signal for safe exit
    listener()

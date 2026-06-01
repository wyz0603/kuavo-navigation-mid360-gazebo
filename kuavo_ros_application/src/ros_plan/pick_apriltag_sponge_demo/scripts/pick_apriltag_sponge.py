import subprocess
import rospy
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))

def run_ros_node():
    try:
        subprocess.run(["python3", script_dir + "/pick_apriltag_sponge_demo.py"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running ROS node: {e}")

def run_cali_node():
    try:
        subprocess.run(["python3", script_dir + "/cali_apriltag_position.py"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running ROS node: {e}")

def main():
    rospy.init_node("pick_apriltag_sponge_demo", anonymous=True)
    mode = rospy.get_param('~mode', 'ros')
    if mode == "ros":
        run_ros_node()
    elif mode == "cali":
        run_cali_node()
    else:
        run_ros_node()

if __name__ == "__main__":
    main()

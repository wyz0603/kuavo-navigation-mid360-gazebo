import subprocess
import rospy

def run_ros_node():
    try:
        subprocess.run(["python3", "/home/kuavo/kuavo_ros_application/src/ros_plan/moveit_interface_plan/scripts/water_catch_vision_demo.py"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running ROS node: {e}")

if __name__ == "__main__":
    rospy.init_node("demo_node", anonymous=True)
    run_ros_node()

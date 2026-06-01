<a id="quickstart"></a>

# Quickstart

#### WARNING
Before running any code examples, make sure to start the robot first by executing either:

- For simulation: `roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch` (Example command)
- For real robot: `roslaunch humanoid_controllers load_kuavo_real.launch` (Example command)

## Getting Started

Before using the Kuavo SDK, you must first initialize it by calling `KuavoSDK().Init()`. This is required before any other SDK functionality can be used.

Here’s a minimal example:

```python3
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot

# Initialize the SDK - this is required!
if not KuavoSDK().Init():
    print("Init KuavoSDK failed, exit!")
    exit(1)

# Create robot instance after successful initialization
robot = KuavoRobot()

# Now you can use the robot object...
```

#### WARNING
Calling SDK functions without first calling `KuavoSDK().Init()` or when it returns `False` is illegal and may lead to undefined behavior. The consequences are unpredictable and could be dangerous.

### Basic Example

Let’s break down this example:

1. First, we import the required modules:
   - `KuavoSDK` - The main SDK interface
   - `KuavoRobot` - The robot control interface
   - `time` - For timing control
2. In the `main()` function:
   - We initialize the SDK with `KuavoSDK().Init()` - this is a critical first step
   - Create a robot instance with `KuavoRobot()`
3. Basic robot control sequence:
   - `arm_reset()` - Moves the arms to their default position
   - `stance()` - Puts the robot in a stable standing position
   - `trot()` - Switches to a trotting gait mode
4. Walking control:
   - We set up a 4-second duration and walking speed of 0.3 m/s

This example demonstrates the basic workflow of initializing the SDK and controlling the robot’s movement.

Here’s a complete example showing basic robot control after initialization:

```python
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
import time

def main():
    # Initialize SDK
    if not KuavoSDK().Init():
        print("Init KuavoSDK failed, exit!")
        exit(1)
        
    robot = KuavoRobot()

    # Reset arm position
    robot.arm_reset()

    # Move to stance position
    robot.stance()

    # Switch to trot gait
    robot.trot()

    # Walk forward for 4 seconds
    duration = 4.0  # seconds
    speed = 0.3     # m/s
    start_time = time.time()
    while (time.time() - start_time < duration):
        robot.walk(linear_x=speed, linear_y=0.0, angular_z=0.0)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
```

Here’s a complete example showing basic robot get apriltag vision data after initialization:

```python
import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk import KuavoRobotState
from kuavo_humanoid_sdk import KuavoRobotVision
def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot() 
    robot_state = KuavoRobotState()
    robot_vision = KuavoRobotVision()
    
    # Stance
    robot.stance()

    # wait for stance state
    if robot_state.wait_for_stance(timeout=100.0):
        print("Robot is in stance state")

    # 获取Apriltag数据
    time.sleep(0.1)
    print("Apriltag data from camera:")
    print(robot_vision.apriltag_data_from_camera)
    print("Apriltag data from base:")
    print(robot_vision.apriltag_data_from_base)
    print("Apriltag data from odom:")
    print(robot_vision.apriltag_data_from_odom)
    
    # 识别到的第一个tag的所有数据
    print(robot_vision.apriltag_data_from_odom.id[0])
    print(robot_vision.apriltag_data_from_odom.size[0])
    print(robot_vision.apriltag_data_from_odom.pose[0].position.x)
    print(robot_vision.apriltag_data_from_odom.pose[0].position.y)
    print(robot_vision.apriltag_data_from_odom.pose[0].position.z)
    print(robot_vision.apriltag_data_from_odom.pose[0].orientation.x)
    print(robot_vision.apriltag_data_from_odom.pose[0].orientation.y)
    print(robot_vision.apriltag_data_from_odom.pose[0].orientation.z)
    print(robot_vision.apriltag_data_from_odom.pose[0].orientation.w)

    # 识别到的第二个tag的所有数据
    print(robot_vision.apriltag_data_from_odom.id[1])
    print(robot_vision.apriltag_data_from_odom.size[1])
    print(robot_vision.apriltag_data_from_odom.pose[1].position.x)
    print(robot_vision.apriltag_data_from_odom.pose[1].position.y)
    print(robot_vision.apriltag_data_from_odom.pose[1].position.z)
    print(robot_vision.apriltag_data_from_odom.pose[1].orientation.x)
    print(robot_vision.apriltag_data_from_odom.pose[1].orientation.y)
    print(robot_vision.apriltag_data_from_odom.pose[1].orientation.z)
    print(robot_vision.apriltag_data_from_odom.pose[1].orientation.w)

    # 获取指定tag的数据
    print("tag 0 data:")
    print(robot_vision.get_data_by_id(0, "odom"))
    print("tag 2 data:")
    print(robot_vision.get_data_by_id(2, "odom"))
    print("tag 3 data:")
    print(robot_vision.get_data_by_id(3, "odom"))

    while True:
        time.sleep(0.1)
if __name__ == "__main__":
    main()
```

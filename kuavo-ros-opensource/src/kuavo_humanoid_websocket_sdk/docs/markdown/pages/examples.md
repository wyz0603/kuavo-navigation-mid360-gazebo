<a id="examples"></a>

# Examples

#### WARNING
Before running any code examples, make sure to start the robot first by executing either:

- For simulation: `roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch` (Example command)
- For real robot: `roslaunch humanoid_controllers load_kuavo_real.launch` (Example command)

## Robot Info

Examples showing how to get basic robot information.

```python
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobotInfo

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot_info = KuavoRobotInfo()
    print("Robot Type:", robot_info.robot_type)
    print("Robot Version:", robot_info.robot_version)
    print("End Effector Type:", robot_info.end_effector_type)
    print("Joint Names:", robot_info.joint_names)
    print("Total Joint DOF:", robot_info.joint_dof)
    print("Arm Joint DOF:", robot_info.arm_joint_dof)
    print("Arm Joint Names:", robot_info.arm_joint_names)
    print("Head Joint DOF:", robot_info.head_joint_dof)
    print("Head Joint Names:", robot_info.head_joint_names)
    print("End Effector Frame Names:", robot_info.eef_frame_names)
if __name__ == "__main__":
    main()
```

## Basic Robot Control

A basic example showing how to initialize the SDK and control the robot’s movement.

```python
import time
import signal
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState

# Global flag for handling Ctrl+C
running = True

def signal_handler(sig, frame):
    global running
    print('\nCtrl+C pressed. Stopping robot...')
    running = False

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    # Set up Ctrl+C handler
    signal.signal(signal.SIGINT, signal_handler)
    
    robot = KuavoRobot()    
    robot_state = KuavoRobotState()
    
    """ arm reset """
    print("Switching to arm reset mode...")
    robot.arm_reset()
    
    """ stance """
    print("Switching to stance mode...")
    robot.stance()

    """ trot """
    print("Switching to trot mode...")
    robot.trot()
    
    """ walk forward """
    print("Starting forward walk...")
    duration = 8.0  # seconds
    speed = 0.3     # m/s
    start_time = time.time()
    start_x = robot_state.odometry.position[0]
    while running and (time.time() - start_time < duration):
        robot.walk(linear_x=speed, linear_y=0.0, angular_z=0.0)
        print(f"Current x position: {robot_state.odometry.position[0]:.3f} m")
        time.sleep(0.1)  # Small sleep to prevent busy loop
    
    forward_distance = robot_state.odometry.position[0] - start_x
    print(f"\033[33mForward distance traveled: {forward_distance:.3f} m\033[0m")

    if running:
        """ stance """
        print("Switching back to stance mode...")
        robot.stance()

        """ walk back """
        print("Starting backward walk...")
        start_time = time.time()
        start_x = robot_state.odometry.position[0]
        while running and (time.time() - start_time < duration):
            robot.walk(linear_x=-speed, linear_y=0.0, angular_z=0.0)
            print(f"Current x position: {robot_state.odometry.position[0]:.3f} m")
            time.sleep(0.1)  # Small sleep to prevent busy loop
        
        backward_distance = abs(robot_state.odometry.position[0] - start_x)
        print(f"\033[33mBackward distance traveled: {backward_distance:.3f} m\033[0m")
        print(f"\033[33mTotal distance traveled: {forward_distance + backward_distance:.3f} m\033[0m")
        
        """ stance """
        print("Final switch to stance mode...")
        robot.stance()

        """ turn left in place """
        print("Starting left turn in place...")
        start_time = time.time()
        turn_speed = 0.4  # rad/s
        turn_duration = 8.0  # seconds
        while running and (time.time() - start_time < turn_duration):
            robot.walk(linear_x=0.0, linear_y=0.0, angular_z=turn_speed)  # Positive angular_z for left turn
            print(f"Current yaw angle: {robot_state.odometry.orientation[2]:.3f} rad")
            time.sleep(0.1)  # Small sleep to prevent busy loop
        
        """ turn right in place """
        print("Starting right turn in place...")
        start_time = time.time()
        while running and (time.time() - start_time < turn_duration):
            robot.walk(linear_x=0.0, linear_y=0.0, angular_z=-turn_speed)  # Negative angular_z for right turn
            print(f"Current yaw angle: {robot_state.odometry.orientation[2]:.3f} rad")
            time.sleep(0.1)  # Small sleep to prevent busy loop

        """ stance """
        print("Final switch to stance mode...")
        robot.stance()

    """ squat """
    if running:
        print("Starting squat motion...")
        robot.squat(-0.1)
        start_time = time.time()
        while running and (time.time() - start_time < 2.0):
            time.sleep(0.1)
        
        if running:
            print("Returning to original height...")
            robot.squat(0.0)  # Return to original height
            start_time = time.time()
            while running and (time.time() - start_time < 2.0):
                time.sleep(0.1)

if __name__ == "__main__":
    main()
```

## End Effector Control

### LejuClaw Gripper

Examples demonstrating how to control the LejuClaw gripper end effector, including position, velocity and torque control.

```python
from kuavo_humanoid_sdk import KuavoSDK, LejuClaw
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    claw = LejuClaw()
    
    # close claw
    if claw.close():
        if claw.wait_for_finish(timeout=2.0): # Wait for the claw motion to finish with a 2 second timeout
            print("Claw motion finished successfully")
            
    # open
    if claw.open():
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Open claw motion finished successfully")

    # control_left
    if claw.control_left(target_positions=[50]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Left claw motion finished successfully")

    # control_right
    if claw.control_right(target_positions=[80]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Right claw motion finished successfully")

    # control_right
    if claw.control(target_positions=[20, 100]):
        if claw.wait_for_finish(): # Wait for the claw motion to finish
            print("Both claw motion finished successfully")

    claw.open()   
    # !!! WARNING: !!!
    # This request may be dropped because the claw may still be executing the previous motion.
    claw.control(target_positions=[10, 10]) 
if __name__ == "__main__":
    main()
```

### QiangNao DexHand

Examples showing how to control the QiangNao DexHand, a dexterous robotic hand with multiple degrees of freedom for complex manipulation tasks.

```python
from kuavo_humanoid_sdk import KuavoSDK, DexterousHand
from kuavo_humanoid_sdk.interfaces.data_types import EndEffectorState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    import time
    dex_hand = DexterousHand() # init dex hand

    dex_hand.control_left([5, 5, 95, 95, 95, 95]) # 6 dof. 'thumbs-up' gesture.
    dex_hand.control_right([5, 5, 95, 95, 95, 5]) # 6 dof. '666' gesture.
    time.sleep(1.0)

    # open/reset/release
    dex_hand.open() # both.
    time.sleep(1.0)

    # get gesture names
    gestures = dex_hand.get_gesture_names()
    if gestures is None:
        print("Get gesture names failed!")
        exit(1)
    else:
        for gesture in gestures:
            print("Supported gesture: ", gesture)
    
    # make gesture
    dex_hand.make_gesture(l_gesture_name="thumbs-up", r_gesture_name="ok")
if __name__ == "__main__":
    main()
```

## Arm Control

Examples showing arm trajectory control and target pose control.

```python
import time
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot,KuavoRobotState
from kuavo_humanoid_sdk.interfaces.data_types import KuavoPose, KuavoManipulationMpcFrame

if not KuavoSDK().Init(log_level='INFO'):# Init!
    print("Init KuavoSDK failed, exit!")
    exit(1)

robot = KuavoRobot()

def control_arm_traj():
    global robot
     # reset arm
    robot.arm_reset()
    
    q_list = []
    q0 = [0.0]*14
    # open arm
    q1 = [0, 50, 0, 0, 0, 0, 0, 0.0, -50, 0, 0, 0, 0, 0]

    num = 90
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q0[j] + i/float(num)*(q1[j] - q0[j])
        q_list.append(q_tmp)
    for q in q_list:
        # !!! Convert degrees to radians
        # !!! Convert degrees to radians
        q = [math.radians(angle) for angle in q]
        robot.control_arm_joint_positions(q)
        time.sleep(0.02)
    
    # fixed arm
    robot.set_fixed_arm_mode()
    time.sleep(1.0)

    # back to q0
    for i in range(num):
        q_tmp = [0.0]*14
        for j in range(14):
            q_tmp[j] = q1[j] - i/float(num)*(q1[j] - q0[j])
                # !!! Convert degrees to radians
                # !!! Convert degrees to radians
        q_tmp = [math.radians(angle) for angle in q_tmp]
        robot.control_arm_joint_positions(q_tmp)
        time.sleep(0.02)

    robot.set_auto_swing_arm_mode() # Restore auto arm swing mode

    robot.arm_reset() # Reset arm position


def control_arm_joint_trajectory():
    global robot
    target_poses = [
        [1.0, [20, 0, 0, -30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]],
        [2.5, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]],
        [4.0, [20, 0, 0, -30, 0, 0, 0, -30, 0, 30, -88, 8, -22, -4]],
        [5.5, [20, 0, 0, -30, 0, 0, 0, -30, -25, -54, -15, -6, -22, -4]],
        [7.0, [10, 10, -20, -70, 0, 0, -24, -30, -25, -54, -15, -6, -22, -4]],
        [8.5, [14, 20, 33, -35, 76, -18, 3.5, -30, -25, -54, -15, -6, -22, -4]],
        [10, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]]
    ]

    times = [pose[0] for pose in target_poses]
    
    # !!! Convert degrees to radians
    # !!! Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]
    
    if not robot.control_arm_joint_trajectory(times, q_frames):
        print("control_arm_joint_trajectory failed!")

def control_arm_end_effector_pose():
    global robot
    robot.control_robot_end_effector_pose(KuavoPose(position=[0.3, 0.4, 0.9], orientation=[0, -0.67566370964, 0, 0.73720997571945]), KuavoPose(position=[0.3, -0.5, 1.0], orientation=[0, -0.67566370964, 0, 0.73720997571945]), KuavoManipulationMpcFrame.WorldFrame)

if __name__ == "__main__":

    robot.stance()
    robot_state = KuavoRobotState()
    if not robot_state.wait_for_stance():
        print("change to stance fail!")

    print("Robot stance !!!!!")
    # !!! Move arm trajectory
    control_arm_traj()
    
    control_arm_joint_trajectory()
    time.sleep(12)  # !!! Wait for the arm to reach the target pose
    
    robot.arm_reset() # !!! after the arm reaches the target pose, reset the arm position.
    control_arm_end_effector_pose()
    time.sleep(3)
    robot.manipulation_mpc_reset()
    robot.arm_reset()
```

## Forward and Inverse Kinematics

Examples demonstrating how to use forward kinematics (FK) to compute end-effector positions from joint angles, and inverse kinematics (IK) to calculate joint angles needed to achieve desired end-effector poses.

```python
import time
import math
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
from kuavo_humanoid_sdk import KuavoPose
 
if __name__ == "__main__":
    # !!! Initialize Kuavo SDK with IK module !!!
    if not KuavoSDK.Init(options=KuavoSDK.Options.WithIK): # Init! Important! if U use IK, you must use this option!
        print("Failed to initialize Kuavo SDK")
        exit(1)
        
    robot = KuavoRobot()
    robot.arm_reset()
    robot_state = KuavoRobotState()

    # Front pose - arms in front, palms facing down
    front_left = [0.45, 0.28, 0.25]   # Left arm front position
    front_right = [0.45, -0.20, 0.25]  # Right arm front position
    r_front_orientation = [-0.41158, -0.503073, 0.577546, 0.493919]
    l_front_orientation = [0.38, -0.45, -0.56, 0.57]
    
    arm_q = robot_state.arm_joint_state().position

    res = robot.arm_ik(
        KuavoPose(position=front_left, orientation=l_front_orientation),
        KuavoPose(position=front_right, orientation=r_front_orientation)
    )
    
    if res:
        times = [0.5, 2.5]
        q_frames = [arm_q, res]
        robot.control_arm_joint_trajectory(times, q_frames)
        time.sleep(3.5)
        robot.set_fixed_arm_mode()
        time.sleep(1.0)
        curr_q = robot_state.arm_joint_state().position
        # !!! arm fk !!!
        l_pose, r_pose = robot.arm_fk(curr_q)
        print(l_pose)
        print(r_pose)
        robot.arm_reset()    
```

## Head Control

Examples showing how to control the robot’s head movements, including nodding (pitch) and shaking (yaw) motions.

```python
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot()
    # Control head to move back and forth slowly for multiple cycles
    import time
    
    cycles = 2  # Number of cycles to perform
    interval = 0.1  # Time interval between movements in seconds
    max_pitch = 25  # Maximum pitch angle in degrees
    
    for cycle in range(cycles):
        # Move from 0 to max_pitch degrees
        for pitch in range(0, max_pitch + 1, 2):
            robot.control_head(yaw=0, pitch=pitch * 0.017)  # Convert degrees to radians
            time.sleep(interval)
        
        # Move from max_pitch to -max_pitch degrees
        for pitch in range(max_pitch, -max_pitch - 1, -2):
            robot.control_head(yaw=0, pitch=pitch * 0.017)
            time.sleep(interval)
            
        # Move from -max_pitch back to 0 degrees
        for pitch in range(-max_pitch, 1, 2):
            robot.control_head(yaw=0, pitch=pitch * 0.017)
            time.sleep(interval)

    # Control head to move left and right
    cycles = 2  # Number of cycles to perform
    interval = 0.1  # Time interval between movements in seconds
    max_yaw = 60  # Maximum yaw angle in degrees
    
    for cycle in range(cycles):
        # Move from 0 to max_yaw degrees
        for yaw in range(0, max_yaw + 1, 2):
            robot.control_head(yaw=yaw * 0.0174533, pitch=0)  # Convert degrees to radians
            time.sleep(interval)
        
        # Move from max_yaw to -max_yaw degrees
        for yaw in range(max_yaw, -max_yaw - 1, -2):
            robot.control_head(yaw=yaw * 0.0174533, pitch=0)
            time.sleep(interval)
            
        # Move from -max_yaw back to 0 degrees
        for yaw in range(-max_yaw, 1, 2):
            robot.control_head(yaw=yaw * 0.0174533, pitch=0)
            time.sleep(interval)
if __name__ == "__main__":
    main()
```

## Step-by-Step Control

Examples showing how to control the robot’s movements step by step, including individual foot placement and trajectory control.

```python
import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState

def main():
    if not KuavoSDK().Init():# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot() 
    robot_state = KuavoRobotState()

    # Stance
    robot.stance()

    # wait for stance state
    if robot_state.wait_for_stance(timeout=100.0):
        print("Robot is in stance state")
    
    # !!! Warning !!!: step_by_step control can only be used in stance mode
    # 
    # Step by step forward 0.8m
    target_poses = [0.8, 0.0, 0.0, 0.0]
    try:
        if not robot.step_by_step(target_poses):
            print("\033[33mStep by step failed\033[0m")
        else:
            print("\033[32mStep by step succeeded\033[0m")
    except Exception as e:
        print(f"Step by step error: {e}")
    
    # Wait up to 15s for stance state (adjust timeout based on actual needs)
    if robot_state.wait_for_stance(timeout=20.0):
        print("Robot is in stance state")
    else:
        print("Timed out waiting for stance state")
    
    target_poses = [0.2, 0.0, 0.0, 1.57]
    try:
        if not robot.step_by_step(target_poses):
            print("\033[33mStep by step failed\033[0m")
        else:
            print("\033[32mStep by step succeeded\033[0m")
    except Exception as e:
        print(f"Step by step error: {e}")

if __name__ == "__main__":
    main()
```

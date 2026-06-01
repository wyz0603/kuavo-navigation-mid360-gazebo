import time
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
from kuavo_humanoid_sdk import KuavoRobotState


def main():
    import argparse 

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Websocket host address')
    parser.add_argument('--port', type=int, default=9090, help='Websocket port')
    args = parser.parse_args()

    if not KuavoSDK().Init(log_level='INFO', websocket_mode=True, websocket_host=args.host, websocket_port=args.port):# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot = KuavoRobot()
    
    # Stance
    robot.stance()
    # Command Pose - Odom
    robot.control_command_pose_world(0.0, 1.0, 0.0, 1.57)
    
    # 等待3s 
    time.sleep(10.0)

    # Stance
    robot.stance()
    # Command Pose - BaseLink
    robot.control_command_pose(-2.0, 0.0, 0.0, -1.57)
    
if __name__ == "__main__":
    main()
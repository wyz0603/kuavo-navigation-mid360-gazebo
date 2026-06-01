from kuavo_humanoid_sdk import KuavoSDK, KuavoRobotObservation

def main():
    import argparse 

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Websocket host address')
    parser.add_argument('--port', type=int, default=9090, help='Websocket port')
    args = parser.parse_args()

    if not KuavoSDK().Init(log_level='INFO', websocket_mode=True, websocket_host=args.host, websocket_port=args.port):# Init!
        print("Init KuavoSDK failed, exit!")
        exit(1)

    robot_obs = KuavoRobotObservation()
    import time
    try:
        print("Printing arm position commands. Press CTRL+C to exit.")
        while True:
            # Get and print the current arm position commands
            arm_positions = robot_obs.arm_position_command
            print("Arm position commands:", arm_positions[0])
            
            # Sleep to avoid flooding the console
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Exiting...")
if __name__ == "__main__":
    main()

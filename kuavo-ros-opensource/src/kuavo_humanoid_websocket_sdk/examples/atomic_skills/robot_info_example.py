from kuavo_humanoid_sdk import KuavoSDK, KuavoRobotInfo

def main():
    import argparse 

    parser = argparse.ArgumentParser()
    parser.add_argument('--host', type=str, default='127.0.0.1', help='Websocket host address')
    parser.add_argument('--port', type=int, default=9090, help='Websocket port')
    args = parser.parse_args()

    if not KuavoSDK().Init(log_level='INFO', websocket_mode=True, websocket_host=args.host, websocket_port=args.port):# Init!
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

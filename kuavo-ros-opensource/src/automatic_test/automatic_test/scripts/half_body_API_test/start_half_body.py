#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import glob
import rospy
from std_msgs.msg import Bool
import threading
def find_half_body_launch_files():
    """
    查找automatic_test包中所有以half_body开头的launch文件
    
    返回：包含所有匹配launch文件路径的列表
    """
    # 获取ROS包路径
    try:
        # 使用rospack命令获取automatic_test包的路径
        cmd = ["rospack", "find", "automatic_test"]
        package_path = subprocess.check_output(cmd).decode().strip()
        
        # 构建launch文件夹路径
        launch_dir = os.path.join(package_path, "launch")
        
        # 检查launch目录是否存在
        if not os.path.exists(launch_dir):
            print(f"错误：launch目录不存在: {launch_dir}")
            return []
        
        # 查找所有以half_body开头的launch文件
        launch_pattern = os.path.join(launch_dir, "half_body*.launch")
        launch_files = glob.glob(launch_pattern)
        
        return launch_files
    
    except subprocess.CalledProcessError:
        print("错误：无法找到automatic_test包，请确保它已安装")
        return []
    except Exception as e:
        print(f"查找launch文件时出错: {str(e)}")
        return []
def API_status_callback(msg):
    global is_finish
    is_finish = msg.data
    if msg.data:
        print("API执行完成")
    else:
        print("API执行未完成")
def main():
    rospy.init_node('half_body_API_status')
    function_status = rospy.Subscriber('funtion_finish', Bool, API_status_callback, queue_size=1, tcp_nodelay=True)
    """
    主函数：查找并列出half_body开头的launch文件，让用户选择并执行
    """
    print("正在查找半身 API 相关的launch文件...")
    launch_files = find_half_body_launch_files()
    
    if not launch_files:
        print("未找到半身 API 开头的launch文件")
        return
    
    print("\n找到以下半身 API 的launch文件:")
    for i, file_path in enumerate(launch_files):
        file_name = os.path.basename(file_path)
        print(f"{i+1}. {file_name}")
    
    # 用户选择
    while True:
        global is_finish
        is_finish = False
        try:
            user_input = input("\n请选择要启动的文件编号 (1-{0})，输入 'A/a' 顺序执行所有文件，或输入 'q' 退出: ".format(len(launch_files)))
            
            # 检查是否要退出
            if user_input.lower() == 'q':
                print("退出程序...")
                break
                
            # 检查是否要顺序执行所有文件
            if user_input.upper() == 'A' or user_input.upper() == 'a':
                print("\n开始顺序执行所有launch文件...")
                for launch_file in launch_files:
                    is_finish = False
                    selected_file = os.path.basename(launch_file)
                    selected_file = os.path.splitext(selected_file)[0]  # 移除.launch扩展名
                    
                    print(f"\n正在启动: {selected_file}")
                    
                    # 执行roslaunch命令
                    try:
                        # 使用rosnode list命令检查节点是否存在
                        node_list = subprocess.check_output(["rosnode", "list"]).decode('utf-8')
                        
                        # 初始化启动命令
                        launch_cmd = ["roslaunch", "automatic_test", f"{selected_file}.launch"]
                        
                        # 检查humanoid_plan_arm_trajectory_node节点是否存在
                        if "/humanoid_plan_arm_trajectory_node" in node_list:
                            print("检测到humanoid_plan_arm_trajectory_node节点已存在，传入参数plan_start:=False")
                            launch_cmd.append("plan_start:=False")
                        
                        # 检查arms_ik_node节点是否存在
                        if "/arms_ik_node" in node_list:
                            print("检测到arms_ik_node节点已存在，传入参数ik_start:=False")
                            launch_cmd.append("ik_start:=False")
                            
                    except subprocess.CalledProcessError:
                        print("获取节点列表失败，使用默认启动命令")
                        launch_cmd = ["roslaunch", "automatic_test", f"{selected_file}.launch"]
                    print(f"执行命令: {' '.join(launch_cmd)}")
                    
                    # 使用Popen代替call以获取进程ID
                    process = subprocess.Popen(launch_cmd)
                    pid = process.pid
                    print(f"启动的进程ID: {pid}")
                    
                    # 创建一个线程来检查is_finish状态
                    def check_finish_status():
                        rate = rospy.Rate(1)  # 1Hz检查频率
                        while not rospy.is_shutdown():
                            if is_finish:
                                print("检测到API执行完成，正在终止进程...")
                                process.terminate()
                                process.wait()
                                break
                            rate.sleep()
                    
                    # 启动检查线程
                    finish_check_thread = threading.Thread(target=check_finish_status)
                    finish_check_thread.daemon = True
                    finish_check_thread.start()
                    
                    try:
                        # 等待进程完成
                        process.wait()
                    except KeyboardInterrupt:
                        print("\n检测到键盘中断，正在终止进程...")
                        process.terminate()
                        process.wait()
                        break
                continue
                
            choice = int(user_input)
            if 1 <= choice <= len(launch_files):
                selected_file = os.path.basename(launch_files[choice-1])
                selected_file = os.path.splitext(selected_file)[0]  # 移除.launch扩展名
                
                print(f"\n正在启动: {selected_file}")
                
                # 执行roslaunch命令
                try:
                    # 使用rosnode list命令检查节点是否存在
                    node_list = subprocess.check_output(["rosnode", "list"]).decode('utf-8')
                    
                    # 初始化启动命令
                    launch_cmd = ["roslaunch", "automatic_test", f"{selected_file}.launch"]
                    
                    # 检查humanoid_plan_arm_trajectory_node节点是否存在
                    if "/humanoid_plan_arm_trajectory_node" in node_list:
                        print("检测到humanoid_plan_arm_trajectory_node节点已存在，传入参数plan_start:=False")
                        launch_cmd.append("plan_start:=False")
                    
                    # 检查arms_ik_node节点是否存在
                    if "/arms_ik_node" in node_list:
                        print("检测到arms_ik_node节点已存在，传入参数ik_start:=False")
                        launch_cmd.append("ik_start:=False")
                        
                except subprocess.CalledProcessError:
                    print("获取节点列表失败，使用默认启动命令")
                    launch_cmd = ["roslaunch", "automatic_test", f"{selected_file}.launch"]
                print(f"执行命令: {' '.join(launch_cmd)}")
                
                # 使用Popen代替call以获取进程ID
                process = subprocess.Popen(launch_cmd)
                pid = process.pid
                print(f"启动的进程ID: {pid}")
                # 创建一个线程来检查is_finish状态
                def check_finish_status():
                    rate = rospy.Rate(1)  # 1Hz检查频率
                    while not rospy.is_shutdown():
                        if is_finish:
                            print("检测到API执行完成，正在终止进程...")
                            process.terminate()
                            process.wait()
                            break
                        rate.sleep()
                
                # 启动检查线程
                finish_check_thread = threading.Thread(target=check_finish_status)
                finish_check_thread.daemon = True
                finish_check_thread.start()
                try:
                    # 等待进程完成
                    process.wait()
                except KeyboardInterrupt:
                    print("\n检测到键盘中断，正在终止进程...")
                    process.terminate()
                    process.wait()
                # 移除break，允许执行完成后继续选择
            else:
                print(f"无效的选择，请输入1到{len(launch_files)}之间的数字")
        except ValueError:
            print("请输入有效的数字")
        except KeyboardInterrupt:
            print("\n操作已取消")
            break

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from h12pro_controller_node.srv import ExecuteArmAction, ExecuteArmActionRequest, ExecuteArmActionResponse
import rospy
import sys
from std_msgs.msg import Bool
def call_execute_arm_action(action_name):
    """Call the /execute_arm_action service
    :param action_name 动作名字
    :return: bool， 服务调用结果
    """
    # 等待服务可用
    rospy.wait_for_service('/execute_arm_action')
    rospy.loginfo("服务 '/execute_arm_action' 已可用")
    try:
        _execute_arm_action_client = rospy.ServiceProxy('/execute_arm_action', ExecuteArmAction)
        request = ExecuteArmActionRequest()
        request.action_name = action_name

        response = _execute_arm_action_client(request)
        rospy.loginfo(f"ExecuteArmAction service response:\nsuccess: {response.success}\nmessage: {response.message}")
        return response.success, response.message
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to '/execute_arm_action' failed: {e}")
        return False, f"Service exception: {e}"
def main():
    """
    主函数，搜索指定路径下的.tact文件并依次执行
    """
    # 初始化ROS节点
    rospy.init_node('h12_action_test_node', anonymous=True)
    function_status_pub = rospy.Publisher('funtion_finish', Bool, queue_size=1)
    
    import os
    import time
    
    # 定义动作文件路径
    action_files_path = "/home/lab/.config/lejuconfig/action_files/"
    
    # 搜索所有.tact文件
    tact_files = []
    try:
        for file in os.listdir(action_files_path):
            if file.endswith(".tact"):
                tact_files.append(file[:-5])  # 去掉.tact后缀
    except Exception as e:
        rospy.logerr(f"搜索动作文件失败: {e}")
        sys.exit(1)
    
    if not tact_files:
        rospy.logerr(f"在 {action_files_path} 路径下未找到.tact文件")
        sys.exit(1)
    
    rospy.loginfo(f"找到以下动作文件: {tact_files}")
    
    # 依次执行每个动作
    for action_name in tact_files:
        rospy.loginfo(f"准备执行动作: {action_name}")
        
        # 调用服务执行动作
        success, message = call_execute_arm_action(action_name)
        
        if success:
            rospy.loginfo(f"动作 {action_name} 执行成功")
        else:
            rospy.logerr(f"动作 {action_name} 执行失败: {message}")
        
        # 等待动作完成后再执行下一个
        time.sleep(10)
    function_status_pub.publish(True)

if __name__ == "__main__":
    main()

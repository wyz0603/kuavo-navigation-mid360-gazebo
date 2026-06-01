#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from pick_basket.srv import pickBasketSrv, pickBasketSrvRequest
import signal
import sys

PICK_BASKET = 0
PUSHDOWN_BASKET = 1
TAG_ID = 7

def pick_basket_client(action, tag_id):
    # 等待服务启动
    rospy.loginfo("Waiting for pick_basket service...")
    rospy.wait_for_service('pick_basket')
    try:
        # 创建服务代理
        pick_basket = rospy.ServiceProxy('pick_basket', pickBasketSrv)
        
        # 创建请求对象
        req = pickBasketSrvRequest()
        req.action = action
        req.tag_id = tag_id
        
        # 发送请求并接收响应
        resp = pick_basket(req)
        
        # 打印响应结果
        rospy.loginfo(f"Result: {resp.result}, Message: {resp.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def signal_handler(sig, frame):
    rospy.loginfo("You pressed Ctrl+C. Exiting gracefully.")
    rospy.signal_shutdown("Ctrl+C was pressed.")
    sys.exit(0)

if __name__ == "__main__":
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # 初始化节点
    rospy.init_node('pick_basket_client_node')
    
    # 测试抓取动作
    rospy.loginfo("Sending request to pick a basket...")
    pick_basket_client(PICK_BASKET, TAG_ID)
    
    rospy.sleep(5.0)
    
    # 测试放置动作
    rospy.loginfo("Sending request to place a basket...")
    pick_basket_client(PUSHDOWN_BASKET, TAG_ID)
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from stair_alignment.srv import stairAlignmentSrv, stairAlignmentSrvRequest
from stair_alignment.msg import StairAlignmentStatus

class StairAlignmentTester:
    def __init__(self):
        rospy.init_node('stair_alignment_tester')
        
        # 等待服务启动
        rospy.wait_for_service('stair_alignment')
        rospy.loginfo("Stair alignment service is available")
        
        # 创建服务代理
        self.stair_alignment_client = rospy.ServiceProxy('stair_alignment', stairAlignmentSrv)
        
        # 订阅状态话题
        self.status_sub = rospy.Subscriber('/stair_alignment_status', StairAlignmentStatus, self.status_callback)
        
    def status_callback(self, msg):
        """状态回调函数"""
        rospy.loginfo(f"Status Update: {msg.current_state} - {msg.message}")
        rospy.loginfo(f"  Current pose: x={msg.current_x:.3f}, y={msg.current_y:.3f}, yaw={msg.current_yaw:.3f}")
        rospy.loginfo(f"  Target pose: x={msg.target_x:.3f}, y={msg.target_y:.3f}, yaw={msg.target_yaw:.3f}")
        rospy.loginfo(f"  Progress: {msg.step_count}/{msg.total_steps} steps")
        rospy.loginfo(f"  Aligned: {msg.is_aligned}")
        rospy.loginfo("=" * 50)
        
    def test_alignment(self, tag_id=1, offset_x=-0.6, offset_y=0.0, offset_yaw=0.0):
        """测试楼梯对齐服务"""
        rospy.loginfo(f"Testing stair alignment with tag_id={tag_id}, offsets=[{offset_x}, {offset_y}, {offset_yaw}]")
        
        try:
            # 创建请求
            req = stairAlignmentSrvRequest()
            req.tag_id = tag_id
            req.offset_x = offset_x
            req.offset_y = offset_y
            req.offset_yaw = offset_yaw
            
            # 调用服务
            rospy.loginfo("Sending alignment request...")
            response = self.stair_alignment_client(req)
            
            if response.result:
                rospy.loginfo(f"Alignment successful: {response.message}")
            else:
                rospy.logerr(f"Alignment failed: {response.message}")
                
            return response.result
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
            
    def test_with_launch_params(self):
        """使用launch文件参数测试"""
        rospy.loginfo("Testing with launch file parameters...")
        return self.test_alignment(0, 0, 0, 0)  # 使用0值表示使用launch参数

def main():
    if len(sys.argv) < 2:
        print("Usage: rosrun stair_alignment test_stair_alignment.py [test_type]")
        print("test_type options:")
        print("  launch - Test with launch file parameters")
        print("  custom - Test with custom parameters")
        print("Example: rosrun stair_alignment test_stair_alignment.py launch")
        return
        
    test_type = sys.argv[1]
    tester = StairAlignmentTester()
    
    if test_type == "launch":
        tester.test_with_launch_params()
    elif test_type == "custom":
        # 自定义参数测试
        tester.test_alignment(tag_id=2, offset_x=-0.5, offset_y=0.1, offset_yaw=0.1)
    else:
        print(f"Unknown test type: {test_type}")
        print("Available: launch, custom")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

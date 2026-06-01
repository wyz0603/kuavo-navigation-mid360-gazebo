#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
楼梯对齐服务客户端示例
演示如何调用楼梯对齐服务并监听状态
"""

import rospy
import time
from stair_alignment.srv import stairAlignmentSrv, stairAlignmentSrvRequest
from stair_alignment.msg import StairAlignmentStatus

class StairAlignmentClient:
    def __init__(self):
        rospy.init_node('stair_alignment_example_client')
        
        # 等待服务启动
        rospy.loginfo("等待楼梯对齐服务启动...")
        rospy.wait_for_service('stair_alignment')
        
        # 创建服务代理
        self.align_service = rospy.ServiceProxy('stair_alignment', stairAlignmentSrv)
        rospy.loginfo("楼梯对齐服务已连接")
        
        # 订阅状态话题
        self.status_sub = rospy.Subscriber('/stair_alignment_status', StairAlignmentStatus, self.status_callback)
        self.alignment_completed = False
        
    def status_callback(self, msg):
        """状态回调函数"""
        rospy.loginfo(f"[状态] {msg.current_state}: {msg.message}")
        
        if msg.current_state in ["aligning", "completed"]:
            rospy.loginfo(f"[进度] 步数: {msg.step_count}/{msg.total_steps}")
            rospy.loginfo(f"[位置] 当前: x={msg.current_x:.3f}, y={msg.current_y:.3f}, yaw={msg.current_yaw:.3f}")
            rospy.loginfo(f"[目标] 目标: x={msg.target_x:.3f}, y={msg.target_y:.3f}, yaw={msg.target_yaw:.3f}")
        
        if msg.current_state == "completed":
            self.alignment_completed = True
            rospy.loginfo("[完成] 楼梯对齐已完成!")
        elif msg.current_state == "failed":
            rospy.logerr("[失败] 楼梯对齐失败!")
            
    def align_to_stair(self, tag_id=1, offset_x=-0.6, offset_y=0.0, offset_yaw=0.0, timeout=60):
        """
        执行楼梯对齐
        
        Args:
            tag_id: AprilTag ID
            offset_x: X方向偏移 (米)
            offset_y: Y方向偏移 (米)
            offset_yaw: Yaw角度偏移 (弧度)
            timeout: 超时时间 (秒)
            
        Returns:
            bool: 是否成功
        """
        rospy.loginfo(f"开始楼梯对齐: tag_id={tag_id}, offsets=[{offset_x}, {offset_y}, {offset_yaw}]")
        
        try:
            # 创建请求
            req = stairAlignmentSrvRequest()
            req.tag_id = tag_id
            req.offset_x = offset_x
            req.offset_y = offset_y
            req.offset_yaw = offset_yaw
            
            # 重置完成标志
            self.alignment_completed = False
            
            # 调用服务
            start_time = time.time()
            response = self.align_service(req)
            
            if response.result:
                rospy.loginfo(f"[成功] {response.message}")
                
                # 等待对齐完成或超时
                while not self.alignment_completed and (time.time() - start_time) < timeout:
                    time.sleep(0.5)
                    
                if self.alignment_completed:
                    rospy.loginfo("楼梯对齐流程完成")
                    return True
                else:
                    rospy.logwarn("楼梯对齐超时")
                    return False
            else:
                rospy.logerr(f"[失败] {response.message}")
                return False
                
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False

def main():
    """主函数 - 演示不同的使用场景"""
    
    client = StairAlignmentClient()
    
    # 等待一下确保连接稳定
    rospy.sleep(1)
    
    # 示例1: 使用默认参数
    rospy.loginfo("=" * 50)
    rospy.loginfo("示例1: 使用默认参数对齐")
    success1 = client.align_to_stair()
    
    if success1:
        rospy.loginfo("示例1: 对齐成功")
    else:
        rospy.logwarn("示例1: 对齐失败")
    
    # 等待一段时间
    rospy.sleep(3)
    
    # 示例2: 使用自定义参数
    rospy.loginfo("=" * 50)
    rospy.loginfo("示例2: 使用自定义参数对齐")
    success2 = client.align_to_stair(
        tag_id=2,
        offset_x=-0.5,
        offset_y=0.1,
        offset_yaw=0.1
    )
    
    if success2:
        rospy.loginfo("示例2: 对齐成功")
    else:
        rospy.logwarn("示例2: 对齐失败")
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("示例演示完成")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断程序")

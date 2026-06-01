#!/usr/bin/env python3
"""
调试脚本：检查点云话题和字段信息
"""

import rospy
import rostopic
from sensor_msgs.msg import PointCloud2

def analyze_pointcloud_topic(topic_name):
    """分析点云话题的字段信息"""
    print(f"\n=== 分析话题: {topic_name} ===")
    
    try:
        # 检查话题是否存在
        topics = rostopic.get_topic_list()[0]
        topic_found = False
        for topic, msg_type in topics:
            if topic == topic_name:
                topic_found = True
                print(f"话题类型: {msg_type}")
                break
        
        if not topic_found:
            print(f"❌ 话题 {topic_name} 不存在")
            return
        
        # 获取一条消息来分析字段
        print("等待消息...")
        try:
            msg = rospy.wait_for_message(topic_name, PointCloud2, timeout=5.0)
            print(f"✅ 接收到消息")
            print(f"点数: {msg.width * msg.height}")
            print(f"坐标系: {msg.header.frame_id}")
            print(f"时间戳: {msg.header.stamp}")
            
            print("字段信息:")
            for i, field in enumerate(msg.fields):
                type_names = {1: 'INT8', 2: 'UINT8', 3: 'INT16', 4: 'UINT16', 
                             5: 'INT32', 6: 'UINT32', 7: 'FLOAT32', 8: 'FLOAT64'}
                type_name = type_names.get(field.datatype, f'UNKNOWN({field.datatype})')
                print(f"  [{i}] {field.name}: {type_name} (offset: {field.offset})")
                
        except rospy.ROSException as e:
            print(f"❌ 等待消息超时: {e}")
            
    except Exception as e:
        print(f"❌ 分析失败: {e}")

def main():
    rospy.init_node('debug_topics', anonymous=True)
    
    print("🔍 点云话题调试工具")
    print("=" * 50)
    
    # 列出所有点云话题
    print("\n📋 查找所有点云话题...")
    topics = rostopic.get_topic_list()[0]
    pointcloud_topics = []
    
    for topic, msg_type in topics:
        if 'PointCloud2' in msg_type:
            pointcloud_topics.append(topic)
            print(f"  - {topic} ({msg_type})")
    
    if not pointcloud_topics:
        print("❌ 未找到任何点云话题")
        return
    
    # 分析每个点云话题
    for topic in pointcloud_topics:
        analyze_pointcloud_topic(topic)
    
    print("\n" + "=" * 50)
    print("调试完成")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

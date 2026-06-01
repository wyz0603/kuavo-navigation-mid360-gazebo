#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import numpy as np

class LioBaseLinkPublisher:
    def __init__(self):
        rospy.init_node("lio_base_link_tf_pub")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(100.0)  # 100 Hz

        # 设定 frame 名
        self.from_frame = "base_link"
        self.to_frame = "radar"  # 或者叫 livox_frame，与你的系统一致
        self.lio_base_frame = "lio_base_link"
        self.lio_tf_parent = "livox_frame"

    def get_yaw_from_quaternion(self, q):
        # 从四元数中提取yaw角
        return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def quaternion_from_yaw(self, yaw):
        # 从yaw角创建四元数
        return tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)

    def run(self):
        while not rospy.is_shutdown():
            try:
                # 获取 base_link → radar 的变换
                trans = self.tf_buffer.lookup_transform(self.from_frame, self.to_frame, rospy.Time(0), rospy.Duration(0.5))

                # 构造变换消息 livox_frame → lio_base_link
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.lio_tf_parent
                t.child_frame_id = self.lio_base_frame

                # 只保留x,y平移
                t.transform.translation.x = trans.transform.translation.x
                t.transform.translation.y = trans.transform.translation.y
                t.transform.translation.z = 0.0  # 忽略z轴

                # 只保留yaw旋转
                yaw = self.get_yaw_from_quaternion(trans.transform.rotation)
                quaternion = self.quaternion_from_yaw(yaw)
                t.transform.rotation.x = quaternion[0]
                t.transform.rotation.y = quaternion[1]
                t.transform.rotation.z = quaternion[2]
                t.transform.rotation.w = quaternion[3]

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.lio_tf_parent
                t.child_frame_id = self.lio_base_frame
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = LioBaseLinkPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
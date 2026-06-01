import rospy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped

class JointPoseStamped(object):
    def __init__(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

    def posestamped(self, ee_name: str) -> PoseStamped:
        try_count=0
        while try_count < 5:
            try_count+=1
            try:
                stamp = rospy.Time.now()
                # 查询变换
                if self.tf_buffer.can_transform('torso', ee_name , stamp, rospy.Duration(1.0)):
                    trans = self.tf_buffer.lookup_transform('torso', ee_name, stamp)
                    # 创建 PoseStamped 对象
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = 'torso'
                    pose_stamped.header.stamp = stamp
                    pose_stamped.pose.position = trans.transform.translation
                    # 显式地赋值 Quaternion 的 x, y, z, w 分量
                    pose_stamped.pose.orientation.x = -trans.transform.rotation.x
                    pose_stamped.pose.orientation.y = -trans.transform.rotation.y
                    pose_stamped.pose.orientation.z = -trans.transform.rotation.z
                    pose_stamped.pose.orientation.w = -trans.transform.rotation.w
                            
                    return pose_stamped
            except Exception as e:
                rospy.logerr("Error looking up transform for %s: %s", ee_name, e)   
        return None

if __name__ == '__main__':
    rospy.init_node('JointPoseStamped')
    ee = JointPoseStamped()
    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        try:
             print(ee.posestamped('r_hand_end_virtual'))
        except Exception as e:
            print("Failed to retrieve transform: {}".format(e))
            rate.sleep()
            continue
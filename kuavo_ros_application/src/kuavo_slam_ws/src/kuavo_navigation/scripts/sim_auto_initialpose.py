#!/usr/bin/env python3
"""Publish an initial pose to /initialpose using the robot's true world-frame
pose from TF (Gazebo ground truth).

In sim we know exactly where the robot is via the URDF chain `world ->
base_link`, so there is no point in making the user click 2D Pose Estimate
in RViz and pray that ICP matches. This node waits until:
  1. TF has world -> base_link
  2. /Odometry is being published (FAST-LIO is up)
  3. /cloud_registered is being published (FAST-LIO has at least one scan)
then publishes the pose to /initialpose and exits.
"""

import math
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def wait_for_topic(topic, msg_type, timeout=60.0):
    """Block until at least one message arrives on `topic` or timeout."""
    rospy.loginfo(f"sim_auto_initialpose: waiting for {topic}...")
    try:
        rospy.wait_for_message(topic, msg_type, timeout=timeout)
        return True
    except rospy.ROSException:
        rospy.logwarn(f"sim_auto_initialpose: timed out waiting for {topic}")
        return False


def main():
    rospy.init_node("sim_auto_initialpose")

    source_frame = rospy.get_param("~source_frame", "base_link")
    target_frame = rospy.get_param("~target_frame", "world")
    delay = rospy.get_param("~delay", 3.0)  # extra wait after FAST-LIO is up

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 1. wait for FAST-LIO outputs - global_localization needs cur_scan + cur_odom
    if not wait_for_topic("/Odometry", Odometry):
        rospy.logerr("sim_auto_initialpose: FAST-LIO /Odometry never came up; not publishing initialpose.")
        return
    if not wait_for_topic("/cloud_registered", PointCloud2):
        rospy.logwarn("sim_auto_initialpose: /cloud_registered never came up; trying anyway.")

    rospy.loginfo(f"sim_auto_initialpose: FAST-LIO ready, waiting {delay}s for it to stabilize...")
    rospy.sleep(delay)

    # 2. look up the robot pose in world frame (Gazebo ground truth via URDF chain)
    deadline = rospy.Time.now() + rospy.Duration(30.0)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            tf = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))
            break
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
            rospy.logwarn_throttle(3.0, f"sim_auto_initialpose: waiting for {target_frame} -> {source_frame}: {e}")
            rospy.sleep(0.5)
    else:
        rospy.logerr(f"sim_auto_initialpose: TF lookup {target_frame} -> {source_frame} never succeeded; giving up.")
        return

    t = tf.transform.translation
    r = tf.transform.rotation

    # 3. publish /initialpose at map frame using the world pose. world == map
    #    in our nav_sim setup (static identity TF), so the numerical values pass
    #    straight through.
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)  # let publisher connect

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = t.x
    msg.pose.pose.position.y = t.y
    msg.pose.pose.position.z = 0.0  # 2D nav
    msg.pose.pose.orientation = r
    # diag(0.25, 0.25, ..., 0.07) - typical RViz click covariance
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.25  # x, y
    cov[35] = 0.07          # yaw
    msg.pose.covariance = cov

    pub.publish(msg)
    rospy.loginfo(f"sim_auto_initialpose: published initialpose at "
                  f"({t.x:.3f}, {t.y:.3f}, yaw_from_quat={math.atan2(2.0*(r.w*r.z + r.x*r.y), 1.0 - 2.0*(r.y*r.y + r.z*r.z)):.3f})")

    # Keep the latched publisher alive for a few seconds so global_localization's
    # subscriber definitely sees the message.
    rospy.sleep(2.0)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

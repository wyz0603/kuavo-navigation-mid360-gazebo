#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices, inverse_matrix, translation_from_matrix, quaternion_from_matrix, euler_from_quaternion
import yaml
import os
import numpy as np
from argparse import ArgumentParser

def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--camera_type', type=str, default='realsense', choices=['realsense', 'zed'])
    parser.add_argument('--namespace_prefix', type=str, default='head', choices=['head', 'left_wrist', 'right_wrist'])
    parser.add_argument('--handeye_cali_eye_on_hand', type=str, default='false')
    parser.add_argument('--parent_frame', type=str, default="camera_link")
    parser.add_argument('--child_frame', type=str, default="camera_color_optical_frame")
    return parser.parse_known_args()

def from_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        in_dict = yaml.load(f, Loader=yaml.FullLoader)
    return from_dict(in_dict)

def from_dict(in_dict):
    """
    Sets values parsed from a given dictionary.

    :param in_dict: input dictionary.
    :type in_dict: dict[string, string|dict[string,float]]

    :rtype: None
    """
    param = in_dict['parameters']
    tr = in_dict['transformation']
    return param, tr

def matrix_from_transform(transform):
    """Convert a transform message to a 4x4 matrix."""
    trans = transform.transform.translation
    rot = transform.transform.rotation
    trans_matrix = translation_matrix([trans.x, trans.y, trans.z])
    rot_matrix = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    return concatenate_matrices(trans_matrix, rot_matrix)

def transform_from_matrix(matrix, parent_frame, child_frame, stamp=None):
    """Convert a 4x4 matrix to a transform message."""
    if stamp is None:
        stamp = rospy.Time.now()
    
    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    
    translation = translation_from_matrix(matrix)
    quaternion = quaternion_from_matrix(matrix)
    
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    rpy = euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
    print(f"POSITION: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")
    print(f"ROTATION: {rpy[0]}, {rpy[1]}, {rpy[2]}")
    return transform

if __name__ == '__main__':
    args, unknown = parse_args()
    camera_type = args.camera_type
    parent_frame = args.parent_frame
    child_frame = args.child_frame
    
    rospy.init_node('publish_calibration_tf', anonymous=True)
    
    file_suffix = "eye_on_base" if args.handeye_cali_eye_on_hand == "false" else "eye_on_hand"
    cali_file = rospy.get_param('~calibration_file', f'~/.ros/easy_handeye/{args.namespace_prefix}_{file_suffix}.yaml')
    cali_file = os.path.expanduser(cali_file)

    try:
        param, tr = from_yaml(cali_file)
        
        print(param)
        print(tr)
        
        # create a tf listener to get the transform between camera_color_optical_frame and camera_link
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        
        # wait for the transform between camera_color_optical_frame and camera_link to be available
        try:
            rospy.loginfo("Waiting for transform between camera_color_optical_frame and camera_link...")
            optical_to_link = tfBuffer.lookup_transform(child_frame, parent_frame, rospy.Time(0), rospy.Duration(5.0))
            rospy.loginfo(f"Got transform from {child_frame} to {parent_frame}")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to lookup transform: {e}")
            exit(1)
        
        # convert the calibration transform (robot_base_frame to camera_color_optical_frame) to a matrix
        base_to_optical = np.identity(4)
        
        # fill the transform matrix in the calibration
        base_to_optical[:3, 3] = [tr["x"], tr["y"], tr["z"]]
        quat_matrix = quaternion_matrix([tr["qx"], tr["qy"], tr["qz"], tr["qw"]])
        base_to_optical[:3, :3] = quat_matrix[:3, :3]
        
        # convert the optical_to_link transform to a matrix
        optical_to_link_matrix = matrix_from_transform(optical_to_link)
        
        # calculate the transform between robot_base_frame and camera_link
        base_to_link = np.dot(base_to_optical, optical_to_link_matrix)
        
        # create and fill the new TransformStamped message
        static_transform = transform_from_matrix(
            base_to_link, 
            param['robot_base_frame'] if param["eye_on_hand"] == False else param["robot_effector_frame"],
            parent_frame
        )
        
        # create a static transform broadcaster
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # publish the static transform
        broadcaster.sendTransform(static_transform)
        rospy.loginfo(f"Publishing static transform from {param['robot_base_frame']} to camera_link")
        
        rospy.spin()
        
    except Exception as e:
        rospy.logerr(f"failed to publish calibration tf: {e}")
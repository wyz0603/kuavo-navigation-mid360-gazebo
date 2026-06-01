import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion

def calculate_transformation_matrix(pose_source:Pose, pose_target:Pose):
    
    # 计算旋转矩阵
    src_quat = [pose_source.orientation.x, pose_source.orientation.y, pose_source.orientation.z, pose_source.orientation.w]
    target_quat = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
    
    rotation_source = R.from_quat(src_quat)
    rotation_target = R.from_quat(target_quat)

    rotation_matrix_source = rotation_source.as_matrix()
    rotation_matrix_target = rotation_target.as_matrix()

    # 计算从源到目标的旋转矩阵
    rotation_matrix = rotation_matrix_target @ rotation_matrix_source.T

    # 计算平移向量
    targat_pos = np.array([pose_target.position.x, pose_target.position.y, pose_target.position.z])
    source_pos = np.array([pose_source.position.x, pose_source.position.y, pose_source.position.z])
    translation_vector = targat_pos - rotation_matrix @ source_pos

    # 组合变换矩阵
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation_vector

    return transformation_matrix

def transform_pose(pose:Pose, transformation_matrix) -> Pose:
    pose_pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    pose_orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    
    # 将 POSE 转换为齐次坐标
    pose_homogeneous = np.hstack([pose_pos, [1]])

    # 计算坐标系2中的 POSE 位置
    pose_homogeneous_transformed = transformation_matrix @ pose_homogeneous
    pose_position_transformed = pose_homogeneous_transformed[:3]

    # 计算姿态的转换
    rotation_source = R.from_quat(pose_orientation)
    rotation_matrix_source = rotation_source.as_matrix()

    # 提取变换矩阵的旋转部分
    rotation_matrix_transform = transformation_matrix[:3, :3]

    # 计算新的旋转矩阵
    rotation_matrix_target = rotation_matrix_transform @ rotation_matrix_source

    # 将新的旋转矩阵转换回四元数
    rotation_target = R.from_matrix(rotation_matrix_target)
    pose_orientation_transformed = rotation_target.as_quat()
    pose_orientation_transformed = R.from_quat(pose_orientation_transformed).as_quat()

    target_pose = Pose()
    target_pose.position.x = pose_position_transformed[0]
    target_pose.position.y = pose_position_transformed[1]
    target_pose.position.z = pose_position_transformed[2]
    target_pose.orientation.x = pose_orientation_transformed[0]
    target_pose.orientation.y = pose_orientation_transformed[1]
    target_pose.orientation.z = pose_orientation_transformed[2]
    target_pose.orientation.w = pose_orientation_transformed[3]

    return target_pose

def Quaternion_to_quat(q:Quaternion):
    quat = [q.x, q.y, q.z, q.w]
    return R.from_quat(quat).as_quat()

def normalize_rpy(rpy):
    """
    规范化欧拉角（RPY），使其在 [-π, π] 范围内
    """
    roll, pitch, yaw = rpy
    roll = (roll + np.pi) % (2 * np.pi) - np.pi
    pitch = (pitch + np.pi) % (2 * np.pi) - np.pi
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
    return np.array([roll, pitch, yaw])

def rpy_degree(rpy:list)->list:    
    return [r * 180.0 / np.pi for r in rpy]    
def rpy_to_orientation(rpy:list, seq='xyz')->Quaternion:
    quat = R.from_euler(seq, rpy).as_quat()
    quat = R.from_quat(quat).as_quat()
    return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

def orientation_to_rpy(orientation:Quaternion, seq='xyz')->list:
    return normalize_rpy(R.from_quat(Quaternion_to_quat(orientation)).as_euler(seq))

# 示例数据
torso_pose = Pose()
torso_pose.position = Point(x=0, y=0.0, z=0.0)
torso_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

camera_pose = Pose()
camera_pose.position = Point(x=1.0, y=2.0, z=3.0)
camera_pose.orientation = Quaternion(0,0, -0.707106771713121, 0.707106790659974)

# 计算变换矩阵
transformation_matrix = calculate_transformation_matrix(torso_pose, camera_pose)

print("变换矩阵：")
print(transformation_matrix)

# 坐标系1中的 POSE1 (位置和姿态)
pose1 = Pose()
pose1.position = Point(x=1.0, y=2.0, z=3.0)
pose1.orientation = rpy_to_orientation([-1.5707963, 0, -1.5707963])
print("pose1:\n", pose1)
print("rpy1:", orientation_to_rpy(pose1.orientation))

# 将 POSE1 转换为坐标系2中的位置和姿态
target_pose = transform_pose(pose1, transformation_matrix)

print("pose2:\n", target_pose)
rpy2 = orientation_to_rpy(target_pose.orientation)
print("rpy2:", rpy2)
print("rpy2:", rpy_degree(rpy2))


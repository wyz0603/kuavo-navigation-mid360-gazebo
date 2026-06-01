from arm_kinematics import rot_to_quat, quat_to_rot

import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tf_trans
from threading import Lock
import tf2_ros
from geometry_msgs.msg import TransformStamped

class CubeAprilTag3D:
    def __init__(self, a, b, face_rotations, h):
        """
        初始化3D立方体AprilTag对象
        
        参数：
        a: 立方体边长（米）
        b: 单个AprilTag边长（米）
        face_rotations: 各面旋转方向字典，格式{面索引: 旋转代码(0-3)}
                        (0: 0°, 1: 90°, 2: 180°, 3: 270°)
        h: 连接结构到立方体中心的距离（米）
        """
        self.a = a
        self.b = b
        self.h = h
        
        # 定义立方体各面属性（前5个面）·
        self.faces = {
            0: {'axis': 'front',  'position': np.array([0, 0, a/2])},    # +Z方向
            1: {'axis': 'back',   'position': np.array([0, 0, -a/2])},   # -Z方向
            2: {'axis': 'left',   'position': np.array([-a/2, 0, 0])},  # -X方向
            3: {'axis': 'right',  'position': np.array([a/2, 0, 0])},    # +X方向
            4: {'axis': 'top',    'position': np.array([0, a/2, 0])},   # +Y方向
        }
        
        # 预计算各面的基础旋转矩阵
        self.base_rotations = {
            0: self.rotation_y(0),         # 前面
            1: self.rotation_y(np.pi),      # 后面
            2: self.rotation_y(-np.pi/2),  # 左面
            3: self.rotation_y(np.pi/2),    # 右面
            4: self.rotation_x(-np.pi/2),    # 顶面
        }
        
        # 存储最终计算结果
        self.tags = {}
        self.compute_all_tags(face_rotations)
        
    @staticmethod
    def rotation_x(theta):
        """生成绕X轴的旋转矩阵"""
        return np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
    
    @staticmethod
    def rotation_y(theta):
        """生成绕Y轴的旋转矩阵"""
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
    
    @staticmethod
    def rotation_z(theta):
        """生成绕Z轴的旋转矩阵"""
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
    
    def compute_all_tags(self, face_rotations):
        """计算所有标签相对于连接结构的位置和姿态"""
        # 连接结构位置（假设位于底面中心，沿-Y方向）
        connector_pos = np.array([0, -self.h, 0])
        
        for face_idx in self.faces:
            # 获取面基础属性
            face = self.faces[face_idx]
            base_rot = self.base_rotations[face_idx]
            rotation_code = face_rotations[face_idx]
            
            # 计算标签旋转（绕该面Z轴的附加旋转）
            theta = rotation_code * np.pi/2  # 转换为弧度
            additional_rot = self.rotation_z(theta)
            
            # 合成最终旋转矩阵
            final_rotation = base_rot @ additional_rot
            
            # 计算相对位置
            relative_position = face['position'] - connector_pos
            
            # 存储结果（位置以米为单位，旋转矩阵为3x3）
            self.tags[face_idx] = {
                'position': relative_position,
                'rotation': final_rotation,
                'size': self.b
            }
    
    def get_tag_pose(self, face_idx):
        """
        获取指定面的标签位姿
        返回：
        (position, rotation_matrix)
        position: 相对于连接结构的3D坐标 (numpy数组)
        rotation_matrix: 3x3旋转矩阵 (numpy数组)
        """
        tag = self.tags.get(face_idx)
        if tag:
            return tag['position'], tag['rotation']
        raise ValueError(f"无效的面索引: {face_idx}")


    def visualize_cube_with_tags(self):
        """
        可视化立方体及其AprilTag的位置和方向
        
        """
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        import numpy as np
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制立方体
        a = self.a
        h = self.h
        connector_pos = np.array([0, -h, 0])
        
        # 立方体顶点坐标（相对于中心）
        vertices = np.array([
            [-a/2, -a/2, -a/2], [a/2, -a/2, -a/2], [a/2, a/2, -a/2], [-a/2, a/2, -a/2],
            [-a/2, -a/2, a/2], [a/2, -a/2, a/2], [a/2, a/2, a/2], [-a/2, a/2, a/2]
        ])
        
        # 立方体面
        faces = [
            [0, 1, 2, 3],  # 底面 (-Z)
            [4, 5, 6, 7],  # 顶面 (+Z)
            [0, 1, 5, 4],  # 前面 (-Y)
            [2, 3, 7, 6],  # 后面 (+Y)
            [0, 3, 7, 4],  # 左面 (-X)
            [1, 2, 6, 5]   # 右面 (+X)
        ]
        
        # 绘制半透明立方体
        cube_faces = [[vertices[idx] for idx in face] for face in faces]
        cube = Poly3DCollection(cube_faces, alpha=0.2, linewidths=1, edgecolor='k')
        ax.add_collection3d(cube)
        
        # 绘制连接点
        ax.scatter(*connector_pos, color='red', s=100, label='connect point')
        
        # 绘制AprilTag
        face_colors = ['blue', 'green', 'purple', 'orange', 'magenta']
        b = self.b  # 标签边长
        
        for face_idx in self.tags:
            tag = self.tags[face_idx]
            pos = tag['position'] + connector_pos  # 转换为全局坐标
            rot = tag['rotation']
            
            # 标签坐标系的三个轴
            axis_len = 0.05
            x_axis = pos + rot[:, 0] * axis_len
            y_axis = pos + rot[:, 1] * axis_len
            z_axis = pos + rot[:, 2] * axis_len
            
            # 绘制标签坐标轴
            ax.plot([pos[0], x_axis[0]], [pos[1], x_axis[1]], [pos[2], x_axis[2]], 'r-', linewidth=2)  # X轴
            ax.plot([pos[0], y_axis[0]], [pos[1], y_axis[1]], [pos[2], y_axis[2]], 'g-', linewidth=2)  # Y轴
            ax.plot([pos[0], z_axis[0]], [pos[1], z_axis[1]], [pos[2], z_axis[2]], 'b-', linewidth=2)  # Z轴
            
            # 计算标签四个角点
            half_b = b/2
            corners = np.array([
                [-half_b, -half_b, 0],
                [half_b, -half_b, 0],
                [half_b, half_b, 0],
                [-half_b, half_b, 0]
            ])
            
            # 转换到世界坐标系
            world_corners = np.array([pos + rot @ corner for corner in corners])
            
            # 绘制标签
            tag_face = Poly3DCollection([world_corners], alpha=0.7, linewidths=1, edgecolor='k')
            tag_face.set_facecolor(face_colors[face_idx])
            ax.add_collection3d(tag_face)
            
            # 添加标签编号
            ax.text(pos[0], pos[1], pos[2], f"Tag {face_idx}", color='black', fontsize=10)
        
        # 设置坐标轴标签和范围
        ax.set_xlabel('X axis (m)')
        ax.set_ylabel('Y axis (m)')
        ax.set_zlabel('Z axis (m)')
        
        # 设置坐标轴范围
        max_range = max(a, h) + 0.1
        ax.set_xlim(-max_range, max_range)
        ax.set_ylim(-max_range, max_range)
        ax.set_zlim(-max_range, max_range)
        
        # 添加坐标轴
        origin = np.zeros(3)
        axis_len = 0.05
        ax.quiver(origin[0], origin[1], origin[2], axis_len, 0, 0, color='red', label='X axis')    
        ax.quiver(origin[0], origin[1], origin[2], 0, axis_len, 0, color='green', label='Y axis')
        ax.quiver(origin[0], origin[1], origin[2], 0, 0, axis_len, color='blue', label='Z axis')
        
        ax.set_title('cube AprilTag pose visualization')
        ax.legend()
        plt.tight_layout()
        plt.show()

    def print_params(self):
        print(f"########################################################")
        print("立方体检测器参数:")
        print(f"立方体边长: {self.a}")
        print(f"标签边长: {self.b}")
        print(f"连接结构高度: {self.h}")
        print(f"面权重: {self.face_weights}")
        print(f"########################################################")

class AprilTag3DRos(CubeAprilTag3D):
    def __init__(self, a, b, face_rotations, h, tag_id_mapping=None):
        """
        初始化3D立方体AprilTag的ROS接口
        
        参数：
        a: 立方体边长（米）
        b: 单个AprilTag边长（米）
        face_rotations: 各面旋转方向字典，格式{面索引: 旋转代码(0-3)}
        h: 连接结构到立方体中心的距离（米）
        tag_id_mapping: 面索引到标签ID的映射字典，例如{0: 5, 1: 6, 2: 7, 3: 8, 4: 9}
                       如果为None，则假设标签ID与面索引相同
        """
        # 调用父类构造函数
        super(AprilTag3DRos, self).__init__(a, b, face_rotations, h)
        
        # 初始化tag_id到面索引的映射
        self.tag_id_to_face = {}
        if tag_id_mapping is None:
            # 默认映射：标签ID与面索引相同
            self.tag_id_to_face = {i: i for i in range(5)}
        else:
            # 使用提供的映射
            self.tag_id_to_face = tag_id_mapping
            
        # 反向映射：面索引到标签ID
        self.face_to_tag_id = {v: k for k, v in self.tag_id_to_face.items()}
        
        # 存储当前立方体位姿
        self.cube_position = np.zeros(3)
        self.cube_orientation = np.eye(3)
        self.cube_pose_timestamp = None
        
        # 最近检测到的标签ID列表
        self.detected_tag_ids = []
        
        # 线程安全锁
        self.lock = Lock()
        
        # 可信度权重 - 每个面的检测可信度调整
        self.face_weights = {
            0: 1.0,  # 前面
            1: 1.0,  # 后面
            2: 1.0,  # 左面
            3: 1.0,  # 右面
            4: 1.0   # 顶面
        }
        
        # 过滤参数
        self.position_threshold = 0.05  # 位置异常值阈值（米）
        self.rotation_threshold = 0.2   # 旋转异常值阈值（弧度）
        
        # 初始化ROS订阅者
        self.tag_sub = rospy.Subscriber("/tag_detections", 
                                        AprilTagDetectionArray, 
                                        self.tag_callback,
                                        queue_size=1)
        
        # 可选：发布计算出的立方体位姿
        self.pose_pub = rospy.Publisher("/cube_pose", 
                                       PoseWithCovarianceStamped, 
                                       queue_size=1)
        
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        rospy.loginfo("AprilTag3DRos 初始化完成")
        
    def get_T_il(self, p_if, R_if, idx):
        p_lf, R_lf = self.get_tag_pose(idx)
        R_il = R_if @ R_lf.T
        p_il = p_if - R_il @ p_lf
        return p_il, R_il

    def tag_callback(self, msg):
        """处理接收到的AprilTag检测数据"""
        with self.lock:
            # 清空当前检测结果
            self.detected_tag_ids = []
            cube_poses = []
            
            for detection in msg.detections:
                tag_id = detection.id[0]  # 获取标签ID
                
                # 检查ID是否在我们的映射中
                if tag_id not in self.tag_id_to_face:
                    continue
                    
                face_idx = self.tag_id_to_face[tag_id]
                self.detected_tag_ids.append(tag_id)
                
                # 获取标签在相机坐标系中的位姿
                pose = detection.pose.pose.pose
                p_it = np.array([
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ])
                tag_orientation_quat = np.array([
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z
                ])
                R_it = quat_to_rot(tag_orientation_quat)
                
                
                # self.publish_tf(p_it, tag_orientation_quat, "img_link", f"ltag_{face_idx}")
                
                p_if, R_if = self.get_T_il(p_it, R_it, face_idx)
                quat_if = rot_to_quat(R_if)
                # self.publish_tf(p_if, quat_if, "img_link", f"cube_{face_idx}")
                
                # 存储这个标签计算的立方体位姿
                confidence = self.face_weights[face_idx]
                cube_poses.append({
                    'position': p_if,
                    'rotation': R_if,
                    'confidence': confidence,
                    'face_idx': face_idx
                })
            
            # 如果没有检测到任何标签，直接返回
            if not cube_poses:
                return
                
            # 使用多个标签融合计算立方体位姿
            self.fuse_cube_poses(cube_poses, msg.header.stamp)
            
            # 发布立方体位姿
            self.publish_pose(msg.header)
    
    def fuse_cube_poses(self, cube_poses, timestamp):
        """融合多个标签提供的立方体位姿估计"""
        if not cube_poses:
            return
            
        # 先进行异常值过滤（如果有多个标签）
        filtered_poses = cube_poses
        if len(cube_poses) > 1:
            filtered_poses = self.filter_outliers(cube_poses)
            
        # 如果过滤后没有剩余的有效估计，直接使用原始数据
        if not filtered_poses:
            filtered_poses = cube_poses
            
        # 按置信度计算加权平均
        total_confidence = sum(pose['confidence'] for pose in filtered_poses)
        if total_confidence == 0:
            # 避免除零错误
            weights = [1.0/len(filtered_poses)] * len(filtered_poses)
        else:
            weights = [pose['confidence']/total_confidence for pose in filtered_poses]
            
        # 位置直接加权平均
        avg_position = np.zeros(3)
        for i, pose in enumerate(filtered_poses):
            avg_position += weights[i] * pose['position']
            
        # 旋转需要特殊处理（四元数加权平均）
        quats = [rot_to_quat(pose['rotation']) for pose in filtered_poses]
        
        # 使用加权平均的四元数，确保方向一致
        ref_quat = quats[0]
        for i in range(1, len(quats)):
            # 确保四元数方向一致（避免平均抵消）
            if np.dot(ref_quat, quats[i]) < 0:
                quats[i] = -quats[i]
                
        avg_quat = np.zeros(4)
        for i, quat in enumerate(quats):
            avg_quat += weights[i] * quat
            
        # 归一化
        avg_quat = avg_quat / np.linalg.norm(avg_quat)
        
        # 转换回旋转矩阵
        avg_rotation = quat_to_rot(avg_quat)
        
        # 更新立方体位姿
        self.cube_position = avg_position
        self.cube_orientation = avg_rotation
        self.cube_pose_timestamp = timestamp
        
        rospy.logdebug(f"融合了{len(filtered_poses)}个标签的立方体位姿")
    
    def filter_outliers(self, cube_poses):
        """过滤异常值"""
        if len(cube_poses) <= 1:
            return cube_poses
            
        # 计算位置的平均值和标准差
        positions = np.array([pose['position'] for pose in cube_poses])
        mean_pos = np.mean(positions, axis=0)
        
        # 计算每个估计与平均位置的距离
        distances = np.linalg.norm(positions - mean_pos, axis=1)
        
        # 计算旋转的"平均值"（用四元数表示）
        quats = [tf_trans.quaternion_from_matrix(np.vstack([
                np.hstack([pose['rotation'], np.zeros((3, 1))]), 
                [0, 0, 0, 1]
            ])) for pose in cube_poses]
        
        # 计算每个四元数与其他四元数的平均差异
        rotation_diffs = []
        for i, q1 in enumerate(quats):
            diffs = []
            for j, q2 in enumerate(quats):
                if i != j:
                    # 计算四元数之间的角度
                    dot_product = np.abs(np.dot(q1, q2))  # 绝对值处理四元数的双倍覆盖
                    dot_product = min(dot_product, 1.0)  # 数值稳定性
                    angle_diff = 2 * np.arccos(dot_product)
                    diffs.append(angle_diff)
            
            # 如果有其他四元数比较
            if diffs:
                rotation_diffs.append(np.mean(diffs))
            else:
                rotation_diffs.append(0.0)
        
        # 根据阈值过滤
        filtered_poses = []
        for i, pose in enumerate(cube_poses):
            if (distances[i] <= self.position_threshold and 
                rotation_diffs[i] <= self.rotation_threshold):
                filtered_poses.append(pose)
                
        return filtered_poses
    
    def publish_pose(self, header):
        """发布立方体位姿"""
        if self.cube_pose_timestamp is None:
            return
            
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
        
        # 设置位置
        pose_msg.pose.pose.position.x = self.cube_position[0]
        pose_msg.pose.pose.position.y = self.cube_position[1]
        pose_msg.pose.pose.position.z = self.cube_position[2]
        
        # 旋转矩阵转四元数
        quat = rot_to_quat(self.cube_orientation)
        
        pose_msg.pose.pose.orientation.w = quat[0]
        pose_msg.pose.pose.orientation.x = quat[1]
        pose_msg.pose.pose.orientation.y = quat[2]
        pose_msg.pose.pose.orientation.z = quat[3]
        
        # 设置协方差（简单起见设为单位阵的对角元素）
        # 实际应用中可能需要根据检测质量动态调整
        pose_msg.pose.covariance = [0.01] * 36

        self.publish_tf(self.cube_position, quat, "img_link", "cube")
        
        # 发布消息
        self.pose_pub.publish(pose_msg)
    
    def get_cube_pose(self):
        """
        获取立方体在连接点坐标系下的位置和姿态
        
        返回:
        (position, rotation_matrix, timestamp, detected_tags)
        position: 立方体位置 (numpy数组)
        rotation_matrix: 3x3旋转矩阵 (numpy数组)
        timestamp: 时间戳
        detected_tags: 用于计算位姿的标签ID列表
        """
        with self.lock:
            return (
                self.cube_position.copy(), 
                self.cube_orientation.copy(), 
                self.cube_pose_timestamp,
                self.detected_tag_ids.copy()
            )
    
    def set_face_weight(self, face_idx, weight):
        """设置某个面的检测权重"""
        with self.lock:
            if face_idx in self.face_weights:
                self.face_weights[face_idx] = weight
                
    def set_filter_thresholds(self, position_thresh, rotation_thresh):
        """设置过滤阈值"""
        with self.lock:
            self.position_threshold = position_thresh
            self.rotation_threshold = rotation_thresh

    def publish_tf(self, p, quat, parent_frame_id, child_frame_id):
        # 发布TF转换
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame_id
        
        transform.child_frame_id = child_frame_id
        
        # 设置位置
        transform.transform.translation.x = p[0]
        transform.transform.translation.y = p[1]
        transform.transform.translation.z = p[2]
        
        # 设置旋转
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        
        # 广播TF
        self.tf_broadcaster.sendTransform(transform)

# 使用示例
if __name__ == "__main__":
    import sys
    
    
    # 定义立方体参数
    cube_size = 0.11       # 立方体边长11cm
    tag_size = 0.088       # 标签边长8.8cm
    connector_height = 0.165  # 连接结构到中心的距离16.5cm
    
    # 定义各面旋转方向（示例配置）
    face_rotations = {
        0: 0,   # 前面：0°
        1: 0,   # 后面：0°
        2: 0,   # 左面：0°
        3: 0,   # 右面：0°
        4: 0    # 顶面：0°
    }
    
    apriltag_cube = CubeAprilTag3D(a=cube_size, b=tag_size, face_rotations=face_rotations, h=connector_height)
    for i in range(5):
        p_lf, R_lf = apriltag_cube.get_tag_pose(i)
        # print(f"p_lf[{1+i}]: {p_lf}")
        # print(f"R_if[{1+i}]: {R_if}")
        R_lf_ext = np.eye(4)
        R_lf_ext[:3, :3] = R_lf 
        # print(f"q_if[{1+i}]: {tf_trans.quaternion_from_matrix(R_lf_ext)}")
    # assert 0
    # apriltag_cube.visualize_cube_with_tags()
    def get_T_il(p_if, R_if, idx):
        # R_if = tf_trans.quaternion_matrix(q_if)[:3, :3]
        p_lf, R_lf = apriltag_cube.get_tag_pose(idx)
        R_il = R_if @ R_lf.T
        p_il = p_if - R_il @ p_lf
        return p_il, R_il


    p_if_list = [
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, 1.1]),
        np.array([0.05, 0.0, 1.05]),
        np.array([-0.05, 0.0, 1.05]),
        np.array([0.0, 0.05, 1.05])
    ]
    q_if_list = [
        tf_trans.quaternion_from_euler(0, np.pi, 0, 'sxyz'),
        tf_trans.quaternion_from_euler(0, 0, 0, 'sxyz'),
        tf_trans.quaternion_from_euler(0, np.pi/2, 0, 'sxyz'),
        tf_trans.quaternion_from_euler(0, -np.pi/2, 0, 'sxyz'),
        tf_trans.quaternion_from_euler(np.pi/2, 0, np.pi, 'sxyz')
    ]
    standard_p_il = np.array([0.0, -0.1, 1.05])
    standard_R_il = tf_trans.quaternion_matrix(tf_trans.quaternion_from_euler(0, np.pi, 0, 'sxyz'))[:3, :3]
    for i in range(5):
        p_if = p_if_list[i]
        q_if = q_if_list[i]
        R_if = quat_to_rot(q_if)
        p_il, R_il = get_T_il(p_if, R_if, i)
        # print(f"p_il[{i}]: {p_il}")
        # print(f"R_il[{i}]: {R_il}")
        if np.linalg.norm(p_il - standard_p_il) < 0.01 and np.allclose(R_il, standard_R_il, atol=0.01):
            print(f"p_il[{i}] and R_il[{i}] is correct")
        else:
            print(f"p_il[{i}] and R_il[{i}] is NOT correct!!!")
    # assert 0
    
    # 如果已有运行中的ROS节点，使用现有的
    try:
        rospy.init_node('apriltag_cube_detector', anonymous=True)
    except rospy.exceptions.ROSException:
        print("ROS节点已经初始化")
    # 定义标签ID映射（假设AprilTag的ID为10-14，映射到立方体的5个面）
    tag_id_mapping = {
        0: 0,  # ID 10 -> 前面
        1: 1,  # ID 11 -> 后面
        2: 2,  # ID 12 -> 左面
        3: 3,  # ID 13 -> 右面
        4: 4   # ID 14 -> 顶面
    }
    
    # 创建立方体对象
    cube_detector = AprilTag3DRos(
        a=cube_size, 
        b=tag_size,
        face_rotations=face_rotations,
        h=connector_height,
        tag_id_mapping=tag_id_mapping
    )
    
    # 设置面检测的权重（例如，如果前面和后面的检测更可靠）
    cube_detector.set_face_weight(0, 1.2)  # 增加前面的权重
    cube_detector.set_face_weight(1, 1.2)  # 增加后面的权重
    
    # 设置过滤阈值
    cube_detector.set_filter_thresholds(position_thresh=0.05, rotation_thresh=0.2)
    
    # 简单测试循环
    try:
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            position, orientation, timestamp, detected_tags = cube_detector.get_cube_pose()
            
            if timestamp is not None:
                print("\n立方体位姿更新：")
                print(f"位置: {position}")
                # print(f"旋转矩阵:\n{orientation}")
                print(f"四元数: {rot_to_quat(orientation)}")
                print(f"检测到的标签: {detected_tags}")
                
            rate.sleep()
            
    except KeyboardInterrupt:
        print("程序终止")

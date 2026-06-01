import rospy
import cv2
import numpy as np
import os
from insightface.app import FaceAnalysis
from insightface.utils import face_align
from sklearn.metrics.pairwise import cosine_similarity

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class FaceDetect:
    def __init__(self):
        rospy.init_node("face_detect", anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.face_pub = rospy.Publisher("/face_detection_result", String, queue_size=10)
        self.bridge = CvBridge()

        # 初始化 insightface 模型
        self.face_app = FaceAnalysis(name="buffalo_l", providers=["CPUExecutionProvider"])
        self.face_app.prepare(ctx_id=-1, det_size=(640, 640))
        
        # 设置相似度阈值
        self.similarity_threshold = 0.4
        
        # 设置最小人脸区域阈值（像素）
        self.min_face_size = 100  # 最小人脸区域的边长

        # 图像缓存
        self.latest_image = None
        
        # 创建faces文件夹（如果不存在）
        self.faces_dir = "./faces"
        if not os.path.exists(self.faces_dir):
            os.makedirs(self.faces_dir)
            print(f"创建faces文件夹: {self.faces_dir}")
        
        # 加载所有模板图片
        self.template_embeddings = {}
        self.load_templates()

        # 添加一个变量来存储最新识别到的人脸名称
        self.last_detected_face = None

        # 修改为列表，用于存储多个识别到的人脸
        self.detected_faces = []

    def image_callback(self, msg):
        """接收图像的回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image.copy()
        except Exception as e:
            print(f"接收图像时出错: {e}")

    def load_templates(self):
        """加载faces文件夹中的所有模板图片并提取特征"""
        try:
            # 获取faces文件夹中的所有图片
            image_files = [f for f in os.listdir(self.faces_dir) 
                         if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
            
            if not image_files:
                print("警告: faces文件夹中没有找到图片文件")
                return
            
            for image_file in image_files:
                template_path = os.path.join(self.faces_dir, image_file)
                template_img = cv2.imread(template_path)
                
                if template_img is None:
                    print(f"无法读取模板图片: {template_path}")
                    continue
                
                # 使用InsightFace检测人脸
                faces = self.face_app.get(template_img)
                if not faces:
                    print(f"模板图片中未检测到人脸: {image_file}")
                    continue
                
                # 获取人脸关键点
                landmarks = faces[0].kps
                
                # 对齐人脸
                aligned_face = self.align_face(template_img, landmarks)
                
                # 提取模板图片中的人脸特征
                template_embedding = faces[0].embedding
                if template_embedding is None:
                    print(f"无法提取模板人脸特征: {image_file}")
                    continue
                
                # 保存模板特征
                self.template_embeddings[image_file] = template_embedding
                print(f"成功加载模板人脸特征: {image_file}")
            
            print(f"共加载了 {len(self.template_embeddings)} 个模板人脸")
            
        except Exception as e:
            print(f"加载模板图片时出错: {e}")
            rospy.signal_shutdown("模板加载失败")

    def align_face(self, image, landmarks, output_size=112):
        """对齐人脸
        Args:
            image: 输入图像
            landmarks: 人脸关键点
            output_size: 输出图像大小
        Returns:
            aligned_face: 对齐后的人脸图像
        """
        try:
            aligned_face = face_align.norm_crop(image, landmark=landmarks, image_size=output_size)
            return aligned_face
        except Exception as e:
            print(f"人脸对齐失败: {e}")
            return image  # 如果对齐失败，返回原始图像
    
    def compare_face(self, face_embedding):
        """比较人脸特征与所有模板的相似度，返回最匹配的结果"""
        best_match = None
        best_similarity = -1
        
        for template_name, template_embedding in self.template_embeddings.items():
            # 计算余弦相似度
            similarity = cosine_similarity(
                face_embedding.reshape(1, -1),
                template_embedding.reshape(1, -1)
            )[0][0]
            
            if similarity > best_similarity:
                best_similarity = similarity
                best_match = template_name
        
        return best_match, best_similarity

    def process_image(self):
        """处理图像的函数"""
        # 清空之前的人脸列表
        self.detected_faces = []
        
        try:
            # 保存当前图像，防止处理过程中被覆盖
            current_image = self.latest_image.copy()
            
            # 使用InsightFace检测人脸
            faces = self.face_app.get(current_image)
            
            # 处理每个检测到的人脸
            for face in faces:
                # 获取人脸框
                bbox = face.bbox.astype(int)
                x1, y1, x2, y2 = bbox
                
                # 计算人脸区域大小
                face_width = x2 - x1
                face_height = y2 - y1
                
                # 过滤掉太小的人脸
                if face_width < self.min_face_size and face_height < self.min_face_size:
                    continue
                
                # 获取人脸特征
                face_embedding = face.embedding
                
                # 计算与所有模板的相似度
                best_match, similarity = self.compare_face(face_embedding)
                
                # 输出匹配结果
                if similarity > self.similarity_threshold:
                    # 从文件名中提取人名（去掉.png后缀）
                    face_name = best_match.split('.')[0]
                    # 将识别到的人脸添加到列表中
                    self.detected_faces.append(face_name)
                    result_msg = f"检测到匹配人脸: {face_name}, 相似度: {similarity:.2f}"
                    print(result_msg)
                    self.face_pub.publish(result_msg)
                    # 在图像上绘制矩形框
                    cv2.rectangle(current_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # 添加标签文本
                    cv2.putText(current_image, f"{face_name} {similarity:.2f}", (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    print(f"未检测到匹配人脸，相似度: {similarity:.2f}, 人脸大小: {face_width}x{face_height}")
            
            # 显示处理后的图像
            # cv2.imshow("Face Detection", current_image)
            # cv2.waitKey(1)
            
        except Exception as e:
            print(f"处理图像时出错: {e}")

def main():
    try:
        face_detect = FaceDetect()
        rate = rospy.Rate(2)  # 设置频率为2Hz
        
        while not rospy.is_shutdown():
            face_detect.process_image()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()


#!/usr/bin/env python3
"""
ROS2 Car Fun - Face Tracking Node
人脸跟踪节点

Detects and tracks human faces using OpenCV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, Int32
import cv2
import numpy as np
from typing import Optional, List, Dict, Tuple
from cv_bridge import CvBridge
import os


class FaceTrackingNode(Node):
    """
    人脸跟踪节点
    
    功能：
    - Haar级联分类器人脸检测
    - 多人脸同时检测
    - 最大人脸跟踪
    - 人脸位置输出
    - 跟踪控制命令
    """
    
    def __init__(self):
        super().__init__('face_tracking_node')
        
        # 声明参数
        self.declare_parameter('cascade_file', 'haarcascade_frontalface_default.xml')
        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('min_size_width', 30)
        self.declare_parameter('min_size_height', 30)
        self.declare_parameter('max_size_width', 300)
        self.declare_parameter('max_size_height', 300)
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('tracking_smoothing', 0.7)
        self.declare_parameter('track_largest_face', True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('enable_recognition', False)  # 人脸识别（需要额外模型）
        self.declare_parameter('debug_mode', False)
        
        # 获取参数
        self.cascade_file = self.get_parameter('cascade_file').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.min_neighbors = self.get_parameter('min_neighbors').value
        self.min_size_width = self.get_parameter('min_size_width').value
        self.min_size_height = self.get_parameter('min_size_height').value
        self.max_size_width = self.get_parameter('max_size_width').value
        self.max_size_height = self.get_parameter('max_size_height').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.tracking_smoothing = self.get_parameter('tracking_smoothing').value
        self.track_largest_face = self.get_parameter('track_largest_face').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.enable_recognition = self.get_parameter('enable_recognition').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 初始化人脸检测器
        self.face_cascade = self._load_cascade()
        
        # 订阅器
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # 发布器
        self.detection_pub = self.create_publisher(Bool, 'face_tracking/detected', 10)
        self.count_pub = self.create_publisher(Int32, 'face_tracking/count', 10)
        self.position_pub = self.create_publisher(Point, 'face_tracking/position', 10)
        self.target_pub = self.create_publisher(PointStamped, 'face_tracking/target', 10)
        
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image, 'face_tracking/debug_image', 10)
        
        # 如果启用跟踪，发布跟踪命令
        if self.enable_tracking:
            self.track_command_pub = self.create_publisher(Point, 'camera/track_target', 10)
        
        # 状态变量
        self.last_faces = []
        self.tracked_face = None
        self.smoothed_position = Point()
        self.detection_count = 0
        self.frame_count = 0
        self.face_id_counter = 0
        
        # 人脸跟踪历史（用于稳定跟踪）
        self.face_history = []
        self.max_history_length = 10
        
        self.get_logger().info('人脸跟踪节点已启动')
    
    def _load_cascade(self) -> Optional[cv2.CascadeClassifier]:
        """加载Haar级联分类器"""
        try:
            # 尝试从OpenCV数据目录加载
            cascade_path = cv2.data.haarcascades + self.cascade_file
            
            if not os.path.exists(cascade_path):
                # 尝试相对路径
                cascade_path = self.cascade_file
            
            if not os.path.exists(cascade_path):
                self.get_logger().error(f'找不到级联文件: {self.cascade_file}')
                return None
            
            cascade = cv2.CascadeClassifier(cascade_path)
            
            if cascade.empty():
                self.get_logger().error(f'级联文件加载失败: {cascade_path}')
                return None
            
            self.get_logger().info(f'人脸检测器加载成功: {cascade_path}')
            return cascade
            
        except Exception as e:
            self.get_logger().error(f'级联加载错误: {e}')
            return None
    
    def image_callback(self, msg: Image):
        """图像回调函数"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # 执行人脸检测
            faces = self.detect_faces(cv_image)
            
            # 处理检测结果
            self.process_faces(faces, cv_image)
            
            # 发布调试图像
            if self.publish_debug_image:
                debug_image = self.create_debug_image(cv_image, faces)
                self.publish_debug_image_msg(debug_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def detect_faces(self, image: np.ndarray) -> List[Dict]:
        """检测人脸"""
        if self.face_cascade is None:
            return []
        
        # 转换为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 直方图均衡化（改善检测效果）
        gray = cv2.equalizeHist(gray)
        
        # 检测人脸
        faces_rect = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=(self.min_size_width, self.min_size_height),
            maxSize=(self.max_size_width, self.max_size_height)
        )
        
        # 转换为字典格式
        faces = []
        for i, (x, y, w, h) in enumerate(faces_rect):
            # 计算中心点
            center_x = x + w // 2
            center_y = y + h // 2
            
            # 计算归一化坐标
            norm_x = center_x / image.shape[1]
            norm_y = center_y / image.shape[0]
            
            face = {
                'id': self.face_id_counter + i,
                'bbox': (x, y, w, h),
                'center': (center_x, center_y),
                'normalized_center': (norm_x, norm_y),
                'area': w * h,
                'confidence': 1.0  # Haar级联没有置信度，设为1.0
            }
            
            faces.append(face)
        
        self.face_id_counter += len(faces)
        
        # 按面积排序（最大的在前）
        faces.sort(key=lambda x: x['area'], reverse=True)
        
        return faces
    
    def process_faces(self, faces: List[Dict], image: np.ndarray):
        """处理人脸检测结果"""
        # 发布检测状态
        detected = len(faces) > 0
        self.publish_detection_status(detected)
        
        # 发布人脸数量
        self.publish_face_count(len(faces))
        
        if detected:
            # 选择跟踪目标
            target_face = self.select_tracking_target(faces)
            
            if target_face:
                # 发布位置信息
                self.publish_position(target_face, image)
                
                # 更新跟踪
                if self.enable_tracking:
                    self.update_tracking(target_face)
                
                self.tracked_face = target_face
                self.detection_count += 1
                
                if self.debug_mode:
                    center = target_face['center']
                    area = target_face['area']
                    self.get_logger().debug(f'跟踪人脸在 {center}, 面积: {area}')
        
        else:
            # 没有检测到人脸
            self.tracked_face = None
        
        # 更新历史记录
        self.update_face_history(faces)
        self.last_faces = faces
    
    def select_tracking_target(self, faces: List[Dict]) -> Optional[Dict]:
        """选择跟踪目标"""
        if not faces:
            return None
        
        if self.track_largest_face:
            # 跟踪最大的人脸
            return faces[0]
        
        # 如果有之前的跟踪目标，尝试匹配最近的人脸
        if self.tracked_face is not None:
            last_center = self.tracked_face['center']
            min_distance = float('inf')
            closest_face = None
            
            for face in faces:
                current_center = face['center']
                distance = np.sqrt((current_center[0] - last_center[0])**2 + 
                                 (current_center[1] - last_center[1])**2)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_face = face
            
            # 如果距离合理，继续跟踪
            if min_distance < 100:  # 像素距离阈值
                return closest_face
        
        # 否则选择最大的人脸
        return faces[0]
    
    def update_face_history(self, faces: List[Dict]):
        """更新人脸历史记录"""
        self.face_history.append(faces)
        
        if len(self.face_history) > self.max_history_length:
            self.face_history.pop(0)
    
    def publish_detection_status(self, detected: bool):
        """发布检测状态"""
        msg = Bool()
        msg.data = detected
        self.detection_pub.publish(msg)
    
    def publish_face_count(self, count: int):
        """发布人脸数量"""
        msg = Int32()
        msg.data = count
        self.count_pub.publish(msg)
    
    def publish_position(self, face: Dict, image: np.ndarray):
        """发布位置信息"""
        # 发布像素坐标
        position_msg = Point()
        position_msg.x = float(face['center'][0])
        position_msg.y = float(face['center'][1])
        position_msg.z = float(face['area'])
        self.position_pub.publish(position_msg)
        
        # 发布归一化坐标
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'camera_frame'
        target_msg.point.x = face['normalized_center'][0]
        target_msg.point.y = face['normalized_center'][1]
        target_msg.point.z = 0.0
        self.target_pub.publish(target_msg)
    
    def update_tracking(self, face: Dict):
        """更新跟踪"""
        # 平滑处理
        if self.tracked_face is not None:
            # 指数移动平均
            alpha = 1.0 - self.tracking_smoothing
            self.smoothed_position.x = (alpha * face['normalized_center'][0] + 
                                      self.tracking_smoothing * self.smoothed_position.x)
            self.smoothed_position.y = (alpha * face['normalized_center'][1] + 
                                      self.tracking_smoothing * self.smoothed_position.y)
        else:
            self.smoothed_position.x = face['normalized_center'][0]
            self.smoothed_position.y = face['normalized_center'][1]
        
        # 发布跟踪命令
        track_msg = Point()
        track_msg.x = self.smoothed_position.x
        track_msg.y = self.smoothed_position.y
        track_msg.z = 0.0
        self.track_command_pub.publish(track_msg)
    
    def create_debug_image(self, image: np.ndarray, faces: List[Dict]) -> np.ndarray:
        """创建调试图像"""
        debug_image = image.copy()
        
        # 绘制所有检测到的人脸
        for i, face in enumerate(faces):
            x, y, w, h = face['bbox']
            center = face['center']
            area = face['area']
            
            # 选择颜色（跟踪目标用红色，其他用绿色）
            if self.tracked_face and face['id'] == self.tracked_face['id']:
                color = (0, 0, 255)  # 红色
                thickness = 3
            else:
                color = (0, 255, 0)  # 绿色
                thickness = 2
            
            # 绘制边界框
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, thickness)
            
            # 绘制中心点
            cv2.circle(debug_image, center, 5, color, -1)
            
            # 绘制标签
            label = f"Face {i+1} ({area})"
            cv2.putText(debug_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 如果是跟踪目标，添加特殊标记
            if self.tracked_face and face['id'] == self.tracked_face['id']:
                cv2.putText(debug_image, "TRACKING", (x, y + h + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 绘制十字线（图像中心）
        h, w = debug_image.shape[:2]
        cv2.line(debug_image, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 255), 1)
        cv2.line(debug_image, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 255), 1)
        
        # 添加信息文本
        info_text = f"Faces: {len(faces)} | Frame: {self.frame_count}"
        cv2.putText(debug_image, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if self.tracked_face:
            track_info = f"Tracking: Face at {self.tracked_face['center']}"
            cv2.putText(debug_image, track_info, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # 显示平滑后的跟踪位置
        if self.enable_tracking and self.tracked_face:
            smooth_x = int(self.smoothed_position.x * w)
            smooth_y = int(self.smoothed_position.y * h)
            cv2.circle(debug_image, (smooth_x, smooth_y), 8, (255, 0, 255), 2)
            cv2.putText(debug_image, "SMOOTH", (smooth_x - 30, smooth_y - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        
        return debug_image
    
    def publish_debug_image_msg(self, debug_image: np.ndarray, header):
        """发布调试图像消息"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'调试图像发布失败: {e}')
    
    def get_face_statistics(self) -> Dict:
        """获取人脸检测统计信息"""
        return {
            'frame_count': self.frame_count,
            'detection_count': self.detection_count,
            'current_faces': len(self.last_faces),
            'tracking_enabled': self.enable_tracking,
            'tracked_face': self.tracked_face is not None,
            'cascade_loaded': self.face_cascade is not None,
            'smoothed_position': {
                'x': self.smoothed_position.x,
                'y': self.smoothed_position.y
            } if self.tracked_face else None
        }
    
    def reset_tracking(self):
        """重置跟踪状态"""
        self.tracked_face = None
        self.smoothed_position = Point()
        self.face_history.clear()
        self.get_logger().info('人脸跟踪状态已重置')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    face_tracking_node = FaceTrackingNode()
    
    try:
        rclpy.spin(face_tracking_node)
    except KeyboardInterrupt:
        face_tracking_node.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        face_tracking_node.get_logger().error(f'人脸跟踪节点运行错误: {e}')
    finally:
        face_tracking_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
ROS2 Car Fun - Object Detection Node
物体检测节点

Simple object detection using contour analysis and template matching
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Bool, Int32
import cv2
import numpy as np
from typing import Optional, List, Dict, Tuple
from cv_bridge import CvBridge


class ObjectDetectionNode(Node):
    """
    物体检测节点
    
    功能：
    - 基于轮廓的物体检测
    - 形状识别（圆形、矩形、三角形）
    - 大小分类
    - 位置跟踪
    - 可视化结果
    """
    
    def __init__(self):
        super().__init__('object_detection_node')
        
        # 声明参数
        self.declare_parameter('detection_method', 'contour')  # 'contour', 'template'
        self.declare_parameter('min_area', 1000)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('min_perimeter', 100)
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('canny_threshold1', 50)
        self.declare_parameter('canny_threshold2', 150)
        self.declare_parameter('enable_shape_detection', True)
        self.declare_parameter('enable_tracking', False)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_mode', False)
        
        # 获取参数
        self.detection_method = self.get_parameter('detection_method').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.min_perimeter = self.get_parameter('min_perimeter').value
        self.blur_kernel_size = self.get_parameter('blur_kernel_size').value
        self.canny_threshold1 = self.get_parameter('canny_threshold1').value
        self.canny_threshold2 = self.get_parameter('canny_threshold2').value
        self.enable_shape_detection = self.get_parameter('enable_shape_detection').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 订阅器
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # 发布器
        self.detection_pub = self.create_publisher(Bool, 'object_detection/detected', 10)
        self.count_pub = self.create_publisher(Int32, 'object_detection/count', 10)
        self.shape_pub = self.create_publisher(String, 'object_detection/shape', 10)
        self.position_pub = self.create_publisher(Point, 'object_detection/position', 10)
        self.target_pub = self.create_publisher(PointStamped, 'object_detection/target', 10)
        
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image, 'object_detection/debug_image', 10)
        
        if self.enable_tracking:
            self.track_command_pub = self.create_publisher(Point, 'camera/track_target', 10)
        
        # 状态变量
        self.last_objects = []
        self.detection_count = 0
        self.frame_count = 0
        self.tracked_object = None
        
        self.get_logger().info(f'物体检测节点已启动 (方法: {self.detection_method})')
    
    def image_callback(self, msg: Image):
        """图像回调函数"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # 执行物体检测
            objects = self.detect_objects(cv_image)
            
            # 处理检测结果
            self.process_objects(objects, cv_image)
            
            # 发布调试图像
            if self.publish_debug_image:
                debug_image = self.create_debug_image(cv_image, objects)
                self.publish_debug_image_msg(debug_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """检测物体"""
        if self.detection_method == 'contour':
            return self._detect_with_contours(image)
        elif self.detection_method == 'template':
            return self._detect_with_template(image)
        else:
            return []
    
    def _detect_with_contours(self, image: np.ndarray) -> List[Dict]:
        """使用轮廓检测物体"""
        objects = []
        
        # 转换为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 高斯模糊
        if self.blur_kernel_size > 0:
            gray = cv2.GaussianBlur(gray, (self.blur_kernel_size, self.blur_kernel_size), 0)
        
        # Canny边缘检测
        edges = cv2.Canny(gray, self.canny_threshold1, self.canny_threshold2)
        
        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for i, contour in enumerate(contours):
            # 计算轮廓属性
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            
            # 过滤小物体
            if area < self.min_area or area > self.max_area:
                continue
            if perimeter < self.min_perimeter:
                continue
            
            # 计算边界框
            x, y, w, h = cv2.boundingRect(contour)
            
            # 计算中心点
            center_x = x + w // 2
            center_y = y + h // 2
            
            # 计算归一化坐标
            norm_x = center_x / image.shape[1]
            norm_y = center_y / image.shape[0]
            
            # 形状检测
            shape = 'unknown'
            if self.enable_shape_detection:
                shape = self._detect_shape(contour)
            
            # 计算物体属性
            aspect_ratio = w / h if h > 0 else 0
            extent = area / (w * h) if w > 0 and h > 0 else 0
            solidity = area / cv2.contourArea(cv2.convexHull(contour)) if cv2.contourArea(cv2.convexHull(contour)) > 0 else 0
            
            obj = {
                'id': i,
                'contour': contour,
                'bbox': (x, y, w, h),
                'center': (center_x, center_y),
                'normalized_center': (norm_x, norm_y),
                'area': area,
                'perimeter': perimeter,
                'shape': shape,
                'aspect_ratio': aspect_ratio,
                'extent': extent,
                'solidity': solidity
            }
            
            objects.append(obj)
        
        # 按面积排序
        objects.sort(key=lambda x: x['area'], reverse=True)
        
        return objects
    
    def _detect_with_template(self, image: np.ndarray) -> List[Dict]:
        """使用模板匹配检测物体（简化实现）"""
        # 这里可以实现模板匹配算法
        # 目前返回空列表
        return []
    
    def _detect_shape(self, contour: np.ndarray) -> str:
        """检测形状"""
        # 近似轮廓
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # 根据顶点数量判断形状
        vertices = len(approx)
        
        if vertices == 3:
            return 'triangle'
        elif vertices == 4:
            # 检查是否为矩形
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h if h > 0 else 0
            
            if 0.8 <= aspect_ratio <= 1.2:
                return 'square'
            else:
                return 'rectangle'
        elif vertices > 8:
            # 检查是否为圆形
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity > 0.7:
                    return 'circle'
        
        return 'polygon'
    
    def process_objects(self, objects: List[Dict], image: np.ndarray):
        """处理物体检测结果"""
        # 发布检测状态
        detected = len(objects) > 0
        self.publish_detection_status(detected)
        
        # 发布物体数量
        self.publish_object_count(len(objects))
        
        if detected:
            # 处理最大的物体
            main_object = objects[0]
            
            # 发布形状信息
            self.publish_shape(main_object['shape'])
            
            # 发布位置信息
            self.publish_position(main_object, image)
            
            # 更新跟踪
            if self.enable_tracking:
                self.update_tracking(main_object)
            
            self.tracked_object = main_object
            self.detection_count += 1
            
            if self.debug_mode:
                shape = main_object['shape']
                center = main_object['center']
                area = main_object['area']
                self.get_logger().debug(f'检测到 {shape} 在 {center}, 面积: {area}')
        
        else:
            self.tracked_object = None
        
        self.last_objects = objects
    
    def publish_detection_status(self, detected: bool):
        """发布检测状态"""
        msg = Bool()
        msg.data = detected
        self.detection_pub.publish(msg)
    
    def publish_object_count(self, count: int):
        """发布物体数量"""
        msg = Int32()
        msg.data = count
        self.count_pub.publish(msg)
    
    def publish_shape(self, shape: str):
        """发布形状信息"""
        msg = String()
        msg.data = shape
        self.shape_pub.publish(msg)
    
    def publish_position(self, obj: Dict, image: np.ndarray):
        """发布位置信息"""
        # 发布像素坐标
        position_msg = Point()
        position_msg.x = float(obj['center'][0])
        position_msg.y = float(obj['center'][1])
        position_msg.z = float(obj['area'])
        self.position_pub.publish(position_msg)
        
        # 发布归一化坐标
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'camera_frame'
        target_msg.point.x = obj['normalized_center'][0]
        target_msg.point.y = obj['normalized_center'][1]
        target_msg.point.z = 0.0
        self.target_pub.publish(target_msg)
    
    def update_tracking(self, obj: Dict):
        """更新跟踪"""
        # 发布跟踪命令
        track_msg = Point()
        track_msg.x = obj['normalized_center'][0]
        track_msg.y = obj['normalized_center'][1]
        track_msg.z = 0.0
        self.track_command_pub.publish(track_msg)
    
    def create_debug_image(self, image: np.ndarray, objects: List[Dict]) -> np.ndarray:
        """创建调试图像"""
        debug_image = image.copy()
        
        # 绘制检测结果
        for i, obj in enumerate(objects):
            contour = obj['contour']
            bbox = obj['bbox']
            center = obj['center']
            shape = obj['shape']
            area = obj['area']
            
            # 选择颜色
            colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
            color = colors[i % len(colors)]
            
            # 绘制轮廓
            cv2.drawContours(debug_image, [contour], -1, color, 2)
            
            # 绘制边界框
            x, y, w, h = bbox
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 1)
            
            # 绘制中心点
            cv2.circle(debug_image, center, 5, color, -1)
            
            # 绘制标签
            label = f"{shape} ({area})"
            cv2.putText(debug_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # 绘制序号
            cv2.putText(debug_image, str(i + 1), (center[0] - 10, center[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # 绘制属性信息
            if self.debug_mode:
                info_y = y + h + 20
                cv2.putText(debug_image, f"AR: {obj['aspect_ratio']:.2f}", 
                           (x, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                cv2.putText(debug_image, f"Sol: {obj['solidity']:.2f}", 
                           (x, info_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # 添加信息文本
        info_text = f"Objects: {len(objects)} | Frame: {self.frame_count}"
        cv2.putText(debug_image, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if self.tracked_object:
            track_info = f"Tracking: {self.tracked_object['shape']} at {self.tracked_object['center']}"
            cv2.putText(debug_image, track_info, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return debug_image
    
    def publish_debug_image_msg(self, debug_image: np.ndarray, header):
        """发布调试图像消息"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'调试图像发布失败: {e}')
    
    def get_detection_statistics(self) -> Dict:
        """获取检测统计信息"""
        return {
            'detection_method': self.detection_method,
            'frame_count': self.frame_count,
            'detection_count': self.detection_count,
            'current_object_count': len(self.last_objects),
            'tracked_object': self.tracked_object is not None,
            'shape_detection_enabled': self.enable_shape_detection,
            'tracking_enabled': self.enable_tracking
        }


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    object_detection_node = ObjectDetectionNode()
    
    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        object_detection_node.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        object_detection_node.get_logger().error(f'物体检测节点运行错误: {e}')
    finally:
        object_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
ROS2 Car Fun - Color Detection Node
颜色检测节点

Detects and tracks colored objects in camera images
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Bool, Int32
import cv2
import numpy as np
from typing import Optional, Tuple, List, Dict
from cv_bridge import CvBridge


class ColorDetectionNode(Node):
    """
    颜色检测节点
    
    功能：
    - HSV颜色空间检测
    - 多颜色同时检测
    - 目标位置计算
    - 跟踪控制输出
    - 可视化结果发布
    """
    
    def __init__(self):
        super().__init__('color_detection_node')
        
        # 声明参数
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('enable_multiple_colors', True)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('morphology_kernel_size', 5)
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('tracking_smoothing', 0.8)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_mode', False)
        
        # 获取参数
        self.target_color = self.get_parameter('target_color').value
        self.enable_multiple_colors = self.get_parameter('enable_multiple_colors').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.blur_kernel_size = self.get_parameter('blur_kernel_size').value
        self.morphology_kernel_size = self.get_parameter('morphology_kernel_size').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.tracking_smoothing = self.get_parameter('tracking_smoothing').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 颜色范围定义 (HSV)
        self.color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255), (170, 50, 50), (180, 255, 255)],  # 红色有两个范围
            'green': [(40, 50, 50), (80, 255, 255)],
            'blue': [(100, 50, 50), (130, 255, 255)],
            'yellow': [(20, 50, 50), (30, 255, 255)],
            'orange': [(10, 50, 50), (20, 255, 255)],
            'purple': [(130, 50, 50), (170, 255, 255)],
            'cyan': [(80, 50, 50), (100, 255, 255)],
            'white': [(0, 0, 200), (180, 30, 255)],
            'black': [(0, 0, 0), (180, 255, 30)]
        }
        
        # 订阅器
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # 发布器
        self.detection_pub = self.create_publisher(Bool, 'color_detection/detected', 10)
        self.position_pub = self.create_publisher(Point, 'color_detection/position', 10)
        self.target_pub = self.create_publisher(PointStamped, 'color_detection/target', 10)
        self.color_pub = self.create_publisher(String, 'color_detection/color', 10)
        self.count_pub = self.create_publisher(Int32, 'color_detection/count', 10)
        
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image, 'color_detection/debug_image', 10)
        
        # 如果启用跟踪，发布跟踪命令
        if self.enable_tracking:
            self.track_command_pub = self.create_publisher(Point, 'camera/track_target', 10)
        
        # 状态变量
        self.last_detection = None
        self.last_position = Point()
        self.detection_count = 0
        self.frame_count = 0
        self.smoothed_position = Point()
        
        self.get_logger().info(f'颜色检测节点已启动 (目标颜色: {self.target_color})')
    
    def image_callback(self, msg: Image):
        """图像回调函数"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # 执行颜色检测
            detections = self.detect_colors(cv_image)
            
            # 处理检测结果
            self.process_detections(detections, cv_image)
            
            # 发布调试图像
            if self.publish_debug_image:
                debug_image = self.create_debug_image(cv_image, detections)
                self.publish_debug_image_msg(debug_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def detect_colors(self, image: np.ndarray) -> List[Dict]:
        """检测颜色"""
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 高斯模糊
        if self.blur_kernel_size > 0:
            hsv = cv2.GaussianBlur(hsv, (self.blur_kernel_size, self.blur_kernel_size), 0)
        
        detections = []
        
        # 检测目标颜色或多种颜色
        colors_to_detect = [self.target_color]
        if self.enable_multiple_colors:
            colors_to_detect = list(self.color_ranges.keys())
        
        for color_name in colors_to_detect:
            if color_name not in self.color_ranges:
                continue
            
            # 创建颜色掩码
            mask = self.create_color_mask(hsv, color_name)
            
            # 形态学操作
            if self.morphology_kernel_size > 0:
                kernel = np.ones((self.morphology_kernel_size, self.morphology_kernel_size), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 处理轮廓
            for contour in contours:
                area = cv2.contourArea(contour)
                
                if self.min_area <= area <= self.max_area:
                    # 计算中心点
                    M = cv2.moments(contour)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        # 计算边界框
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        detection = {
                            'color': color_name,
                            'center': (cx, cy),
                            'area': area,
                            'contour': contour,
                            'bbox': (x, y, w, h),
                            'normalized_center': (cx / image.shape[1], cy / image.shape[0])
                        }
                        
                        detections.append(detection)
        
        # 按面积排序（最大的在前）
        detections.sort(key=lambda x: x['area'], reverse=True)
        
        return detections
    
    def create_color_mask(self, hsv: np.ndarray, color_name: str) -> np.ndarray:
        """创建颜色掩码"""
        color_range = self.color_ranges[color_name]
        
        if len(color_range) == 4:  # 红色有两个范围
            lower1, upper1, lower2, upper2 = color_range
            mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            mask = cv2.bitwise_or(mask1, mask2)
        else:  # 其他颜色只有一个范围
            lower, upper = color_range
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        
        return mask
    
    def process_detections(self, detections: List[Dict], image: np.ndarray):
        """处理检测结果"""
        # 发布检测状态
        detected = len(detections) > 0
        self.publish_detection_status(detected)
        
        # 发布检测数量
        self.publish_detection_count(len(detections))
        
        if detected:
            # 使用最大的检测结果
            best_detection = detections[0]
            
            # 发布颜色信息
            self.publish_detected_color(best_detection['color'])
            
            # 发布位置信息
            self.publish_position(best_detection, image)
            
            # 更新跟踪
            if self.enable_tracking:
                self.update_tracking(best_detection)
            
            self.last_detection = best_detection
            self.detection_count += 1
            
            if self.debug_mode:
                color = best_detection['color']
                center = best_detection['center']
                area = best_detection['area']
                self.get_logger().debug(f'检测到 {color} 在 {center}, 面积: {area}')
        
        else:
            # 没有检测到目标
            self.last_detection = None
    
    def publish_detection_status(self, detected: bool):
        """发布检测状态"""
        msg = Bool()
        msg.data = detected
        self.detection_pub.publish(msg)
    
    def publish_detection_count(self, count: int):
        """发布检测数量"""
        msg = Int32()
        msg.data = count
        self.count_pub.publish(msg)
    
    def publish_detected_color(self, color: str):
        """发布检测到的颜色"""
        msg = String()
        msg.data = color
        self.color_pub.publish(msg)
    
    def publish_position(self, detection: Dict, image: np.ndarray):
        """发布位置信息"""
        # 发布像素坐标
        position_msg = Point()
        position_msg.x = float(detection['center'][0])
        position_msg.y = float(detection['center'][1])
        position_msg.z = float(detection['area'])
        self.position_pub.publish(position_msg)
        
        # 发布归一化坐标
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'camera_frame'
        target_msg.point.x = detection['normalized_center'][0]
        target_msg.point.y = detection['normalized_center'][1]
        target_msg.point.z = 0.0
        self.target_pub.publish(target_msg)
        
        self.last_position = position_msg
    
    def update_tracking(self, detection: Dict):
        """更新跟踪"""
        # 平滑处理
        if self.last_detection is not None:
            # 指数移动平均
            alpha = 1.0 - self.tracking_smoothing
            self.smoothed_position.x = (alpha * detection['normalized_center'][0] + 
                                      self.tracking_smoothing * self.smoothed_position.x)
            self.smoothed_position.y = (alpha * detection['normalized_center'][1] + 
                                      self.tracking_smoothing * self.smoothed_position.y)
        else:
            self.smoothed_position.x = detection['normalized_center'][0]
            self.smoothed_position.y = detection['normalized_center'][1]
        
        # 发布跟踪命令
        track_msg = Point()
        track_msg.x = self.smoothed_position.x
        track_msg.y = self.smoothed_position.y
        track_msg.z = 0.0
        self.track_command_pub.publish(track_msg)
    
    def create_debug_image(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """创建调试图像"""
        debug_image = image.copy()
        
        # 绘制检测结果
        for i, detection in enumerate(detections):
            color = detection['color']
            center = detection['center']
            bbox = detection['bbox']
            area = detection['area']
            
            # 选择绘制颜色
            draw_color = self.get_draw_color(color)
            
            # 绘制边界框
            x, y, w, h = bbox
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), draw_color, 2)
            
            # 绘制中心点
            cv2.circle(debug_image, center, 5, draw_color, -1)
            
            # 绘制标签
            label = f"{color} ({area})"
            cv2.putText(debug_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)
            
            # 绘制序号
            cv2.putText(debug_image, str(i + 1), (center[0] - 10, center[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 绘制十字线（图像中心）
        h, w = debug_image.shape[:2]
        cv2.line(debug_image, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 255), 1)
        cv2.line(debug_image, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 255), 1)
        
        # 添加信息文本
        info_text = f"Detections: {len(detections)} | Frame: {self.frame_count}"
        cv2.putText(debug_image, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if self.last_detection:
            target_info = f"Target: {self.last_detection['color']} at {self.last_detection['center']}"
            cv2.putText(debug_image, target_info, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return debug_image
    
    def get_draw_color(self, color_name: str) -> Tuple[int, int, int]:
        """获取绘制颜色"""
        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'orange': (0, 165, 255),
            'purple': (255, 0, 255),
            'cyan': (255, 255, 0),
            'white': (255, 255, 255),
            'black': (0, 0, 0)
        }
        return color_map.get(color_name, (128, 128, 128))
    
    def publish_debug_image_msg(self, debug_image: np.ndarray, header):
        """发布调试图像消息"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'调试图像发布失败: {e}')
    
    def set_target_color(self, color: str):
        """设置目标颜色"""
        if color in self.color_ranges:
            self.target_color = color
            self.get_logger().info(f'目标颜色已设置为: {color}')
        else:
            self.get_logger().warn(f'未知颜色: {color}')
    
    def add_custom_color(self, name: str, hsv_range: Tuple):
        """添加自定义颜色"""
        self.color_ranges[name] = hsv_range
        self.get_logger().info(f'添加自定义颜色: {name}')
    
    def get_detection_statistics(self) -> Dict:
        """获取检测统计信息"""
        return {
            'target_color': self.target_color,
            'frame_count': self.frame_count,
            'detection_count': self.detection_count,
            'last_detection': self.last_detection is not None,
            'last_position': {
                'x': self.last_position.x,
                'y': self.last_position.y,
                'area': self.last_position.z
            } if self.last_detection else None,
            'tracking_enabled': self.enable_tracking,
            'multiple_colors_enabled': self.enable_multiple_colors
        }


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    color_detection_node = ColorDetectionNode()
    
    try:
        rclpy.spin(color_detection_node)
    except KeyboardInterrupt:
        color_detection_node.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        color_detection_node.get_logger().error(f'颜色检测节点运行错误: {e}')
    finally:
        color_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

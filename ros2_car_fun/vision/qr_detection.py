#!/usr/bin/env python3
"""
ROS2 Car Fun - QR Code Detection Node
二维码检测节点

Detects and decodes QR codes using OpenCV and pyzbar
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

# 尝试导入pyzbar（可选依赖）
try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False
    print("Warning: pyzbar not available, using OpenCV QR detector only")


class QRDetectionNode(Node):
    """
    二维码检测节点
    
    功能：
    - QR码检测和解码
    - 多种检测方法（OpenCV + pyzbar）
    - 位置和内容输出
    - 命令解析和执行
    - 可视化结果
    """
    
    def __init__(self):
        super().__init__('qr_detection_node')
        
        # 声明参数
        self.declare_parameter('detection_method', 'opencv')  # 'opencv', 'pyzbar', 'both'
        self.declare_parameter('enable_command_parsing', True)
        self.declare_parameter('command_prefix', 'CAR:')
        self.declare_parameter('enable_tracking', False)
        self.declare_parameter('min_area', 100)
        self.declare_parameter('max_area', 100000)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_mode', False)
        
        # 获取参数
        self.detection_method = self.get_parameter('detection_method').value
        self.enable_command_parsing = self.get_parameter('enable_command_parsing').value
        self.command_prefix = self.get_parameter('command_prefix').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.publish_debug_image = self.get_parameter('publish_debug_image').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 初始化QR检测器
        self.qr_detector = None
        self._init_detectors()
        
        # 订阅器
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # 发布器
        self.detection_pub = self.create_publisher(Bool, 'qr_detection/detected', 10)
        self.count_pub = self.create_publisher(Int32, 'qr_detection/count', 10)
        self.content_pub = self.create_publisher(String, 'qr_detection/content', 10)
        self.position_pub = self.create_publisher(Point, 'qr_detection/position', 10)
        self.target_pub = self.create_publisher(PointStamped, 'qr_detection/target', 10)
        
        if self.enable_command_parsing:
            self.command_pub = self.create_publisher(String, 'qr_detection/command', 10)
        
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image, 'qr_detection/debug_image', 10)
        
        if self.enable_tracking:
            self.track_command_pub = self.create_publisher(Point, 'camera/track_target', 10)
        
        # 状态变量
        self.last_qr_codes = []
        self.detection_count = 0
        self.frame_count = 0
        self.last_command = None
        
        # QR码历史记录（用于去重）
        self.qr_history = []
        self.max_history_length = 10
        
        self.get_logger().info(f'二维码检测节点已启动 (方法: {self.detection_method})')
    
    def _init_detectors(self):
        """初始化检测器"""
        if self.detection_method in ['opencv', 'both']:
            try:
                self.qr_detector = cv2.QRCodeDetector()
                self.get_logger().info('OpenCV QR检测器初始化成功')
            except Exception as e:
                self.get_logger().error(f'OpenCV QR检测器初始化失败: {e}')
        
        if self.detection_method in ['pyzbar', 'both']:
            if not PYZBAR_AVAILABLE:
                self.get_logger().warn('pyzbar不可用，请安装: pip install pyzbar')
                if self.detection_method == 'pyzbar':
                    self.detection_method = 'opencv'
    
    def image_callback(self, msg: Image):
        """图像回调函数"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # 执行QR码检测
            qr_codes = self.detect_qr_codes(cv_image)
            
            # 处理检测结果
            self.process_qr_codes(qr_codes, cv_image)
            
            # 发布调试图像
            if self.publish_debug_image:
                debug_image = self.create_debug_image(cv_image, qr_codes)
                self.publish_debug_image_msg(debug_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def detect_qr_codes(self, image: np.ndarray) -> List[Dict]:
        """检测QR码"""
        qr_codes = []
        
        # OpenCV检测
        if self.detection_method in ['opencv', 'both']:
            opencv_codes = self._detect_with_opencv(image)
            qr_codes.extend(opencv_codes)
        
        # pyzbar检测
        if self.detection_method in ['pyzbar', 'both'] and PYZBAR_AVAILABLE:
            pyzbar_codes = self._detect_with_pyzbar(image)
            qr_codes.extend(pyzbar_codes)
        
        # 去重（基于内容和位置）
        qr_codes = self._remove_duplicates(qr_codes)
        
        # 按面积排序
        qr_codes.sort(key=lambda x: x['area'], reverse=True)
        
        return qr_codes
    
    def _detect_with_opencv(self, image: np.ndarray) -> List[Dict]:
        """使用OpenCV检测QR码"""
        qr_codes = []
        
        if self.qr_detector is None:
            return qr_codes
        
        try:
            # 检测和解码
            data, points, _ = self.qr_detector.detectAndDecode(image)
            
            if data and points is not None:
                # 计算边界框
                points = points[0].astype(int)
                x, y, w, h = cv2.boundingRect(points)
                
                # 计算中心点
                center_x = x + w // 2
                center_y = y + h // 2
                
                # 计算归一化坐标
                norm_x = center_x / image.shape[1]
                norm_y = center_y / image.shape[0]
                
                qr_code = {
                    'method': 'opencv',
                    'data': data,
                    'points': points,
                    'bbox': (x, y, w, h),
                    'center': (center_x, center_y),
                    'normalized_center': (norm_x, norm_y),
                    'area': w * h,
                    'confidence': 1.0
                }
                
                qr_codes.append(qr_code)
                
        except Exception as e:
            if self.debug_mode:
                self.get_logger().debug(f'OpenCV QR检测错误: {e}')
        
        return qr_codes
    
    def _detect_with_pyzbar(self, image: np.ndarray) -> List[Dict]:
        """使用pyzbar检测QR码"""
        qr_codes = []
        
        if not PYZBAR_AVAILABLE:
            return qr_codes
        
        try:
            # 转换为灰度图
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 检测条码
            barcodes = pyzbar.decode(gray)
            
            for barcode in barcodes:
                # 只处理QR码
                if barcode.type == 'QRCODE':
                    # 获取数据
                    data = barcode.data.decode('utf-8')
                    
                    # 获取位置信息
                    points = np.array([[point.x, point.y] for point in barcode.polygon])
                    x, y, w, h = cv2.boundingRect(points)
                    
                    # 计算中心点
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # 计算归一化坐标
                    norm_x = center_x / image.shape[1]
                    norm_y = center_y / image.shape[0]
                    
                    qr_code = {
                        'method': 'pyzbar',
                        'data': data,
                        'points': points,
                        'bbox': (x, y, w, h),
                        'center': (center_x, center_y),
                        'normalized_center': (norm_x, norm_y),
                        'area': w * h,
                        'confidence': barcode.quality if hasattr(barcode, 'quality') else 1.0
                    }
                    
                    qr_codes.append(qr_code)
                    
        except Exception as e:
            if self.debug_mode:
                self.get_logger().debug(f'pyzbar QR检测错误: {e}')
        
        return qr_codes
    
    def _remove_duplicates(self, qr_codes: List[Dict]) -> List[Dict]:
        """去除重复的QR码"""
        unique_codes = []
        
        for code in qr_codes:
            is_duplicate = False
            
            for unique_code in unique_codes:
                # 检查内容是否相同
                if code['data'] == unique_code['data']:
                    # 检查位置是否接近
                    distance = np.sqrt((code['center'][0] - unique_code['center'][0])**2 + 
                                     (code['center'][1] - unique_code['center'][1])**2)
                    
                    if distance < 50:  # 像素距离阈值
                        is_duplicate = True
                        break
            
            if not is_duplicate:
                unique_codes.append(code)
        
        return unique_codes
    
    def process_qr_codes(self, qr_codes: List[Dict], image: np.ndarray):
        """处理QR码检测结果"""
        # 发布检测状态
        detected = len(qr_codes) > 0
        self.publish_detection_status(detected)
        
        # 发布QR码数量
        self.publish_qr_count(len(qr_codes))
        
        if detected:
            # 处理最大的QR码
            main_qr = qr_codes[0]
            
            # 检查面积是否在有效范围内
            if self.min_area <= main_qr['area'] <= self.max_area:
                # 发布内容
                self.publish_content(main_qr['data'])
                
                # 发布位置
                self.publish_position(main_qr, image)
                
                # 解析命令
                if self.enable_command_parsing:
                    self.parse_and_publish_command(main_qr['data'])
                
                # 更新跟踪
                if self.enable_tracking:
                    self.update_tracking(main_qr)
                
                self.detection_count += 1
                
                if self.debug_mode:
                    self.get_logger().debug(f'检测到QR码: "{main_qr["data"]}" 在 {main_qr["center"]}')
        
        # 更新历史记录
        self.update_qr_history(qr_codes)
        self.last_qr_codes = qr_codes
    
    def publish_detection_status(self, detected: bool):
        """发布检测状态"""
        msg = Bool()
        msg.data = detected
        self.detection_pub.publish(msg)
    
    def publish_qr_count(self, count: int):
        """发布QR码数量"""
        msg = Int32()
        msg.data = count
        self.count_pub.publish(msg)
    
    def publish_content(self, content: str):
        """发布QR码内容"""
        msg = String()
        msg.data = content
        self.content_pub.publish(msg)
    
    def publish_position(self, qr_code: Dict, image: np.ndarray):
        """发布位置信息"""
        # 发布像素坐标
        position_msg = Point()
        position_msg.x = float(qr_code['center'][0])
        position_msg.y = float(qr_code['center'][1])
        position_msg.z = float(qr_code['area'])
        self.position_pub.publish(position_msg)
        
        # 发布归一化坐标
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'camera_frame'
        target_msg.point.x = qr_code['normalized_center'][0]
        target_msg.point.y = qr_code['normalized_center'][1]
        target_msg.point.z = 0.0
        self.target_pub.publish(target_msg)
    
    def parse_and_publish_command(self, content: str):
        """解析并发布命令"""
        if content.startswith(self.command_prefix):
            command = content[len(self.command_prefix):].strip()
            
            # 发布命令
            command_msg = String()
            command_msg.data = command
            self.command_pub.publish(command_msg)
            
            self.last_command = command
            
            if self.debug_mode:
                self.get_logger().debug(f'解析命令: {command}')
    
    def update_tracking(self, qr_code: Dict):
        """更新跟踪"""
        # 发布跟踪命令
        track_msg = Point()
        track_msg.x = qr_code['normalized_center'][0]
        track_msg.y = qr_code['normalized_center'][1]
        track_msg.z = 0.0
        self.track_command_pub.publish(track_msg)
    
    def update_qr_history(self, qr_codes: List[Dict]):
        """更新QR码历史记录"""
        self.qr_history.append(qr_codes)
        
        if len(self.qr_history) > self.max_history_length:
            self.qr_history.pop(0)
    
    def create_debug_image(self, image: np.ndarray, qr_codes: List[Dict]) -> np.ndarray:
        """创建调试图像"""
        debug_image = image.copy()
        
        # 绘制检测结果
        for i, qr_code in enumerate(qr_codes):
            points = qr_code['points']
            center = qr_code['center']
            data = qr_code['data']
            method = qr_code['method']
            area = qr_code['area']
            
            # 选择颜色
            if method == 'opencv':
                color = (0, 255, 0)  # 绿色
            else:
                color = (255, 0, 0)  # 蓝色
            
            # 绘制轮廓
            if len(points) >= 4:
                cv2.polylines(debug_image, [points], True, color, 2)
            else:
                # 如果没有轮廓点，绘制边界框
                x, y, w, h = qr_code['bbox']
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 2)
            
            # 绘制中心点
            cv2.circle(debug_image, center, 5, color, -1)
            
            # 绘制标签
            label = f"{method.upper()} ({area})"
            cv2.putText(debug_image, label, (center[0] - 50, center[1] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # 绘制内容（截断长文本）
            display_data = data[:20] + "..." if len(data) > 20 else data
            cv2.putText(debug_image, display_data, (center[0] - 50, center[1] + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # 绘制序号
            cv2.putText(debug_image, str(i + 1), (center[0] - 10, center[1] + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # 添加信息文本
        info_text = f"QR Codes: {len(qr_codes)} | Frame: {self.frame_count}"
        cv2.putText(debug_image, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if self.last_command:
            command_text = f"Last Command: {self.last_command}"
            cv2.putText(debug_image, command_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return debug_image
    
    def publish_debug_image_msg(self, debug_image: np.ndarray, header):
        """发布调试图像消息"""
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'调试图像发布失败: {e}')
    
    def get_qr_statistics(self) -> Dict:
        """获取QR检测统计信息"""
        return {
            'detection_method': self.detection_method,
            'frame_count': self.frame_count,
            'detection_count': self.detection_count,
            'current_qr_count': len(self.last_qr_codes),
            'last_command': self.last_command,
            'command_parsing_enabled': self.enable_command_parsing,
            'tracking_enabled': self.enable_tracking,
            'pyzbar_available': PYZBAR_AVAILABLE
        }


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    qr_detection_node = QRDetectionNode()
    
    try:
        rclpy.spin(qr_detection_node)
    except KeyboardInterrupt:
        qr_detection_node.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        qr_detection_node.get_logger().error(f'二维码检测节点运行错误: {e}')
    finally:
        qr_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

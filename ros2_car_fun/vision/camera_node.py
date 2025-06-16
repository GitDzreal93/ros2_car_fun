#!/usr/bin/env python3
"""
ROS2 Car Fun - Camera Node
摄像头节点

Captures and publishes camera images with servo control for pan/tilt
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point
import cv2
import numpy as np
import time
from typing import Optional, Tuple
from cv_bridge import CvBridge

from ..hardware import HardwareInterface, CarMode


class CameraNode(Node):
    """
    摄像头节点
    
    功能：
    - 摄像头图像采集和发布
    - 云台控制（pan/tilt）
    - 图像参数调节
    - 多种图像格式支持
    - 自动跟踪模式
    """
    
    def __init__(self):
        super().__init__('camera_node')
        
        # 声明参数
        self.declare_parameter('hardware_mode', 'simulation')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('publish_frequency', 30.0)
        self.declare_parameter('enable_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('brightness', 50)
        self.declare_parameter('contrast', 50)
        self.declare_parameter('saturation', 50)
        self.declare_parameter('enable_servo_control', True)
        self.declare_parameter('pan_center', 90)
        self.declare_parameter('tilt_center', 90)
        self.declare_parameter('pan_range', 60)  # ±60度
        self.declare_parameter('tilt_range', 30)  # ±30度
        self.declare_parameter('servo_speed', 2.0)  # 度/秒
        self.declare_parameter('debug_mode', False)
        
        # 获取参数
        self.hardware_mode = self.get_parameter('hardware_mode').value
        self.camera_index = self.get_parameter('camera_index').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.enable_compressed = self.get_parameter('enable_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.auto_exposure = self.get_parameter('auto_exposure').value
        self.brightness = self.get_parameter('brightness').value
        self.contrast = self.get_parameter('contrast').value
        self.saturation = self.get_parameter('saturation').value
        self.enable_servo_control = self.get_parameter('enable_servo_control').value
        self.pan_center = self.get_parameter('pan_center').value
        self.tilt_center = self.get_parameter('tilt_center').value
        self.pan_range = self.get_parameter('pan_range').value
        self.tilt_range = self.get_parameter('tilt_range').value
        self.servo_speed = self.get_parameter('servo_speed').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 初始化硬件接口
        mode = CarMode.HARDWARE if self.hardware_mode == 'hardware' else CarMode.SIMULATION
        self.hardware = HardwareInterface(mode, debug=self.debug_mode)
        
        # 初始化摄像头
        self.camera = None
        self.bridge = CvBridge()
        self._init_camera()
        
        # 发布器
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        if self.enable_compressed:
            self.compressed_pub = self.create_publisher(CompressedImage, 'camera/image_compressed', 10)
        
        # 订阅器 - 云台控制
        if self.enable_servo_control:
            self.pan_sub = self.create_subscription(
                Float32, 'camera/pan_command', self.pan_callback, 10)
            self.tilt_sub = self.create_subscription(
                Float32, 'camera/tilt_command', self.tilt_callback, 10)
            self.track_sub = self.create_subscription(
                Point, 'camera/track_target', self.track_callback, 10)
        
        # 状态变量
        self.current_pan = self.pan_center
        self.current_tilt = self.tilt_center
        self.last_frame = None
        self.frame_count = 0
        self.error_count = 0
        self.tracking_enabled = False
        self.track_target = None
        
        # 定时器
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        
        # 云台控制定时器
        if self.enable_servo_control:
            self.servo_timer = self.create_timer(0.1, self.update_servo_position)
            # 初始化云台位置
            self.hardware.center_camera()
        
        self.get_logger().info(f'摄像头节点已启动 (模式: {self.hardware_mode}, 分辨率: {self.image_width}x{self.image_height})')
    
    def _init_camera(self):
        """初始化摄像头"""
        if self.hardware_mode == 'simulation':
            # 仿真模式 - 创建虚拟图像
            self.get_logger().info('使用仿真摄像头')
            return
        
        try:
            self.camera = cv2.VideoCapture(self.camera_index)
            
            if not self.camera.isOpened():
                raise RuntimeError(f"无法打开摄像头 {self.camera_index}")
            
            # 设置摄像头参数
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.camera.set(cv2.CAP_PROP_FPS, self.fps)
            
            if not self.auto_exposure:
                self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            
            self.camera.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness / 100.0)
            self.camera.set(cv2.CAP_PROP_CONTRAST, self.contrast / 100.0)
            self.camera.set(cv2.CAP_PROP_SATURATION, self.saturation / 100.0)
            
            self.get_logger().info(f'摄像头初始化成功: {self.image_width}x{self.image_height}@{self.fps}fps')
            
        except Exception as e:
            self.get_logger().error(f'摄像头初始化失败: {e}')
            self.camera = None
    
    def _generate_simulation_image(self) -> np.ndarray:
        """生成仿真图像"""
        # 创建彩色测试图像
        image = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        # 添加渐变背景
        for y in range(self.image_height):
            for x in range(self.image_width):
                image[y, x] = [
                    int(255 * x / self.image_width),  # R
                    int(255 * y / self.image_height), # G
                    128  # B
                ]
        
        # 添加一些几何图形
        center_x, center_y = self.image_width // 2, self.image_height // 2
        
        # 圆形
        cv2.circle(image, (center_x, center_y), 50, (0, 255, 0), 2)
        
        # 矩形
        cv2.rectangle(image, (center_x - 100, center_y - 30), 
                     (center_x + 100, center_y + 30), (255, 0, 0), 2)
        
        # 添加时间戳
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(image, f"SIM {timestamp}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 添加云台位置信息
        if self.enable_servo_control:
            servo_text = f"Pan:{self.current_pan:.1f} Tilt:{self.current_tilt:.1f}"
            cv2.putText(image, servo_text, (10, self.image_height - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # 添加跟踪目标
        if self.tracking_enabled and self.track_target:
            target_x = int(self.track_target.x * self.image_width)
            target_y = int(self.track_target.y * self.image_height)
            cv2.circle(image, (target_x, target_y), 20, (0, 0, 255), 3)
            cv2.putText(image, "TARGET", (target_x - 30, target_y - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        return image
    
    def capture_and_publish(self):
        """采集并发布图像"""
        try:
            # 获取图像
            if self.hardware_mode == 'simulation':
                frame = self._generate_simulation_image()
                success = True
            else:
                if self.camera is None:
                    return
                success, frame = self.camera.read()
            
            if not success or frame is None:
                self.error_count += 1
                if self.debug_mode:
                    self.get_logger().warn('图像采集失败')
                return
            
            self.frame_count += 1
            self.last_frame = frame.copy()
            
            # 发布原始图像
            self._publish_raw_image(frame)
            
            # 发布压缩图像
            if self.enable_compressed:
                self._publish_compressed_image(frame)
            
            if self.debug_mode and self.frame_count % 30 == 0:
                self.get_logger().debug(f'已发布 {self.frame_count} 帧图像')
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'图像处理错误: {e}')
    
    def _publish_raw_image(self, frame: np.ndarray):
        """发布原始图像"""
        try:
            # 转换为ROS图像消息
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_frame'
            
            self.image_pub.publish(image_msg)
            
        except Exception as e:
            self.get_logger().error(f'原始图像发布失败: {e}')
    
    def _publish_compressed_image(self, frame: np.ndarray):
        """发布压缩图像"""
        try:
            # JPEG压缩
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
            
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.header.frame_id = 'camera_frame'
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_image.tobytes()
                
                self.compressed_pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'压缩图像发布失败: {e}')
    
    def pan_callback(self, msg: Float32):
        """云台水平控制回调"""
        if not self.enable_servo_control:
            return
        
        target_pan = max(self.pan_center - self.pan_range, 
                        min(self.pan_center + self.pan_range, msg.data))
        self.current_pan = target_pan
        
        if self.debug_mode:
            self.get_logger().debug(f'云台水平目标: {target_pan}°')
    
    def tilt_callback(self, msg: Float32):
        """云台垂直控制回调"""
        if not self.enable_servo_control:
            return
        
        target_tilt = max(self.tilt_center - self.tilt_range, 
                         min(self.tilt_center + self.tilt_range, msg.data))
        self.current_tilt = target_tilt
        
        if self.debug_mode:
            self.get_logger().debug(f'云台垂直目标: {target_tilt}°')
    
    def track_callback(self, msg: Point):
        """目标跟踪回调"""
        if not self.enable_servo_control:
            return
        
        self.track_target = msg
        self.tracking_enabled = True
        
        # 计算云台控制量
        # 图像中心为(0.5, 0.5)，目标偏移转换为云台角度
        error_x = msg.x - 0.5  # 水平误差
        error_y = msg.y - 0.5  # 垂直误差
        
        # 比例控制
        pan_adjustment = error_x * 30  # 最大30度调整
        tilt_adjustment = -error_y * 20  # 最大20度调整（负号因为图像坐标系）
        
        target_pan = self.current_pan + pan_adjustment
        target_tilt = self.current_tilt + tilt_adjustment
        
        # 限制范围
        target_pan = max(self.pan_center - self.pan_range, 
                        min(self.pan_center + self.pan_range, target_pan))
        target_tilt = max(self.tilt_center - self.tilt_range, 
                         min(self.tilt_center + self.tilt_range, target_tilt))
        
        self.current_pan = target_pan
        self.current_tilt = target_tilt
        
        if self.debug_mode:
            self.get_logger().debug(f'跟踪目标: ({msg.x:.2f}, {msg.y:.2f}) -> Pan:{target_pan:.1f}° Tilt:{target_tilt:.1f}°')
    
    def update_servo_position(self):
        """更新云台位置"""
        if not self.enable_servo_control:
            return
        
        try:
            self.hardware.set_camera_pan(self.current_pan)
            self.hardware.set_camera_tilt(self.current_tilt)
            
        except Exception as e:
            self.get_logger().error(f'云台控制错误: {e}')
    
    def get_current_frame(self) -> Optional[np.ndarray]:
        """获取当前帧"""
        return self.last_frame
    
    def get_camera_info(self) -> dict:
        """获取摄像头信息"""
        return {
            'width': self.image_width,
            'height': self.image_height,
            'fps': self.fps,
            'frame_count': self.frame_count,
            'error_count': self.error_count,
            'current_pan': self.current_pan,
            'current_tilt': self.current_tilt,
            'tracking_enabled': self.tracking_enabled,
            'hardware_mode': self.hardware_mode
        }
    
    def reset_camera_position(self):
        """重置云台位置"""
        if self.enable_servo_control:
            self.current_pan = self.pan_center
            self.current_tilt = self.tilt_center
            self.tracking_enabled = False
            self.track_target = None
            self.hardware.center_camera()
            self.get_logger().info('云台位置已重置')
    
    def set_camera_parameters(self, brightness: int = None, contrast: int = None, 
                             saturation: int = None):
        """设置摄像头参数"""
        if self.camera is None or self.hardware_mode == 'simulation':
            return
        
        try:
            if brightness is not None:
                self.camera.set(cv2.CAP_PROP_BRIGHTNESS, brightness / 100.0)
                self.brightness = brightness
            
            if contrast is not None:
                self.camera.set(cv2.CAP_PROP_CONTRAST, contrast / 100.0)
                self.contrast = contrast
            
            if saturation is not None:
                self.camera.set(cv2.CAP_PROP_SATURATION, saturation / 100.0)
                self.saturation = saturation
            
            self.get_logger().info(f'摄像头参数已更新: 亮度={self.brightness}, 对比度={self.contrast}, 饱和度={self.saturation}')
            
        except Exception as e:
            self.get_logger().error(f'摄像头参数设置失败: {e}')
    
    def cleanup(self):
        """清理资源"""
        if self.camera is not None:
            self.camera.release()
            self.get_logger().info('摄像头资源已释放')
        
        if hasattr(self, 'hardware'):
            self.hardware.cleanup()
            self.get_logger().info('摄像头硬件资源已清理')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        camera_node.get_logger().error(f'摄像头节点运行错误: {e}')
    finally:
        camera_node.cleanup()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

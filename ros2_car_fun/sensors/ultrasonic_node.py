#!/usr/bin/env python3
"""
ROS2 Car Fun - Ultrasonic Sensor Node
超声波传感器节点

Publishes ultrasonic distance measurements as Range messages
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import math
import time
from typing import Optional

from ..hardware import HardwareInterface, CarMode


class UltrasonicNode(Node):
    """
    超声波传感器节点
    
    功能：
    - 读取超声波距离数据
    - 发布Range消息
    - 提供距离滤波和异常检测
    - 支持多种数据格式输出
    """
    
    def __init__(self):
        super().__init__('ultrasonic_node')
        
        # 声明参数
        self.declare_parameter('hardware_mode', 'simulation')
        self.declare_parameter('publish_frequency', 10.0)  # Hz
        self.declare_parameter('frame_id', 'ultrasonic_frame')
        self.declare_parameter('min_range', 0.02)  # 2cm
        self.declare_parameter('max_range', 2.0)   # 200cm
        self.declare_parameter('field_of_view', 0.26)  # ~15度
        self.declare_parameter('enable_filtering', True)
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('max_change_rate', 0.5)  # m/s
        self.declare_parameter('debug_mode', False)
        
        # 获取参数
        self.hardware_mode = self.get_parameter('hardware_mode').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.field_of_view = self.get_parameter('field_of_view').value
        self.enable_filtering = self.get_parameter('enable_filtering').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        self.max_change_rate = self.get_parameter('max_change_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 初始化硬件接口
        mode = CarMode.HARDWARE if self.hardware_mode == 'hardware' else CarMode.SIMULATION
        self.hardware = HardwareInterface(mode, debug=self.debug_mode)
        
        # 发布器
        self.range_pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.distance_pub = self.create_publisher(Float32, 'ultrasonic/distance', 10)
        self.point_pub = self.create_publisher(PointStamped, 'ultrasonic/point', 10)
        
        # 状态变量
        self.last_distance = None
        self.last_time = time.time()
        self.distance_history = []
        self.error_count = 0
        self.total_readings = 0
        
        # 定时器
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(timer_period, self.read_and_publish)
        
        self.get_logger().info(f'超声波传感器节点已启动 (模式: {self.hardware_mode}, 频率: {self.publish_frequency}Hz)')
    
    def read_and_publish(self):
        """读取传感器数据并发布"""
        try:
            # 读取原始距离数据
            raw_distance = self.hardware.read_ultrasonic_distance()
            
            if raw_distance is None:
                self.error_count += 1
                if self.debug_mode:
                    self.get_logger().warn('超声波传感器读取失败')
                return
            
            self.total_readings += 1
            
            # 转换单位 (cm to m)
            distance_m = raw_distance / 100.0
            
            # 数据验证
            if not self._is_valid_distance(distance_m):
                self.error_count += 1
                if self.debug_mode:
                    self.get_logger().warn(f'无效距离数据: {distance_m:.3f}m')
                return
            
            # 数据滤波
            filtered_distance = self._filter_distance(distance_m)
            
            # 发布数据
            self._publish_range_message(filtered_distance)
            self._publish_distance_message(filtered_distance)
            self._publish_point_message(filtered_distance)
            
            # 更新状态
            self.last_distance = filtered_distance
            self.last_time = time.time()
            
            if self.debug_mode:
                self.get_logger().debug(f'距离: {filtered_distance:.3f}m (原始: {distance_m:.3f}m)')
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'超声波传感器处理错误: {e}')
    
    def _is_valid_distance(self, distance: float) -> bool:
        """验证距离数据有效性"""
        # 范围检查
        if distance < self.min_range or distance > self.max_range:
            return False
        
        # 变化率检查
        if self.last_distance is not None:
            current_time = time.time()
            dt = current_time - self.last_time
            if dt > 0:
                change_rate = abs(distance - self.last_distance) / dt
                if change_rate > self.max_change_rate:
                    return False
        
        return True
    
    def _filter_distance(self, distance: float) -> float:
        """距离数据滤波"""
        if not self.enable_filtering:
            return distance
        
        # 添加到历史记录
        self.distance_history.append(distance)
        
        # 保持窗口大小
        if len(self.distance_history) > self.filter_window_size:
            self.distance_history.pop(0)
        
        # 中值滤波
        if len(self.distance_history) >= 3:
            sorted_history = sorted(self.distance_history)
            median_index = len(sorted_history) // 2
            return sorted_history[median_index]
        else:
            # 数据不足时使用平均值
            return sum(self.distance_history) / len(self.distance_history)
    
    def _publish_range_message(self, distance: float):
        """发布Range消息"""
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = self.frame_id
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = self.field_of_view
        range_msg.min_range = self.min_range
        range_msg.max_range = self.max_range
        range_msg.range = distance
        
        self.range_pub.publish(range_msg)
    
    def _publish_distance_message(self, distance: float):
        """发布简单距离消息"""
        distance_msg = Float32()
        distance_msg.data = distance
        
        self.distance_pub.publish(distance_msg)
    
    def _publish_point_message(self, distance: float):
        """发布点云消息（假设传感器朝前）"""
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.frame_id
        
        # 假设传感器朝前（X轴正方向）
        point_msg.point.x = distance
        point_msg.point.y = 0.0
        point_msg.point.z = 0.0
        
        self.point_pub.publish(point_msg)
    
    def get_statistics(self) -> dict:
        """获取传感器统计信息"""
        success_rate = 0.0
        if self.total_readings > 0:
            success_rate = (self.total_readings - self.error_count) / self.total_readings
        
        return {
            'total_readings': self.total_readings,
            'error_count': self.error_count,
            'success_rate': success_rate,
            'last_distance': self.last_distance,
            'filter_enabled': self.enable_filtering,
            'history_size': len(self.distance_history)
        }
    
    def reset_statistics(self):
        """重置统计信息"""
        self.error_count = 0
        self.total_readings = 0
        self.distance_history.clear()
        self.get_logger().info('传感器统计信息已重置')
    
    def set_simulation_distance(self, distance: float):
        """设置仿真距离（仅仿真模式）"""
        if self.hardware.is_simulation_mode():
            self.hardware.set_simulation_ultrasonic_distance(distance * 100)  # m to cm
            if self.debug_mode:
                self.get_logger().info(f'设置仿真距离: {distance:.3f}m')
    
    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'hardware'):
            self.hardware.cleanup()
            self.get_logger().info('超声波传感器资源已清理')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    ultrasonic_node = UltrasonicNode()
    
    try:
        rclpy.spin(ultrasonic_node)
    except KeyboardInterrupt:
        ultrasonic_node.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        ultrasonic_node.get_logger().error(f'超声波节点运行错误: {e}')
    finally:
        ultrasonic_node.cleanup()
        ultrasonic_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

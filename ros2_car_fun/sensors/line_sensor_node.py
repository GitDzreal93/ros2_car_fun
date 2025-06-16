#!/usr/bin/env python3
"""
ROS2 Car Fun - Line Sensor Node
巡线传感器节点

Publishes line sensor data and line position information
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, Float32
from geometry_msgs.msg import Vector3
import time
from typing import Optional, Tuple, List

from ..hardware import HardwareInterface, CarMode


class LineSensorNode(Node):
    """
    巡线传感器节点
    
    功能：
    - 读取4路巡线传感器数据
    - 计算线的位置和方向
    - 发布传感器状态和位置信息
    - 提供线检测和跟踪算法
    """
    
    def __init__(self):
        super().__init__('line_sensor_node')
        
        # 声明参数
        self.declare_parameter('hardware_mode', 'simulation')
        self.declare_parameter('publish_frequency', 20.0)  # Hz
        self.declare_parameter('sensor_positions', [-1.5, -0.5, 0.5, 1.5])  # 传感器位置权重
        self.declare_parameter('line_threshold', 0.5)  # 线检测阈值
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('smoothing_factor', 0.8)
        self.declare_parameter('debug_mode', False)
        
        # 获取参数
        self.hardware_mode = self.get_parameter('hardware_mode').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.sensor_positions = self.get_parameter('sensor_positions').value
        self.line_threshold = self.get_parameter('line_threshold').value
        self.enable_smoothing = self.get_parameter('enable_smoothing').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 初始化硬件接口
        mode = CarMode.HARDWARE if self.hardware_mode == 'hardware' else CarMode.SIMULATION
        self.hardware = HardwareInterface(mode, debug=self.debug_mode)
        
        # 发布器
        self.sensors_pub = self.create_publisher(Int32MultiArray, 'line_sensors/raw', 10)
        self.line_detected_pub = self.create_publisher(Bool, 'line_sensors/line_detected', 10)
        self.line_position_pub = self.create_publisher(Float32, 'line_sensors/position', 10)
        self.line_vector_pub = self.create_publisher(Vector3, 'line_sensors/vector', 10)
        
        # 状态变量
        self.last_position = 0.0
        self.last_sensors = [False, False, False, False]
        self.position_history = []
        self.error_count = 0
        self.total_readings = 0
        self.line_lost_count = 0
        
        # 定时器
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(timer_period, self.read_and_publish)
        
        self.get_logger().info(f'巡线传感器节点已启动 (模式: {self.hardware_mode}, 频率: {self.publish_frequency}Hz)')
    
    def read_and_publish(self):
        """读取传感器数据并发布"""
        try:
            # 读取传感器数据
            sensor_data = self.hardware.read_line_sensors()
            
            if sensor_data is None:
                self.error_count += 1
                if self.debug_mode:
                    self.get_logger().warn('巡线传感器读取失败')
                return
            
            self.total_readings += 1
            self.last_sensors = list(sensor_data)
            
            # 发布原始传感器数据
            self._publish_raw_sensors(sensor_data)
            
            # 检测是否有线
            line_detected = any(sensor_data)
            self._publish_line_detected(line_detected)
            
            if line_detected:
                # 计算线的位置
                position = self._calculate_line_position(sensor_data)
                if position is not None:
                    # 平滑处理
                    smoothed_position = self._smooth_position(position)
                    
                    # 发布位置信息
                    self._publish_line_position(smoothed_position)
                    self._publish_line_vector(smoothed_position)
                    
                    self.last_position = smoothed_position
                else:
                    self.line_lost_count += 1
            else:
                self.line_lost_count += 1
                # 发布最后已知位置
                self._publish_line_position(self.last_position)
                self._publish_line_vector(self.last_position)
            
            if self.debug_mode:
                self.get_logger().debug(
                    f'传感器: {sensor_data}, 线检测: {line_detected}, '
                    f'位置: {self.last_position:.3f}'
                )
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'巡线传感器处理错误: {e}')
    
    def _calculate_line_position(self, sensors: Tuple[bool, bool, bool, bool]) -> Optional[float]:
        """
        计算线的位置
        
        Args:
            sensors: (左外, 左内, 右内, 右外) 传感器状态
            
        Returns:
            线的位置 (-1.0 到 1.0, 0为中心, 负值偏左, 正值偏右)
        """
        if not any(sensors):
            return None
        
        # 加权位置计算
        weighted_sum = 0.0
        weight_total = 0.0
        
        for i, (detected, position) in enumerate(zip(sensors, self.sensor_positions)):
            if detected:
                weighted_sum += position
                weight_total += 1.0
        
        if weight_total > 0:
            return weighted_sum / weight_total
        else:
            return None
    
    def _smooth_position(self, position: float) -> float:
        """位置平滑处理"""
        if not self.enable_smoothing:
            return position
        
        # 指数移动平均
        if hasattr(self, 'last_position') and self.last_position is not None:
            smoothed = (self.smoothing_factor * self.last_position + 
                       (1 - self.smoothing_factor) * position)
        else:
            smoothed = position
        
        # 添加到历史记录
        self.position_history.append(smoothed)
        if len(self.position_history) > 10:  # 保持最近10个值
            self.position_history.pop(0)
        
        return smoothed
    
    def _publish_raw_sensors(self, sensors: Tuple[bool, bool, bool, bool]):
        """发布原始传感器数据"""
        sensor_msg = Int32MultiArray()
        sensor_msg.data = [int(s) for s in sensors]
        
        self.sensors_pub.publish(sensor_msg)
    
    def _publish_line_detected(self, detected: bool):
        """发布线检测状态"""
        detected_msg = Bool()
        detected_msg.data = detected
        
        self.line_detected_pub.publish(detected_msg)
    
    def _publish_line_position(self, position: float):
        """发布线位置"""
        position_msg = Float32()
        position_msg.data = position
        
        self.line_position_pub.publish(position_msg)
    
    def _publish_line_vector(self, position: float):
        """发布线向量（用于可视化）"""
        vector_msg = Vector3()
        vector_msg.x = 1.0  # 前进方向
        vector_msg.y = -position  # 侧向偏移（负号因为坐标系）
        vector_msg.z = 0.0
        
        self.line_vector_pub.publish(vector_msg)
    
    def get_line_following_error(self) -> float:
        """获取巡线误差（用于控制算法）"""
        return self.last_position
    
    def is_line_detected(self) -> bool:
        """检查是否检测到线"""
        return any(self.last_sensors)
    
    def get_sensor_pattern(self) -> str:
        """获取传感器模式字符串（用于调试）"""
        pattern = ""
        for sensor in self.last_sensors:
            pattern += "1" if sensor else "0"
        return pattern
    
    def get_statistics(self) -> dict:
        """获取传感器统计信息"""
        success_rate = 0.0
        if self.total_readings > 0:
            success_rate = (self.total_readings - self.error_count) / self.total_readings
        
        line_detection_rate = 0.0
        if self.total_readings > 0:
            line_detection_rate = (self.total_readings - self.line_lost_count) / self.total_readings
        
        return {
            'total_readings': self.total_readings,
            'error_count': self.error_count,
            'line_lost_count': self.line_lost_count,
            'success_rate': success_rate,
            'line_detection_rate': line_detection_rate,
            'last_position': self.last_position,
            'sensor_pattern': self.get_sensor_pattern(),
            'smoothing_enabled': self.enable_smoothing
        }
    
    def reset_statistics(self):
        """重置统计信息"""
        self.error_count = 0
        self.total_readings = 0
        self.line_lost_count = 0
        self.position_history.clear()
        self.get_logger().info('巡线传感器统计信息已重置')
    
    def set_simulation_sensors(self, left_outer: bool, left_inner: bool, 
                              right_inner: bool, right_outer: bool):
        """设置仿真传感器状态（仅仿真模式）"""
        if self.hardware.is_simulation_mode():
            self.hardware.set_simulation_line_sensors(
                left_outer, left_inner, right_inner, right_outer
            )
            if self.debug_mode:
                pattern = f"{int(left_outer)}{int(left_inner)}{int(right_inner)}{int(right_outer)}"
                self.get_logger().info(f'设置仿真传感器: {pattern}')
    
    def simulate_line_following_scenario(self):
        """模拟巡线场景（仅用于测试）"""
        if not self.hardware.is_simulation_mode():
            return
        
        scenarios = [
            (False, True, True, False),   # 中心
            (False, True, False, False),  # 左偏
            (False, False, True, False),  # 右偏
            (True, False, False, False),  # 大幅左偏
            (False, False, False, True),  # 大幅右偏
            (False, False, False, False), # 丢线
            (True, True, True, True),     # 交叉线
        ]
        
        import threading
        import time
        
        def run_scenario():
            for i, sensors in enumerate(scenarios):
                self.set_simulation_sensors(*sensors)
                self.get_logger().info(f'场景 {i+1}: {sensors}')
                time.sleep(2.0)
        
        thread = threading.Thread(target=run_scenario)
        thread.daemon = True
        thread.start()
    
    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'hardware'):
            self.hardware.cleanup()
            self.get_logger().info('巡线传感器资源已清理')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    line_sensor_node = LineSensorNode()
    
    try:
        rclpy.spin(line_sensor_node)
    except KeyboardInterrupt:
        line_sensor_node.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        line_sensor_node.get_logger().error(f'巡线传感器节点运行错误: {e}')
    finally:
        line_sensor_node.cleanup()
        line_sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
ROS2 Car Fun - IR Receiver Node
红外遥控接收节点

Receives and processes IR remote control commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
import time
from typing import Optional, Dict, List

from ..hardware import HardwareInterface, CarMode


class IRReceiverNode(Node):
    """
    红外遥控接收节点
    
    功能：
    - 接收红外遥控信号
    - 解析遥控按键
    - 转换为控制命令
    - 发布Twist消息用于车辆控制
    """
    
    def __init__(self):
        super().__init__('ir_receiver_node')
        
        # 声明参数
        self.declare_parameter('hardware_mode', 'simulation')
        self.declare_parameter('publish_frequency', 10.0)  # Hz
        self.declare_parameter('enable_velocity_control', True)
        self.declare_parameter('linear_speed', 1.0)  # m/s
        self.declare_parameter('angular_speed', 1.0)  # rad/s
        self.declare_parameter('command_timeout', 0.5)  # 命令超时时间
        self.declare_parameter('debug_mode', False)
        
        # 获取参数
        self.hardware_mode = self.get_parameter('hardware_mode').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.enable_velocity_control = self.get_parameter('enable_velocity_control').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 初始化硬件接口
        mode = CarMode.HARDWARE if self.hardware_mode == 'hardware' else CarMode.SIMULATION
        self.hardware = HardwareInterface(mode, debug=self.debug_mode)
        
        # 发布器
        self.raw_code_pub = self.create_publisher(Int32, 'ir_receiver/raw_code', 10)
        self.command_pub = self.create_publisher(String, 'ir_receiver/command', 10)
        
        if self.enable_velocity_control:
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 红外遥控按键映射 (根据实际遥控器调整)
        self.key_mapping = {
            0x45: 'power',
            0x46: 'vol_up',
            0x47: 'func_stop',
            0x44: 'prev',
            0x40: 'pause',
            0x43: 'next',
            0x07: 'vol_down',
            0x15: 'up',
            0x09: 'down',
            0x16: 'zero',
            0x19: 'eq',
            0x0D: 'st_rept',
            0x0C: 'one',
            0x18: 'two',
            0x5E: 'three',
            0x08: 'four',
            0x1C: 'five',
            0x5A: 'six',
            0x42: 'seven',
            0x52: 'eight',
            0x4A: 'nine',
        }
        
        # 按键到运动命令的映射
        self.motion_mapping = {
            'up': (self.linear_speed, 0.0, 0.0),      # 前进
            'down': (-self.linear_speed, 0.0, 0.0),   # 后退
            'four': (0.0, self.linear_speed, 0.0),    # 左移
            'six': (0.0, -self.linear_speed, 0.0),    # 右移
            'one': (0.0, 0.0, self.angular_speed),    # 左转
            'three': (0.0, 0.0, -self.angular_speed), # 右转
            'five': (0.0, 0.0, 0.0),                  # 停止
            'pause': (0.0, 0.0, 0.0),                 # 停止
        }
        
        # 状态变量
        self.last_command = None
        self.last_command_time = 0.0
        self.current_velocity = Twist()
        self.error_count = 0
        self.total_commands = 0
        
        # 定时器
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(timer_period, self.read_and_publish)
        
        # 命令超时检查定时器
        self.timeout_timer = self.create_timer(0.1, self.check_command_timeout)
        
        self.get_logger().info(f'红外遥控接收节点已启动 (模式: {self.hardware_mode})')
    
    def read_and_publish(self):
        """读取红外信号并发布"""
        try:
            # 这里需要实现实际的红外信号读取
            # 由于硬件抽象层中还没有实现IR读取，我们先模拟
            raw_code = self._read_ir_code()
            
            if raw_code is not None:
                self.total_commands += 1
                
                # 发布原始代码
                self._publish_raw_code(raw_code)
                
                # 解析命令
                command = self._decode_command(raw_code)
                if command:
                    self._publish_command(command)
                    
                    # 处理运动控制
                    if self.enable_velocity_control:
                        self._process_motion_command(command)
                    
                    self.last_command = command
                    self.last_command_time = time.time()
                    
                    if self.debug_mode:
                        self.get_logger().debug(f'IR命令: {command} (代码: 0x{raw_code:02X})')
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'红外接收处理错误: {e}')
    
    def _read_ir_code(self) -> Optional[int]:
        """读取红外代码（需要在硬件抽象层中实现）"""
        # TODO: 实现实际的红外读取
        # 这里返回模拟数据用于测试
        if self.hardware.is_simulation_mode():
            # 模拟偶尔接收到命令
            import random
            if random.random() < 0.1:  # 10%概率
                return random.choice(list(self.key_mapping.keys()))
        
        return None
    
    def _decode_command(self, raw_code: int) -> Optional[str]:
        """解码红外命令"""
        return self.key_mapping.get(raw_code)
    
    def _process_motion_command(self, command: str):
        """处理运动控制命令"""
        if command in self.motion_mapping:
            vx, vy, omega = self.motion_mapping[command]
            
            # 创建Twist消息
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = omega
            
            # 发布速度命令
            self.cmd_vel_pub.publish(twist)
            self.current_velocity = twist
            
            if self.debug_mode:
                self.get_logger().debug(f'运动命令: vx={vx}, vy={vy}, omega={omega}')
    
    def _publish_raw_code(self, code: int):
        """发布原始红外代码"""
        code_msg = Int32()
        code_msg.data = code
        
        self.raw_code_pub.publish(code_msg)
    
    def _publish_command(self, command: str):
        """发布解析后的命令"""
        command_msg = String()
        command_msg.data = command
        
        self.command_pub.publish(command_msg)
    
    def check_command_timeout(self):
        """检查命令超时"""
        if (self.last_command_time > 0 and 
            time.time() - self.last_command_time > self.command_timeout):
            
            # 超时后停止运动
            if self.enable_velocity_control:
                stop_twist = Twist()  # 所有值默认为0
                self.cmd_vel_pub.publish(stop_twist)
                self.current_velocity = stop_twist
            
            self.last_command_time = 0.0  # 重置时间避免重复发送
    
    def add_custom_key_mapping(self, code: int, command: str):
        """添加自定义按键映射"""
        self.key_mapping[code] = command
        self.get_logger().info(f'添加按键映射: 0x{code:02X} -> {command}')
    
    def add_custom_motion_mapping(self, command: str, vx: float, vy: float, omega: float):
        """添加自定义运动映射"""
        self.motion_mapping[command] = (vx, vy, omega)
        self.get_logger().info(f'添加运动映射: {command} -> ({vx}, {vy}, {omega})')
    
    def get_statistics(self) -> dict:
        """获取接收统计信息"""
        success_rate = 0.0
        if self.total_commands > 0:
            success_rate = (self.total_commands - self.error_count) / self.total_commands
        
        return {
            'total_commands': self.total_commands,
            'error_count': self.error_count,
            'success_rate': success_rate,
            'last_command': self.last_command,
            'velocity_control_enabled': self.enable_velocity_control,
            'current_velocity': {
                'linear_x': self.current_velocity.linear.x,
                'linear_y': self.current_velocity.linear.y,
                'angular_z': self.current_velocity.angular.z
            }
        }
    
    def reset_statistics(self):
        """重置统计信息"""
        self.error_count = 0
        self.total_commands = 0
        self.get_logger().info('红外接收统计信息已重置')
    
    def simulate_ir_command(self, command: str):
        """模拟红外命令（仅用于测试）"""
        if self.hardware.is_simulation_mode():
            # 查找命令对应的代码
            code = None
            for c, cmd in self.key_mapping.items():
                if cmd == command:
                    code = c
                    break
            
            if code is not None:
                self._publish_raw_code(code)
                self._publish_command(command)
                
                if self.enable_velocity_control:
                    self._process_motion_command(command)
                
                self.last_command = command
                self.last_command_time = time.time()
                
                self.get_logger().info(f'模拟IR命令: {command}')
            else:
                self.get_logger().warn(f'未知命令: {command}')
    
    def list_available_commands(self) -> List[str]:
        """列出可用的命令"""
        return list(self.key_mapping.values())
    
    def cleanup(self):
        """清理资源"""
        # 停止运动
        if self.enable_velocity_control:
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
        
        if hasattr(self, 'hardware'):
            self.hardware.cleanup()
            self.get_logger().info('红外接收器资源已清理')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    ir_receiver_node = IRReceiverNode()
    
    try:
        rclpy.spin(ir_receiver_node)
    except KeyboardInterrupt:
        ir_receiver_node.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        ir_receiver_node.get_logger().error(f'红外接收节点运行错误: {e}')
    finally:
        ir_receiver_node.cleanup()
        ir_receiver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

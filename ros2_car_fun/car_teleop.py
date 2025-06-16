#!/usr/bin/env python3
"""
ROS2 Car Teleoperation Node
车辆键盘遥控节点 - 通过键盘控制车辆运动
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class CarTeleop(Node):
    """车辆遥控类"""
    
    def __init__(self):
        super().__init__('car_teleop')
        
        # 声明参数
        self.declare_parameter('linear_speed_step', 0.1)  # 线速度步长
        self.declare_parameter('angular_speed_step', 0.1)  # 角速度步长
        self.declare_parameter('max_linear_speed', 2.0)   # 最大线速度
        self.declare_parameter('max_angular_speed', 1.0)  # 最大角速度
        
        # 获取参数
        self.linear_step = self.get_parameter('linear_speed_step').value
        self.angular_step = self.get_parameter('angular_speed_step').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 当前速度
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 键盘映射
        self.key_bindings = {
            'w': (self.linear_step, 0.0),      # 前进
            's': (-self.linear_step, 0.0),     # 后退
            'a': (0.0, self.angular_step),     # 左转
            'd': (0.0, -self.angular_step),    # 右转
            'q': (self.linear_step, self.angular_step),   # 前进+左转
            'e': (self.linear_step, -self.angular_step),  # 前进+右转
            'z': (-self.linear_step, self.angular_step),  # 后退+左转
            'c': (-self.linear_step, -self.angular_step), # 后退+右转
            ' ': (0.0, 0.0),                   # 停止 (空格键)
        }
        
        self.print_instructions()
        
        # 定时器 - 检查键盘输入
        self.timer = self.create_timer(0.1, self.check_keyboard)
        
        self.get_logger().info('车辆遥控节点已启动')
    
    def print_instructions(self):
        """打印操作说明"""
        instructions = """
╔══════════════════════════════════════════════════════════════╗
║                        车辆遥控操作说明                        ║
╠══════════════════════════════════════════════════════════════╣
║  移动控制:                                                   ║
║    w - 前进          s - 后退                                ║
║    a - 左转          d - 右转                                ║
║                                                              ║
║  组合控制:                                                   ║
║    q - 前进+左转     e - 前进+右转                           ║
║    z - 后退+左转     c - 后退+右转                           ║
║                                                              ║
║  其他:                                                       ║
║    空格 - 停止       Ctrl+C - 退出                           ║
║                                                              ║
║  当前速度限制:                                               ║
║    最大线速度: {:.1f} m/s                                    ║
║    最大角速度: {:.1f} rad/s                                  ║
╚══════════════════════════════════════════════════════════════╝
""".format(self.max_linear, self.max_angular)
        
        print(instructions)
    
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def check_keyboard(self):
        """检查键盘输入"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = self.get_key()
            self.process_key(key)
    
    def process_key(self, key):
        """处理键盘输入"""
        if key == '\x03':  # Ctrl+C
            self.get_logger().info('接收到退出信号')
            rclpy.shutdown()
            return
        
        if key in self.key_bindings:
            linear_delta, angular_delta = self.key_bindings[key]
            
            if key == ' ':  # 空格键 - 停止
                self.linear_vel = 0.0
                self.angular_vel = 0.0
            else:
                # 更新速度
                self.linear_vel += linear_delta
                self.angular_vel += angular_delta
                
                # 限制速度范围
                self.linear_vel = max(-self.max_linear, 
                                     min(self.max_linear, self.linear_vel))
                self.angular_vel = max(-self.max_angular, 
                                      min(self.max_angular, self.angular_vel))
            
            # 发布速度命令
            self.publish_velocity()
            
            # 显示当前状态
            self.print_status(key)
        
        elif key == 'h':  # 帮助
            self.print_instructions()
        
        elif key == 'r':  # 重置速度
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.publish_velocity()
            self.get_logger().info('速度已重置')
    
    def publish_velocity(self):
        """发布速度命令"""
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel
        twist_msg.angular.z = self.angular_vel
        
        self.cmd_vel_pub.publish(twist_msg)
    
    def print_status(self, key):
        """打印当前状态"""
        status = f"按键: {key} | 线速度: {self.linear_vel:+.2f} m/s | 角速度: {self.angular_vel:+.2f} rad/s"
        print(f"\r{status}", end='', flush=True)
    
    def cleanup(self):
        """清理资源"""
        # 发送停止命令
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)
        
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
        print("\n车辆遥控节点已关闭")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    car_teleop = CarTeleop()
    
    try:
        rclpy.spin(car_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        car_teleop.cleanup()
        car_teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
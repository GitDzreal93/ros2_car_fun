#!/usr/bin/env python3
"""
ROS2 Car Controller Node
车辆控制器节点 - 接收速度命令并控制车辆运动

Enhanced version with hardware abstraction layer support
支持硬件抽象层的增强版本
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool, Int32MultiArray
import math
import time

# 导入硬件抽象层
from .hardware import HardwareInterface, CarMode, LEDColor


class CarController(Node):
    """
    增强版车辆控制器类

    集成硬件抽象层，支持麦克纳姆轮全向运动
    """

    def __init__(self):
        super().__init__('car_controller')

        # 声明参数
        self.declare_parameter('max_linear_speed', 2.0)  # 最大线速度 m/s
        self.declare_parameter('max_angular_speed', 1.0)  # 最大角速度 rad/s
        self.declare_parameter('wheel_base', 0.15)  # 轴距 m (更新为实际值)
        self.declare_parameter('wheel_radius', 0.03)  # 轮子半径 m (更新为实际值)
        self.declare_parameter('hardware_mode', 'simulation')  # 硬件模式
        self.declare_parameter('debug_mode', False)  # 调试模式
        self.declare_parameter('speed_scale', 100.0)  # 速度缩放因子 (m/s to motor units)

        # 获取参数
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.hardware_mode = self.get_parameter('hardware_mode').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.speed_scale = self.get_parameter('speed_scale').value

        # 初始化硬件接口
        mode = CarMode.HARDWARE if self.hardware_mode == 'hardware' else CarMode.SIMULATION
        self.hardware = HardwareInterface(mode, debug=self.debug_mode)

        # 订阅速度命令
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 发布里程计信息
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 发布关节状态 (4个麦克纳姆轮)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 发布传感器数据
        self.ultrasonic_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.line_sensor_pub = self.create_publisher(Int32MultiArray, 'line_sensors', 10)

        # 车辆状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel_x = 0.0  # X方向线速度
        self.linear_vel_y = 0.0  # Y方向线速度 (麦克纳姆轮支持)
        self.angular_vel = 0.0

        # 轮子角度 (4个麦克纳姆轮)
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]  # [L1, L2, R1, R2]

        # 上次更新时间
        self.last_time = time.time()

        # 定时器 - 50Hz更新频率
        self.timer = self.create_timer(0.02, self.update_state)

        # 传感器读取定时器 - 10Hz
        self.sensor_timer = self.create_timer(0.1, self.read_sensors)

        self.get_logger().info(f'车辆控制器节点已启动 (模式: {self.hardware_mode})')

    def cmd_vel_callback(self, msg):
        """
        处理速度命令 (支持麦克纳姆轮全向运动)

        Twist消息映射:
        - linear.x: 前后运动 (前进为正)
        - linear.y: 左右运动 (左移为正)
        - angular.z: 旋转运动 (逆时针为正)
        """
        # 限制速度范围
        self.linear_vel_x = max(-self.max_linear_speed,
                               min(self.max_linear_speed, msg.linear.x))
        self.linear_vel_y = max(-self.max_linear_speed,
                               min(self.max_linear_speed, msg.linear.y))
        self.angular_vel = max(-self.max_angular_speed,
                              min(self.max_angular_speed, msg.angular.z))

        # 转换为电机控制单位并发送到硬件
        vx_motor = self.linear_vel_x * self.speed_scale
        vy_motor = self.linear_vel_y * self.speed_scale
        omega_motor = self.angular_vel * self.speed_scale

        # 控制硬件运动
        self.hardware.move(vx_motor, vy_motor, omega_motor)

        if self.debug_mode:
            self.get_logger().debug(
                f'速度命令: vx={self.linear_vel_x:.2f} vy={self.linear_vel_y:.2f} '
                f'omega={self.angular_vel:.2f} m/s'
            )

    def update_state(self):
        """更新车辆状态 (麦克纳姆轮运动学)"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 麦克纳姆轮运动学 - 更新位置和姿态
        # 在机器人坐标系中的运动
        vx_robot = self.linear_vel_x  # 前后
        vy_robot = self.linear_vel_y  # 左右
        omega = self.angular_vel       # 旋转

        # 转换到全局坐标系
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        vx_global = vx_robot * cos_theta - vy_robot * sin_theta
        vy_global = vx_robot * sin_theta + vy_robot * cos_theta

        # 更新位置和姿态
        self.x += vx_global * dt
        self.y += vy_global * dt
        self.theta += omega * dt

        # 归一化角度到 [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 计算4个麦克纳姆轮的转速 (用于里程计)
        wheel_speeds = self._calculate_wheel_speeds_for_odometry(
            vx_robot, vy_robot, omega
        )

        # 更新轮子角度
        for i in range(4):
            self.wheel_angles[i] += wheel_speeds[i] * dt

        # 发布里程计信息
        self.publish_odometry()

        # 发布关节状态
        self.publish_joint_states(wheel_speeds)

    def _calculate_wheel_speeds_for_odometry(self, vx: float, vy: float, omega: float):
        """计算4个轮子的转速 (用于里程计)"""
        # 麦克纳姆轮运动学逆解
        # 这里使用简化的模型，实际应该根据具体的轮子配置调整
        rotation_factor = (self.wheel_base + 0.13) / 2  # 假设轮距为0.13m

        l1 = (vy + vx - omega * rotation_factor) / self.wheel_radius
        l2 = (vy - vx - omega * rotation_factor) / self.wheel_radius
        r1 = (vy - vx + omega * rotation_factor) / self.wheel_radius
        r2 = (vy + vx + omega * rotation_factor) / self.wheel_radius

        return [l1, l2, r1, r2]

    def read_sensors(self):
        """读取传感器数据并发布"""
        # 读取超声波传感器
        distance = self.hardware.read_ultrasonic_distance()
        if distance is not None:
            self.publish_ultrasonic_data(distance)

        # 读取巡线传感器
        line_sensors = self.hardware.read_line_sensors()
        if line_sensors is not None:
            self.publish_line_sensor_data(line_sensors)

    def publish_odometry(self):
        """发布里程计信息 (支持麦克纳姆轮)"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # 位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # 姿态 (四元数)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        # 速度 (支持Y方向运动)
        odom_msg.twist.twist.linear.x = self.linear_vel_x
        odom_msg.twist.twist.linear.y = self.linear_vel_y
        odom_msg.twist.twist.angular.z = self.angular_vel

        self.odom_pub.publish(odom_msg)

    def publish_joint_states(self, wheel_speeds):
        """发布关节状态 (4个麦克纳姆轮)"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'left_front_wheel_joint',   # L1
            'left_rear_wheel_joint',    # L2
            'right_front_wheel_joint',  # R1
            'right_rear_wheel_joint'    # R2
        ]
        joint_msg.position = self.wheel_angles
        joint_msg.velocity = wheel_speeds

        self.joint_state_pub.publish(joint_msg)

    def publish_ultrasonic_data(self, distance):
        """发布超声波传感器数据"""
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'ultrasonic_frame'
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.26  # 约15度
        range_msg.min_range = 0.02  # 2cm
        range_msg.max_range = 2.0   # 200cm
        range_msg.range = distance / 100.0  # 转换为米

        self.ultrasonic_pub.publish(range_msg)

    def publish_line_sensor_data(self, sensors):
        """发布巡线传感器数据"""
        sensor_msg = Int32MultiArray()
        sensor_msg.data = [int(s) for s in sensors]  # 转换为整数列表

        self.line_sensor_pub.publish(sensor_msg)

    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'hardware'):
            self.hardware.cleanup()
            self.get_logger().info('硬件资源已清理')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    car_controller = CarController()

    try:
        rclpy.spin(car_controller)
    except KeyboardInterrupt:
        car_controller.get_logger().info('接收到中断信号，正在关闭...')
    except Exception as e:
        car_controller.get_logger().error(f'控制器运行错误: {e}')
    finally:
        car_controller.cleanup()
        car_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
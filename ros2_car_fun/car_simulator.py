#!/usr/bin/env python3
"""
ROS2 Car Simulator Node
车辆模拟器节点 - 模拟车辆物理行为和传感器数据
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid
import math
import random
import numpy as np


class CarSimulator(Node):
    """车辆模拟器类"""
    
    def __init__(self):
        super().__init__('car_simulator')
        
        # 声明参数
        self.declare_parameter('simulation_frequency', 50.0)  # 模拟频率 Hz
        self.declare_parameter('laser_range_max', 10.0)  # 激光雷达最大距离 m
        self.declare_parameter('laser_angle_min', -3.14159)  # 激光雷达最小角度
        self.declare_parameter('laser_angle_max', 3.14159)   # 激光雷达最大角度
        self.declare_parameter('laser_angle_increment', 0.017453)  # 角度增量
        self.declare_parameter('noise_level', 0.1)  # 噪声水平
        
        # 获取参数
        self.sim_freq = self.get_parameter('simulation_frequency').value
        self.laser_range_max = self.get_parameter('laser_range_max').value
        self.laser_angle_min = self.get_parameter('laser_angle_min').value
        self.laser_angle_max = self.get_parameter('laser_angle_max').value
        self.laser_angle_increment = self.get_parameter('laser_angle_increment').value
        self.noise_level = self.get_parameter('noise_level').value
        
        # 订阅速度命令
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 发布传感器数据
        self.laser_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # 车辆状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # 模拟环境 - 简单的矩形房间
        self.room_width = 10.0
        self.room_height = 8.0
        self.obstacles = [
            {'x': 3.0, 'y': 2.0, 'radius': 0.5},  # 圆形障碍物
            {'x': 6.0, 'y': 5.0, 'radius': 0.8},
            {'x': 1.5, 'y': 6.0, 'radius': 0.3},
        ]
        
        # 定时器
        self.timer = self.create_timer(1.0 / self.sim_freq, self.simulation_step)
        
        # 发布静态地图
        self.publish_map()
        
        self.get_logger().info('车辆模拟器节点已启动')
    
    def cmd_vel_callback(self, msg):
        """处理速度命令"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def simulation_step(self):
        """模拟步骤"""
        dt = 1.0 / self.sim_freq
        
        # 更新车辆位置
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt
        
        # 边界检查
        self.x = max(0.5, min(self.room_width - 0.5, self.x))
        self.y = max(0.5, min(self.room_height - 0.5, self.y))
        
        # 发布传感器数据
        self.publish_laser_scan()
        self.publish_imu()
        self.publish_pose()
    
    def publish_laser_scan(self):
        """发布激光雷达数据"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        
        scan_msg.angle_min = self.laser_angle_min
        scan_msg.angle_max = self.laser_angle_max
        scan_msg.angle_increment = self.laser_angle_increment
        scan_msg.range_min = 0.1
        scan_msg.range_max = self.laser_range_max
        
        # 计算激光束数量
        num_beams = int((self.laser_angle_max - self.laser_angle_min) / self.laser_angle_increment)
        ranges = []
        
        for i in range(num_beams):
            angle = self.laser_angle_min + i * self.laser_angle_increment
            beam_angle = self.theta + angle
            
            # 计算到墙壁的距离
            range_to_wall = self.calculate_wall_distance(beam_angle)
            
            # 计算到障碍物的距离
            range_to_obstacle = self.calculate_obstacle_distance(beam_angle)
            
            # 取最小距离
            measured_range = min(range_to_wall, range_to_obstacle)
            
            # 添加噪声
            noise = random.gauss(0, self.noise_level * 0.1)
            measured_range += noise
            
            # 限制范围
            measured_range = max(scan_msg.range_min, 
                               min(scan_msg.range_max, measured_range))
            
            ranges.append(measured_range)
        
        scan_msg.ranges = ranges
        self.laser_pub.publish(scan_msg)
    
    def calculate_wall_distance(self, beam_angle):
        """计算到墙壁的距离"""
        dx = math.cos(beam_angle)
        dy = math.sin(beam_angle)
        
        # 计算到四面墙的距离
        distances = []
        
        # 右墙
        if dx > 0:
            t = (self.room_width - self.x) / dx
            if t > 0:
                distances.append(t)
        
        # 左墙
        if dx < 0:
            t = -self.x / dx
            if t > 0:
                distances.append(t)
        
        # 上墙
        if dy > 0:
            t = (self.room_height - self.y) / dy
            if t > 0:
                distances.append(t)
        
        # 下墙
        if dy < 0:
            t = -self.y / dy
            if t > 0:
                distances.append(t)
        
        return min(distances) if distances else self.laser_range_max
    
    def calculate_obstacle_distance(self, beam_angle):
        """计算到障碍物的距离"""
        dx = math.cos(beam_angle)
        dy = math.sin(beam_angle)
        
        min_distance = self.laser_range_max
        
        for obstacle in self.obstacles:
            # 计算射线与圆形障碍物的交点
            ox, oy, r = obstacle['x'], obstacle['y'], obstacle['radius']
            
            # 射线起点到圆心的向量
            to_center_x = ox - self.x
            to_center_y = oy - self.y
            
            # 投影长度
            proj_length = to_center_x * dx + to_center_y * dy
            
            if proj_length > 0:  # 障碍物在射线前方
                # 最近点到圆心的距离
                closest_x = self.x + proj_length * dx
                closest_y = self.y + proj_length * dy
                dist_to_center = math.sqrt((closest_x - ox)**2 + (closest_y - oy)**2)
                
                if dist_to_center <= r:  # 射线与圆相交
                    # 计算交点距离
                    chord_half = math.sqrt(r**2 - dist_to_center**2)
                    intersection_dist = proj_length - chord_half
                    if intersection_dist > 0:
                        min_distance = min(min_distance, intersection_dist)
        
        return min_distance
    
    def publish_imu(self):
        """发布IMU数据"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_frame'
        
        # 角速度 (添加噪声)
        imu_msg.angular_velocity.z = self.angular_vel + random.gauss(0, self.noise_level * 0.01)
        
        # 线性加速度 (简化模拟)
        imu_msg.linear_acceleration.x = random.gauss(0, self.noise_level * 0.1)
        imu_msg.linear_acceleration.y = random.gauss(0, self.noise_level * 0.1)
        imu_msg.linear_acceleration.z = 9.81 + random.gauss(0, self.noise_level * 0.1)
        
        # 姿态 (四元数)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(self.theta / 2)
        imu_msg.orientation.w = math.cos(self.theta / 2)
        
        self.imu_pub.publish(imu_msg)
    
    def publish_pose(self):
        """发布位姿信息"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(self.theta / 2)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2)
        
        self.pose_pub.publish(pose_msg)
    
    def publish_map(self):
        """发布静态地图"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        # 地图参数
        resolution = 0.1  # 10cm per pixel
        width = int(self.room_width / resolution)
        height = int(self.room_height / resolution)
        
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # 创建地图数据
        data = [0] * (width * height)  # 0 = 自由空间
        
        # 添加墙壁 (100 = 占用)
        for y in range(height):
            for x in range(width):
                if x == 0 or x == width-1 or y == 0 or y == height-1:
                    data[y * width + x] = 100
        
        # 添加障碍物
        for obstacle in self.obstacles:
            ox = int(obstacle['x'] / resolution)
            oy = int(obstacle['y'] / resolution)
            r = int(obstacle['radius'] / resolution)
            
            for dy in range(-r, r+1):
                for dx in range(-r, r+1):
                    if dx*dx + dy*dy <= r*r:
                        px, py = ox + dx, oy + dy
                        if 0 <= px < width and 0 <= py < height:
                            data[py * width + px] = 100
        
        map_msg.data = data
        self.map_pub.publish(map_msg)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    car_simulator = CarSimulator()
    
    try:
        rclpy.spin(car_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        car_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
ROS2 Car System Launch File
车辆系统启动文件 - 启动所有相关节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='car_params.yaml',
        description='配置文件路径'
    )
    
    # 车辆控制器节点 (增强版，支持硬件抽象层)
    car_controller_node = Node(
        package='ros2_car_fun',
        executable='car_controller',
        name='car_controller',
        output='screen',
        parameters=[
            {
                'max_linear_speed': 2.0,
                'max_angular_speed': 1.0,
                'wheel_base': 0.15,        # 麦克纳姆轮轴距
                'wheel_radius': 0.03,      # 实际轮子半径
                'hardware_mode': 'simulation',  # 硬件模式
                'debug_mode': False,       # 调试模式
                'speed_scale': 100.0,      # 速度缩放
                'sensor_frequency': 10.0,  # 传感器频率
            }
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom'),
            ('joint_states', 'joint_states'),
            ('ultrasonic', 'ultrasonic'),
            ('line_sensors', 'line_sensors'),
        ]
    )
    
    # 注意：car_simulator 和 car_teleop 节点已被移除
    # 使用硬件抽象层的仿真模式替代 car_simulator
    # 使用红外遥控接收节点替代 car_teleop
    # 如需激光雷达仿真，请使用专门的仿真环境如Gazebo
    
    # 基础变换发布器 - base_link到odom
    static_transform_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg='正在启动ROS2车辆系统...'
    )
    
    return LaunchDescription([
        # 启动参数
        config_file_arg,
        
        # 启动信息
        launch_info,
        
        # 节点
        car_controller_node,

        # 变换发布器
        static_transform_odom,
    ])


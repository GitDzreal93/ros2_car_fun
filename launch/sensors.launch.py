#!/usr/bin/env python3
"""
ROS2 Car Fun - Sensors Launch File
传感器启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode',
        default_value='simulation',
        choices=['hardware', 'simulation'],
        description='Hardware mode for sensors'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable debug output'
    )
    
    enable_ultrasonic_arg = DeclareLaunchArgument(
        'enable_ultrasonic',
        default_value='true',
        choices=['true', 'false'],
        description='Enable ultrasonic sensor'
    )
    
    enable_line_sensor_arg = DeclareLaunchArgument(
        'enable_line_sensor',
        default_value='true',
        choices=['true', 'false'],
        description='Enable line sensor'
    )
    
    enable_ir_receiver_arg = DeclareLaunchArgument(
        'enable_ir_receiver',
        default_value='true',
        choices=['true', 'false'],
        description='Enable IR receiver'
    )
    
    # 获取配置文件路径
    sensor_config_file = PathJoinSubstitution([
        FindPackageShare('ros2_car_fun'),
        'config',
        'sensor_config.yaml'
    ])
    
    # 获取启动配置
    hardware_mode = LaunchConfiguration('hardware_mode')
    debug_mode = LaunchConfiguration('debug_mode')
    enable_ultrasonic = LaunchConfiguration('enable_ultrasonic')
    enable_line_sensor = LaunchConfiguration('enable_line_sensor')
    enable_ir_receiver = LaunchConfiguration('enable_ir_receiver')
    
    # 超声波传感器节点
    ultrasonic_node = Node(
        package='ros2_car_fun',
        executable='ultrasonic_node',
        name='ultrasonic_node',
        output='screen',
        condition=IfCondition(enable_ultrasonic),
        parameters=[
            sensor_config_file,
            {
                'hardware_mode': hardware_mode,
                'debug_mode': debug_mode,
            }
        ],
        remappings=[
            ('ultrasonic/range', 'sensors/ultrasonic/range'),
            ('ultrasonic/distance', 'sensors/ultrasonic/distance'),
            ('ultrasonic/point', 'sensors/ultrasonic/point'),
        ]
    )
    
    # 巡线传感器节点
    line_sensor_node = Node(
        package='ros2_car_fun',
        executable='line_sensor_node',
        name='line_sensor_node',
        output='screen',
        condition=IfCondition(enable_line_sensor),
        parameters=[
            sensor_config_file,
            {
                'hardware_mode': hardware_mode,
                'debug_mode': debug_mode,
            }
        ],
        remappings=[
            ('line_sensors/raw', 'sensors/line_sensors/raw'),
            ('line_sensors/line_detected', 'sensors/line_sensors/line_detected'),
            ('line_sensors/position', 'sensors/line_sensors/position'),
            ('line_sensors/vector', 'sensors/line_sensors/vector'),
        ]
    )
    
    # 红外接收器节点
    ir_receiver_node = Node(
        package='ros2_car_fun',
        executable='ir_receiver_node',
        name='ir_receiver_node',
        output='screen',
        condition=IfCondition(enable_ir_receiver),
        parameters=[
            sensor_config_file,
            {
                'hardware_mode': hardware_mode,
                'debug_mode': debug_mode,
            }
        ],
        remappings=[
            ('ir_receiver/raw_code', 'sensors/ir_receiver/raw_code'),
            ('ir_receiver/command', 'sensors/ir_receiver/command'),
            ('cmd_vel', 'cmd_vel'),  # 直接控制车辆
        ]
    )
    
    # 传感器组
    sensors_group = GroupAction([
        ultrasonic_node,
        line_sensor_node,
        ir_receiver_node,
    ])
    
    # 静态变换发布器 - 传感器坐标系
    static_transform_ultrasonic = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_ultrasonic',
        arguments=['0.12', '0', '0.05', '0', '0', '0', 'base_link', 'ultrasonic_frame'],
        condition=IfCondition(enable_ultrasonic)
    )
    
    static_transform_line_sensors = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_line_sensors',
        arguments=['0.0', '0', '-0.02', '0', '0', '0', 'base_link', 'line_sensor_frame'],
        condition=IfCondition(enable_line_sensor)
    )
    
    return LaunchDescription([
        # 参数声明
        hardware_mode_arg,
        debug_mode_arg,
        enable_ultrasonic_arg,
        enable_line_sensor_arg,
        enable_ir_receiver_arg,
        
        # 传感器节点
        sensors_group,
        
        # 静态变换
        static_transform_ultrasonic,
        static_transform_line_sensors,
    ])

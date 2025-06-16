#!/usr/bin/env python3
"""
ROS2 Car Fun - Hardware Test Launch File
硬件测试启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='simulation',
        choices=['hardware', 'simulation'],
        description='Hardware test mode'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        choices=['true', 'false'],
        description='Enable debug output'
    )
    
    test_type_arg = DeclareLaunchArgument(
        'test',
        default_value='all',
        choices=['movement', 'servo', 'led', 'sensor', 'beeper', 'all'],
        description='Specific test to run'
    )
    
    interactive_arg = DeclareLaunchArgument(
        'interactive',
        default_value='false',
        choices=['true', 'false'],
        description='Run in interactive mode'
    )
    
    # 获取配置文件路径
    config_file = PathJoinSubstitution([
        FindPackageShare('ros2_car_fun'),
        'config',
        'hardware_config.yaml'
    ])
    
    # 硬件测试进程
    hardware_test_process = ExecuteProcess(
        cmd=[
            'python3', 'test_hardware.py',
            '--mode', LaunchConfiguration('mode'),
            '--test', LaunchConfiguration('test'),
        ],
        output='screen',
        emulate_tty=True,
        # 根据debug参数添加--debug标志
        additional_env={'PYTHONUNBUFFERED': '1'}
    )
    
    return LaunchDescription([
        mode_arg,
        debug_arg,
        test_type_arg,
        interactive_arg,
        hardware_test_process,
    ])

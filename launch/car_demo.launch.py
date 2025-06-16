#!/usr/bin/env python3
"""
ROS2 Car Demo Launch File
车辆演示启动文件 - 启动所有相关节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description='是否启动模拟器节点'
    )
    
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='是否启动遥控节点'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='car_params.yaml',
        description='配置文件路径'
    )
    
    # 获取启动配置
    use_simulator = LaunchConfiguration('use_simulator')
    use_teleop = LaunchConfiguration('use_teleop')
    config_file = LaunchConfiguration('config_file')
    
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
    
    # 车辆模拟器节点 (条件启动)
    car_simulator_node = Node(
        package='ros2_car_fun',
        executable='car_simulator',
        name='car_simulator',
        output='screen',
        condition=IfCondition(use_simulator),
        parameters=[
            {
                'simulation_frequency': 50.0,
                'laser_range_max': 10.0,
                'laser_angle_min': -3.14159,
                'laser_angle_max': 3.14159,
                'laser_angle_increment': 0.017453,
                'noise_level': 0.1,
            }
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('scan', 'scan'),
            ('imu', 'imu'),
            ('pose', 'pose'),
            ('map', 'map'),
        ]
    )
    
    # 遥控节点 (条件启动)
    car_teleop_node = Node(
        package='ros2_car_fun',
        executable='car_teleop',
        name='car_teleop',
        output='screen',
        condition=IfCondition(use_teleop),
        parameters=[
            {
                'linear_speed_step': 0.1,
                'angular_speed_step': 0.1,
                'max_linear_speed': 2.0,
                'max_angular_speed': 1.0,
            }
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
        ]
    )
    
    # 静态变换发布器 - base_link到laser_frame
    static_transform_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    # 静态变换发布器 - base_link到imu_frame
    static_transform_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_frame']
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': get_robot_description(),
                'use_sim_time': True,
            }
        ]
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg='正在启动ROS2车辆演示系统...'
    )
    
    return LaunchDescription([
        # 启动参数
        use_simulator_arg,
        use_teleop_arg,
        config_file_arg,
        
        # 启动信息
        launch_info,
        
        # 节点
        car_controller_node,
        car_simulator_node,
        car_teleop_node,
        
        # 变换发布器
        static_transform_laser,
        static_transform_imu,
        robot_state_publisher,
    ])


def get_robot_description():
    """获取机器人描述 (简化的URDF)"""
    urdf_content = """
<?xml version="1.0"?>
<robot name="simple_car">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""
    return urdf_content
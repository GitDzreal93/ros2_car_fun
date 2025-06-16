#!/usr/bin/env python3
"""
ROS2 Car Fun - Vision Launch File
视觉处理启动文件
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
        description='Hardware mode for vision system'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable debug output'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        choices=['true', 'false'],
        description='Enable camera node'
    )
    
    enable_color_detection_arg = DeclareLaunchArgument(
        'enable_color_detection',
        default_value='true',
        choices=['true', 'false'],
        description='Enable color detection'
    )
    
    enable_face_tracking_arg = DeclareLaunchArgument(
        'enable_face_tracking',
        default_value='true',
        choices=['true', 'false'],
        description='Enable face tracking'
    )
    
    enable_qr_detection_arg = DeclareLaunchArgument(
        'enable_qr_detection',
        default_value='true',
        choices=['true', 'false'],
        description='Enable QR code detection'
    )
    
    enable_object_detection_arg = DeclareLaunchArgument(
        'enable_object_detection',
        default_value='true',
        choices=['true', 'false'],
        description='Enable object detection'
    )
    
    # 获取配置文件路径
    vision_config_file = PathJoinSubstitution([
        FindPackageShare('ros2_car_fun'),
        'config',
        'vision_config.yaml'
    ])
    
    # 获取启动配置
    hardware_mode = LaunchConfiguration('hardware_mode')
    debug_mode = LaunchConfiguration('debug_mode')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_color_detection = LaunchConfiguration('enable_color_detection')
    enable_face_tracking = LaunchConfiguration('enable_face_tracking')
    enable_qr_detection = LaunchConfiguration('enable_qr_detection')
    enable_object_detection = LaunchConfiguration('enable_object_detection')
    
    # 摄像头节点
    camera_node = Node(
        package='ros2_car_fun',
        executable='camera_node',
        name='camera_node',
        output='screen',
        condition=IfCondition(enable_camera),
        parameters=[
            vision_config_file,
            {
                'hardware_mode': hardware_mode,
                'debug_mode': debug_mode,
            }
        ],
        remappings=[
            ('camera/image_raw', 'camera/image_raw'),
            ('camera/image_compressed', 'camera/image_compressed'),
            ('camera/pan_command', 'camera/pan_command'),
            ('camera/tilt_command', 'camera/tilt_command'),
            ('camera/track_target', 'camera/track_target'),
        ]
    )
    
    # 颜色检测节点
    color_detection_node = Node(
        package='ros2_car_fun',
        executable='color_detection_node',
        name='color_detection_node',
        output='screen',
        condition=IfCondition(enable_color_detection),
        parameters=[
            vision_config_file,
            {
                'debug_mode': debug_mode,
            }
        ],
        remappings=[
            ('camera/image_raw', 'camera/image_raw'),
            ('color_detection/detected', 'vision/color_detection/detected'),
            ('color_detection/position', 'vision/color_detection/position'),
            ('color_detection/target', 'vision/color_detection/target'),
            ('color_detection/color', 'vision/color_detection/color'),
            ('color_detection/count', 'vision/color_detection/count'),
            ('color_detection/debug_image', 'vision/color_detection/debug_image'),
            ('camera/track_target', 'camera/track_target'),
        ]
    )
    
    # 人脸跟踪节点
    face_tracking_node = Node(
        package='ros2_car_fun',
        executable='face_tracking_node',
        name='face_tracking_node',
        output='screen',
        condition=IfCondition(enable_face_tracking),
        parameters=[
            vision_config_file,
            {
                'debug_mode': debug_mode,
            }
        ],
        remappings=[
            ('camera/image_raw', 'camera/image_raw'),
            ('face_tracking/detected', 'vision/face_tracking/detected'),
            ('face_tracking/count', 'vision/face_tracking/count'),
            ('face_tracking/position', 'vision/face_tracking/position'),
            ('face_tracking/target', 'vision/face_tracking/target'),
            ('face_tracking/debug_image', 'vision/face_tracking/debug_image'),
            ('camera/track_target', 'camera/track_target'),
        ]
    )
    
    # 二维码检测节点
    qr_detection_node = Node(
        package='ros2_car_fun',
        executable='qr_detection_node',
        name='qr_detection_node',
        output='screen',
        condition=IfCondition(enable_qr_detection),
        parameters=[
            vision_config_file,
            {
                'debug_mode': debug_mode,
            }
        ],
        remappings=[
            ('camera/image_raw', 'camera/image_raw'),
            ('qr_detection/detected', 'vision/qr_detection/detected'),
            ('qr_detection/count', 'vision/qr_detection/count'),
            ('qr_detection/content', 'vision/qr_detection/content'),
            ('qr_detection/position', 'vision/qr_detection/position'),
            ('qr_detection/target', 'vision/qr_detection/target'),
            ('qr_detection/command', 'vision/qr_detection/command'),
            ('qr_detection/debug_image', 'vision/qr_detection/debug_image'),
            ('camera/track_target', 'camera/track_target'),
        ]
    )
    
    # 物体检测节点
    object_detection_node = Node(
        package='ros2_car_fun',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
        condition=IfCondition(enable_object_detection),
        parameters=[
            vision_config_file,
            {
                'debug_mode': debug_mode,
            }
        ],
        remappings=[
            ('camera/image_raw', 'camera/image_raw'),
            ('object_detection/detected', 'vision/object_detection/detected'),
            ('object_detection/count', 'vision/object_detection/count'),
            ('object_detection/shape', 'vision/object_detection/shape'),
            ('object_detection/position', 'vision/object_detection/position'),
            ('object_detection/target', 'vision/object_detection/target'),
            ('object_detection/debug_image', 'vision/object_detection/debug_image'),
            ('camera/track_target', 'camera/track_target'),
        ]
    )
    
    # 视觉处理组
    vision_group = GroupAction([
        camera_node,
        color_detection_node,
        face_tracking_node,
        qr_detection_node,
        object_detection_node,
    ])
    
    # 静态变换发布器 - 摄像头坐标系
    static_transform_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_camera',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'camera_frame'],
        condition=IfCondition(enable_camera)
    )
    
    # 图像查看器（可选）
    image_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        condition=IfCondition('false'),  # 默认不启动，可通过参数控制
        arguments=['/camera/image_raw']
    )
    
    return LaunchDescription([
        # 参数声明
        hardware_mode_arg,
        debug_mode_arg,
        enable_camera_arg,
        enable_color_detection_arg,
        enable_face_tracking_arg,
        enable_qr_detection_arg,
        enable_object_detection_arg,
        
        # 视觉处理节点
        vision_group,
        
        # 静态变换
        static_transform_camera,
        
        # 可选的图像查看器
        # image_viewer,
    ])

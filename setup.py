from setuptools import setup, find_packages

package_name = 'ros2_car_fun'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/car_demo.launch.py',
            'launch/hardware_test.launch.py',
            'launch/sensors.launch.py',
            'launch/vision.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/car_params.yaml',
            'config/hardware_config.yaml',
            'config/sensor_config.yaml',
            'config/vision_config.yaml'
        ]),
    ],
    install_requires=[
        'setuptools',
        'smbus2',  # I2C communication (optional for simulation)
        'opencv-python',  # Computer vision (optional for simulation)
        'numpy',  # Numerical computing
    ],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='ROS2 car control and simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_controller = ros2_car_fun.car_controller:main',
            'car_simulator = ros2_car_fun.car_simulator:main',
            'car_teleop = ros2_car_fun.car_teleop:main',
            'hardware_test = test_hardware:main',
            'ultrasonic_node = ros2_car_fun.sensors.ultrasonic_node:main',
            'line_sensor_node = ros2_car_fun.sensors.line_sensor_node:main',
            'ir_receiver_node = ros2_car_fun.sensors.ir_receiver_node:main',
            'sensor_test = test_sensors:main',
            'integration_test = test_integration:main',
            'camera_node = ros2_car_fun.vision.camera_node:main',
            'color_detection_node = ros2_car_fun.vision.color_detection:main',
            'face_tracking_node = ros2_car_fun.vision.face_tracking:main',
            'qr_detection_node = ros2_car_fun.vision.qr_detection:main',
            'object_detection_node = ros2_car_fun.vision.object_detection:main',
            'vision_test = test_vision:main',
        ],
    },
)
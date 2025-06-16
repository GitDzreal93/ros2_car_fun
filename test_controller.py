#!/usr/bin/env python3
"""
ROS2 Car Fun - Controller Test Script
控制器测试脚本

Test the enhanced car controller with hardware abstraction layer
"""

import sys
import time
import threading

# 模拟ROS2环境用于测试
class MockRclpy:
    """模拟ROS2 rclpy模块"""
    
    @staticmethod
    def init(args=None):
        print("Mock ROS2 initialized")
    
    @staticmethod
    def shutdown():
        print("Mock ROS2 shutdown")
    
    @staticmethod
    def spin(node):
        print(f"Mock spinning node: {node.__class__.__name__}")
        # 模拟运行一段时间
        time.sleep(2)

class MockNode:
    """模拟ROS2 Node基类"""
    
    def __init__(self, name):
        self.name = name
        self.parameters = {}
        self.subscriptions = []
        self.publishers = []
        self.timers = []
        print(f"Mock node created: {name}")
    
    def declare_parameter(self, name, default_value):
        self.parameters[name] = default_value
        print(f"Parameter declared: {name} = {default_value}")
    
    def get_parameter(self, name):
        class MockParameter:
            def __init__(self, value):
                self.value = value
        return MockParameter(self.parameters.get(name))
    
    def create_subscription(self, msg_type, topic, callback, qos):
        print(f"Subscription created: {topic}")
        return f"sub_{topic}"
    
    def create_publisher(self, msg_type, topic, qos):
        print(f"Publisher created: {topic}")
        return f"pub_{topic}"
    
    def create_timer(self, period, callback):
        print(f"Timer created: {period}s")
        return f"timer_{period}"
    
    def get_logger(self):
        class MockLogger:
            def info(self, msg):
                print(f"[INFO] {msg}")
            def debug(self, msg):
                print(f"[DEBUG] {msg}")
            def error(self, msg):
                print(f"[ERROR] {msg}")
        return MockLogger()
    
    def get_clock(self):
        class MockClock:
            def now(self):
                class MockTime:
                    def to_msg(self):
                        return "mock_time"
                return MockTime()
        return MockClock()
    
    def destroy_node(self):
        print(f"Node destroyed: {self.name}")

class MockTwist:
    """模拟Twist消息"""
    def __init__(self):
        self.linear = MockVector3()
        self.angular = MockVector3()

class MockVector3:
    """模拟Vector3"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class MockOdometry:
    """模拟Odometry消息"""
    def __init__(self):
        self.header = MockHeader()
        self.child_frame_id = ""
        self.pose = MockPoseWithCovariance()
        self.twist = MockTwistWithCovariance()

class MockHeader:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""

class MockPoseWithCovariance:
    def __init__(self):
        self.pose = MockPose()

class MockPose:
    def __init__(self):
        self.position = MockPoint()
        self.orientation = MockQuaternion()

class MockPoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class MockQuaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0

class MockTwistWithCovariance:
    def __init__(self):
        self.twist = MockTwist()

class MockJointState:
    def __init__(self):
        self.header = MockHeader()
        self.name = []
        self.position = []
        self.velocity = []

class MockRange:
    def __init__(self):
        self.header = MockHeader()
        self.radiation_type = 0
        self.field_of_view = 0.0
        self.min_range = 0.0
        self.max_range = 0.0
        self.range = 0.0
    
    ULTRASOUND = 0

class MockInt32MultiArray:
    def __init__(self):
        self.data = []

def patch_imports():
    """修补导入，使用模拟对象"""
    import sys
    
    # 创建模拟模块
    mock_rclpy = type(sys)('rclpy')
    mock_rclpy.init = MockRclpy.init
    mock_rclpy.shutdown = MockRclpy.shutdown
    mock_rclpy.spin = MockRclpy.spin
    
    mock_rclpy_node = type(sys)('rclpy.node')
    mock_rclpy_node.Node = MockNode
    
    mock_geometry_msgs = type(sys)('geometry_msgs.msg')
    mock_geometry_msgs.Twist = MockTwist
    
    mock_nav_msgs = type(sys)('nav_msgs.msg')
    mock_nav_msgs.Odometry = MockOdometry
    
    mock_sensor_msgs = type(sys)('sensor_msgs.msg')
    mock_sensor_msgs.JointState = MockJointState
    mock_sensor_msgs.Range = MockRange
    
    mock_std_msgs = type(sys)('std_msgs.msg')
    mock_std_msgs.Bool = bool
    mock_std_msgs.Int32MultiArray = MockInt32MultiArray
    
    # 注册模拟模块
    sys.modules['rclpy'] = mock_rclpy
    sys.modules['rclpy.node'] = mock_rclpy_node
    sys.modules['geometry_msgs'] = type(sys)('geometry_msgs')
    sys.modules['geometry_msgs.msg'] = mock_geometry_msgs
    sys.modules['nav_msgs'] = type(sys)('nav_msgs')
    sys.modules['nav_msgs.msg'] = mock_nav_msgs
    sys.modules['sensor_msgs'] = type(sys)('sensor_msgs')
    sys.modules['sensor_msgs.msg'] = mock_sensor_msgs
    sys.modules['std_msgs'] = type(sys)('std_msgs')
    sys.modules['std_msgs.msg'] = mock_std_msgs

def test_controller_creation(CarController):
    """测试控制器创建"""
    print("\n=== Testing Controller Creation ===")

    try:
        controller = CarController()
        print("✅ Controller created successfully")
        
        # 测试参数
        print(f"Hardware mode: {controller.hardware_mode}")
        print(f"Debug mode: {controller.debug_mode}")
        print(f"Wheel base: {controller.wheel_base}")
        print(f"Speed scale: {controller.speed_scale}")
        
        return controller
    except Exception as e:
        print(f"❌ Controller creation failed: {e}")
        import traceback
        traceback.print_exc()
        return None

def test_velocity_commands(controller):
    """测试速度命令"""
    if not controller:
        return
    
    print("\n=== Testing Velocity Commands ===")
    
    # 创建测试命令
    test_commands = [
        (1.0, 0.0, 0.0),   # 前进
        (-1.0, 0.0, 0.0),  # 后退
        (0.0, 1.0, 0.0),   # 左移
        (0.0, -1.0, 0.0),  # 右移
        (0.0, 0.0, 1.0),   # 左转
        (0.0, 0.0, -1.0),  # 右转
        (1.0, 1.0, 0.5),   # 组合运动
    ]
    
    for i, (vx, vy, omega) in enumerate(test_commands):
        print(f"Test {i+1}: vx={vx}, vy={vy}, omega={omega}")
        
        # 创建模拟Twist消息
        twist = MockTwist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        
        # 发送命令
        try:
            controller.cmd_vel_callback(twist)
            print(f"  ✅ Command processed")
        except Exception as e:
            print(f"  ❌ Command failed: {e}")
        
        time.sleep(0.1)

def test_state_updates(controller):
    """测试状态更新"""
    if not controller:
        return
    
    print("\n=== Testing State Updates ===")
    
    try:
        # 模拟几次状态更新
        for i in range(5):
            controller.update_state()
            print(f"Update {i+1}: x={controller.x:.2f}, y={controller.y:.2f}, theta={controller.theta:.2f}")
            time.sleep(0.02)
        
        print("✅ State updates successful")
    except Exception as e:
        print(f"❌ State update failed: {e}")

def test_sensor_reading(controller):
    """测试传感器读取"""
    if not controller:
        return
    
    print("\n=== Testing Sensor Reading ===")
    
    try:
        controller.read_sensors()
        print("✅ Sensor reading successful")
    except Exception as e:
        print(f"❌ Sensor reading failed: {e}")

def test_cleanup(controller):
    """测试清理"""
    if not controller:
        return
    
    print("\n=== Testing Cleanup ===")
    
    try:
        controller.cleanup()
        print("✅ Cleanup successful")
    except Exception as e:
        print(f"❌ Cleanup failed: {e}")

def main():
    """主测试函数"""
    print("🚗 ROS2 Car Controller Test")
    print("=" * 50)

    # 修补导入
    patch_imports()

    # 现在可以安全导入控制器
    from ros2_car_fun.car_controller import CarController

    try:
        # 测试控制器创建
        controller = test_controller_creation(CarController)
        
        # 测试速度命令
        test_velocity_commands(controller)
        
        # 测试状态更新
        test_state_updates(controller)
        
        # 测试传感器读取
        test_sensor_reading(controller)
        
        # 测试清理
        test_cleanup(controller)
        
        print("\n" + "=" * 50)
        print("🎉 All tests completed!")
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()

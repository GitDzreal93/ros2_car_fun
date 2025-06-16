#!/usr/bin/env python3
"""
ROS2 Car Fun - Sensors Test Script
传感器测试脚本

Test script for all sensor nodes
"""

import sys
import time
import threading
import argparse

# 模拟ROS2环境
def patch_ros2_imports():
    """修补ROS2导入"""
    import sys
    
    # 创建模拟模块
    mock_rclpy = type(sys)('rclpy')
    mock_rclpy.init = lambda args=None: print("Mock ROS2 initialized")
    mock_rclpy.shutdown = lambda: print("Mock ROS2 shutdown")
    mock_rclpy.spin = lambda node: time.sleep(2)
    
    mock_rclpy_node = type(sys)('rclpy.node')
    
    class MockNode:
        def __init__(self, name):
            self.name = name
            self.parameters = {}
            print(f"Mock node created: {name}")
        
        def declare_parameter(self, name, default_value):
            self.parameters[name] = default_value
        
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
            return MockPublisher(topic)
        
        def create_timer(self, period, callback):
            print(f"Timer created: {period}s")
            return f"timer_{period}"
        
        def get_logger(self):
            class MockLogger:
                def info(self, msg): print(f"[INFO] {msg}")
                def debug(self, msg): print(f"[DEBUG] {msg}")
                def error(self, msg): print(f"[ERROR] {msg}")
                def warn(self, msg): print(f"[WARN] {msg}")
            return MockLogger()
        
        def get_clock(self):
            class MockClock:
                def now(self):
                    class MockTime:
                        def to_msg(self): return "mock_time"
                    return MockTime()
            return MockClock()
        
        def destroy_node(self):
            print(f"Node destroyed: {self.name}")
    
    class MockPublisher:
        def __init__(self, topic):
            self.topic = topic
        
        def publish(self, msg):
            print(f"Published to {self.topic}: {type(msg).__name__}")
    
    mock_rclpy_node.Node = MockNode
    
    # 模拟消息类型
    class MockMessage:
        def __init__(self):
            pass
    
    class MockRange(MockMessage):
        ULTRASOUND = 0
        def __init__(self):
            super().__init__()
            self.header = MockHeader()
            self.radiation_type = 0
            self.field_of_view = 0.0
            self.min_range = 0.0
            self.max_range = 0.0
            self.range = 0.0
    
    class MockHeader:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""
    
    class MockFloat32(MockMessage):
        def __init__(self):
            super().__init__()
            self.data = 0.0
    
    class MockBool(MockMessage):
        def __init__(self):
            super().__init__()
            self.data = False
    
    class MockInt32(MockMessage):
        def __init__(self):
            super().__init__()
            self.data = 0
    
    class MockInt32MultiArray(MockMessage):
        def __init__(self):
            super().__init__()
            self.data = []
    
    class MockString(MockMessage):
        def __init__(self):
            super().__init__()
            self.data = ""
    
    class MockTwist(MockMessage):
        def __init__(self):
            super().__init__()
            self.linear = MockVector3()
            self.angular = MockVector3()
    
    class MockVector3(MockMessage):
        def __init__(self):
            super().__init__()
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
    
    class MockPointStamped(MockMessage):
        def __init__(self):
            super().__init__()
            self.header = MockHeader()
            self.point = MockVector3()
    
    # 注册模拟模块
    sys.modules['rclpy'] = mock_rclpy
    sys.modules['rclpy.node'] = mock_rclpy_node
    
    # 消息模块
    mock_sensor_msgs = type(sys)('sensor_msgs.msg')
    mock_sensor_msgs.Range = MockRange
    
    mock_std_msgs = type(sys)('std_msgs.msg')
    mock_std_msgs.Float32 = MockFloat32
    mock_std_msgs.Bool = MockBool
    mock_std_msgs.Int32 = MockInt32
    mock_std_msgs.Int32MultiArray = MockInt32MultiArray
    mock_std_msgs.String = MockString
    
    mock_geometry_msgs = type(sys)('geometry_msgs.msg')
    mock_geometry_msgs.Twist = MockTwist
    mock_geometry_msgs.Vector3 = MockVector3
    mock_geometry_msgs.PointStamped = MockPointStamped
    
    sys.modules['sensor_msgs'] = type(sys)('sensor_msgs')
    sys.modules['sensor_msgs.msg'] = mock_sensor_msgs
    sys.modules['std_msgs'] = type(sys)('std_msgs')
    sys.modules['std_msgs.msg'] = mock_std_msgs
    sys.modules['geometry_msgs'] = type(sys)('geometry_msgs')
    sys.modules['geometry_msgs.msg'] = mock_geometry_msgs


def test_ultrasonic_sensor():
    """测试超声波传感器"""
    print("\n=== Testing Ultrasonic Sensor ===")
    
    try:
        from ros2_car_fun.sensors.ultrasonic_node import UltrasonicNode
        
        node = UltrasonicNode()
        print("✅ Ultrasonic node created successfully")
        
        # 测试仿真功能
        if node.hardware.is_simulation_mode():
            print("Testing simulation features...")
            
            # 设置不同距离
            test_distances = [0.1, 0.5, 1.0, 1.5, 2.0]
            for distance in test_distances:
                node.set_simulation_distance(distance)
                print(f"  Set distance: {distance}m")
                time.sleep(0.1)
        
        # 测试统计信息
        stats = node.get_statistics()
        print(f"Statistics: {stats}")
        
        # 清理
        node.cleanup()
        print("✅ Ultrasonic sensor test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ Ultrasonic sensor test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_line_sensor():
    """测试巡线传感器"""
    print("\n=== Testing Line Sensor ===")
    
    try:
        from ros2_car_fun.sensors.line_sensor_node import LineSensorNode
        
        node = LineSensorNode()
        print("✅ Line sensor node created successfully")
        
        # 测试仿真功能
        if node.hardware.is_simulation_mode():
            print("Testing simulation features...")
            
            # 测试不同传感器模式
            test_patterns = [
                (False, True, True, False),   # 中心
                (False, True, False, False),  # 左偏
                (False, False, True, False),  # 右偏
                (True, False, False, False),  # 大幅左偏
                (False, False, False, True),  # 大幅右偏
                (False, False, False, False), # 丢线
            ]
            
            for i, pattern in enumerate(test_patterns):
                node.set_simulation_sensors(*pattern)
                print(f"  Pattern {i+1}: {pattern}")
                time.sleep(0.1)
        
        # 测试统计信息
        stats = node.get_statistics()
        print(f"Statistics: {stats}")
        
        # 清理
        node.cleanup()
        print("✅ Line sensor test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ Line sensor test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_ir_receiver():
    """测试红外接收器"""
    print("\n=== Testing IR Receiver ===")
    
    try:
        from ros2_car_fun.sensors.ir_receiver_node import IRReceiverNode
        
        node = IRReceiverNode()
        print("✅ IR receiver node created successfully")
        
        # 测试仿真功能
        if node.hardware.is_simulation_mode():
            print("Testing simulation features...")
            
            # 测试不同命令
            test_commands = ['up', 'down', 'four', 'six', 'one', 'three', 'five']
            
            for command in test_commands:
                node.simulate_ir_command(command)
                print(f"  Simulated command: {command}")
                time.sleep(0.2)
        
        # 列出可用命令
        commands = node.list_available_commands()
        print(f"Available commands: {commands[:10]}...")  # 只显示前10个
        
        # 测试统计信息
        stats = node.get_statistics()
        print(f"Statistics: {stats}")
        
        # 清理
        node.cleanup()
        print("✅ IR receiver test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ IR receiver test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_all_sensors():
    """测试所有传感器"""
    print("🔍 Testing All Sensors")
    print("=" * 50)
    
    results = []
    
    # 测试超声波传感器
    results.append(test_ultrasonic_sensor())
    
    # 测试巡线传感器
    results.append(test_line_sensor())
    
    # 测试红外接收器
    results.append(test_ir_receiver())
    
    # 总结
    print("\n" + "=" * 50)
    success_count = sum(results)
    total_count = len(results)
    
    if success_count == total_count:
        print("🎉 All sensor tests passed!")
    else:
        print(f"⚠️  {success_count}/{total_count} sensor tests passed")
    
    return success_count == total_count


def interactive_sensor_test():
    """交互式传感器测试"""
    print("\n=== Interactive Sensor Test ===")
    print("Commands:")
    print("  1 - Test ultrasonic sensor")
    print("  2 - Test line sensor")
    print("  3 - Test IR receiver")
    print("  a - Test all sensors")
    print("  q - Quit")
    
    while True:
        try:
            cmd = input("\nEnter command: ").strip().lower()
            
            if cmd == '1':
                test_ultrasonic_sensor()
            elif cmd == '2':
                test_line_sensor()
            elif cmd == '3':
                test_ir_receiver()
            elif cmd == 'a':
                test_all_sensors()
            elif cmd == 'q':
                break
            else:
                print("Unknown command")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("Interactive test ended")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Sensor Test Script')
    parser.add_argument('--sensor', choices=['ultrasonic', 'line', 'ir', 'all'],
                       default='all', help='Sensor to test')
    parser.add_argument('--interactive', action='store_true', help='Interactive mode')
    
    args = parser.parse_args()
    
    print("🔍 ROS2 Car Fun - Sensor Test")
    print("=" * 50)
    
    # 修补ROS2导入
    patch_ros2_imports()
    
    try:
        if args.interactive:
            interactive_sensor_test()
        else:
            if args.sensor == 'ultrasonic':
                test_ultrasonic_sensor()
            elif args.sensor == 'line':
                test_line_sensor()
            elif args.sensor == 'ir':
                test_ir_receiver()
            elif args.sensor == 'all':
                test_all_sensors()
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

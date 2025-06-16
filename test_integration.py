#!/usr/bin/env python3
"""
ROS2 Car Fun - Integration Test Script
集成测试脚本

Test the integration between hardware, sensors, and controller
"""

import sys
import time
import threading
import argparse

# 使用之前的模拟环境
import test_sensors
import test_controller


def test_hardware_sensor_integration():
    """测试硬件和传感器集成"""
    print("\n=== Testing Hardware-Sensor Integration ===")
    
    try:
        # 修补ROS2导入
        test_sensors.patch_ros2_imports()
        
        # 导入硬件接口
        from ros2_car_fun.hardware import HardwareInterface, CarMode
        
        # 创建硬件接口
        hardware = HardwareInterface(CarMode.SIMULATION, debug=True)
        print("✅ Hardware interface created")
        
        # 测试传感器读取
        print("Testing sensor readings...")
        
        # 设置仿真数据
        hardware.set_simulation_ultrasonic_distance(50.0)  # 50cm
        hardware.set_simulation_line_sensors(False, True, True, False)  # 中心线
        
        # 读取传感器
        distance = hardware.read_ultrasonic_distance()
        line_sensors = hardware.read_line_sensors()
        line_position = hardware.get_line_position()
        
        print(f"  超声波距离: {distance}cm")
        print(f"  巡线传感器: {line_sensors}")
        print(f"  线位置: {line_position}")
        
        # 测试运动控制
        print("Testing motion control...")
        hardware.move_forward(100)
        time.sleep(0.1)
        hardware.move_left(100)
        time.sleep(0.1)
        hardware.rotate_right(100)
        time.sleep(0.1)
        hardware.stop()
        
        # 测试LED和蜂鸣器
        print("Testing peripherals...")
        from ros2_car_fun.hardware import LEDColor
        hardware.set_all_leds(True, LEDColor.GREEN)
        time.sleep(0.2)
        hardware.beep(0.1)
        hardware.turn_off_all_leds()
        
        # 清理
        hardware.cleanup()
        print("✅ Hardware-sensor integration test passed")
        
        return True
        
    except Exception as e:
        print(f"❌ Hardware-sensor integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_sensor_controller_integration():
    """测试传感器和控制器集成"""
    print("\n=== Testing Sensor-Controller Integration ===")
    
    try:
        # 导入传感器节点
        from ros2_car_fun.sensors.ultrasonic_node import UltrasonicNode
        from ros2_car_fun.sensors.line_sensor_node import LineSensorNode
        from ros2_car_fun.sensors.ir_receiver_node import IRReceiverNode
        
        # 创建传感器节点
        ultrasonic = UltrasonicNode()
        line_sensor = LineSensorNode()
        ir_receiver = IRReceiverNode()
        
        print("✅ All sensor nodes created")
        
        # 测试传感器数据处理
        print("Testing sensor data processing...")
        
        # 超声波传感器测试
        ultrasonic.set_simulation_distance(0.8)  # 80cm
        stats = ultrasonic.get_statistics()
        print(f"  超声波统计: {stats}")
        
        # 巡线传感器测试
        line_sensor.set_simulation_sensors(False, True, False, False)  # 左偏
        error = line_sensor.get_line_following_error()
        pattern = line_sensor.get_sensor_pattern()
        print(f"  巡线误差: {error}, 模式: {pattern}")
        
        # 红外遥控测试
        ir_receiver.simulate_ir_command('up')
        commands = ir_receiver.list_available_commands()
        print(f"  可用命令数: {len(commands)}")
        
        # 清理
        ultrasonic.cleanup()
        line_sensor.cleanup()
        ir_receiver.cleanup()
        
        print("✅ Sensor-controller integration test passed")
        
        return True
        
    except Exception as e:
        print(f"❌ Sensor-controller integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_full_system_simulation():
    """测试完整系统仿真"""
    print("\n=== Testing Full System Simulation ===")
    
    try:
        # 导入所有组件
        from ros2_car_fun.hardware import HardwareInterface, CarMode, LEDColor
        from ros2_car_fun.sensors.ultrasonic_node import UltrasonicNode
        from ros2_car_fun.sensors.line_sensor_node import LineSensorNode
        
        # 创建系统组件
        hardware = HardwareInterface(CarMode.SIMULATION, debug=False)
        ultrasonic = UltrasonicNode()
        line_sensor = LineSensorNode()
        
        print("✅ Full system components created")
        
        # 模拟巡线场景
        print("Simulating line following scenario...")
        
        scenarios = [
            ("直线前进", (False, True, True, False), 100, 0, 0),
            ("左偏修正", (False, True, False, False), 80, 20, 0),
            ("右偏修正", (False, False, True, False), 80, -20, 0),
            ("大幅左偏", (True, False, False, False), 60, 40, 0),
            ("大幅右偏", (False, False, False, True), 60, -40, 0),
            ("丢线停止", (False, False, False, False), 0, 0, 0),
        ]
        
        for i, (desc, sensors, forward, lateral, rotation) in enumerate(scenarios):
            print(f"  场景 {i+1}: {desc}")
            
            # 设置传感器状态
            line_sensor.set_simulation_sensors(*sensors)
            
            # 获取线位置
            position = line_sensor.get_line_following_error()
            detected = line_sensor.is_line_detected()
            
            # 控制车辆运动
            if detected:
                hardware.move(lateral, forward, rotation)
                hardware.set_all_leds(True, LEDColor.GREEN)
            else:
                hardware.stop()
                hardware.set_all_leds(True, LEDColor.RED)
                hardware.beep(0.1)
            
            print(f"    传感器: {sensors}, 位置: {position:.2f}, 检测: {detected}")
            print(f"    控制: 前进={forward}, 侧移={lateral}, 旋转={rotation}")
            
            time.sleep(0.5)
        
        # 模拟避障场景
        print("Simulating obstacle avoidance scenario...")
        
        distances = [200, 150, 100, 50, 30, 15, 10]  # cm
        
        for distance in distances:
            ultrasonic.set_simulation_distance(distance)
            
            if distance > 30:
                # 安全距离，正常前进
                hardware.move_forward(100)
                hardware.set_all_leds(True, LEDColor.GREEN)
                action = "前进"
            elif distance > 15:
                # 警告距离，减速
                hardware.move_forward(50)
                hardware.set_all_leds(True, LEDColor.YELLOW)
                action = "减速"
            else:
                # 危险距离，停止并后退
                hardware.move_backward(80)
                hardware.set_all_leds(True, LEDColor.RED)
                hardware.beep(0.1)
                action = "后退"
            
            print(f"  距离: {distance}cm -> {action}")
            time.sleep(0.3)
        
        # 清理
        hardware.stop()
        hardware.turn_off_all_leds()
        hardware.cleanup()
        ultrasonic.cleanup()
        line_sensor.cleanup()
        
        print("✅ Full system simulation test passed")
        
        return True
        
    except Exception as e:
        print(f"❌ Full system simulation test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_performance_monitoring():
    """测试性能监控"""
    print("\n=== Testing Performance Monitoring ===")
    
    try:
        from ros2_car_fun.hardware import HardwareInterface, CarMode
        
        hardware = HardwareInterface(CarMode.SIMULATION, debug=False)
        
        # 性能测试
        start_time = time.time()
        iterations = 100
        
        for i in range(iterations):
            # 传感器读取
            distance = hardware.read_ultrasonic_distance()
            sensors = hardware.read_line_sensors()
            
            # 运动控制
            hardware.move(i % 10, (i * 2) % 10, (i * 3) % 10)
            
            if i % 20 == 0:
                print(f"  进度: {i}/{iterations}")
        
        end_time = time.time()
        total_time = end_time - start_time
        avg_time = total_time / iterations
        
        print(f"  总时间: {total_time:.3f}s")
        print(f"  平均时间: {avg_time:.6f}s/iteration")
        print(f"  频率: {1/avg_time:.1f} Hz")
        
        # 获取系统状态
        status = hardware.get_status()
        print(f"  系统状态: {status}")
        
        hardware.cleanup()
        
        print("✅ Performance monitoring test passed")
        
        return True
        
    except Exception as e:
        print(f"❌ Performance monitoring test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_integration_tests():
    """运行所有集成测试"""
    print("🔧 ROS2 Car Fun - Integration Tests")
    print("=" * 60)
    
    tests = [
        ("Hardware-Sensor Integration", test_hardware_sensor_integration),
        ("Sensor-Controller Integration", test_sensor_controller_integration),
        ("Full System Simulation", test_full_system_simulation),
        ("Performance Monitoring", test_performance_monitoring),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n🧪 Running: {test_name}")
        print("-" * 40)
        
        try:
            result = test_func()
            results.append(result)
            
            if result:
                print(f"✅ {test_name} PASSED")
            else:
                print(f"❌ {test_name} FAILED")
                
        except Exception as e:
            print(f"❌ {test_name} CRASHED: {e}")
            results.append(False)
    
    # 总结
    print("\n" + "=" * 60)
    passed = sum(results)
    total = len(results)
    
    print(f"📊 Integration Test Results: {passed}/{total} passed")
    
    if passed == total:
        print("🎉 All integration tests passed!")
        print("\n✨ System is ready for advanced behaviors!")
    else:
        print("⚠️  Some integration tests failed")
        print("🔧 Please check the error messages above")
    
    return passed == total


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Integration Test Script')
    parser.add_argument('--test', choices=['hardware', 'sensors', 'simulation', 'performance', 'all'],
                       default='all', help='Test to run')
    
    args = parser.parse_args()
    
    # 修补ROS2导入
    test_sensors.patch_ros2_imports()
    
    try:
        if args.test == 'hardware':
            test_hardware_sensor_integration()
        elif args.test == 'sensors':
            test_sensor_controller_integration()
        elif args.test == 'simulation':
            test_full_system_simulation()
        elif args.test == 'performance':
            test_performance_monitoring()
        elif args.test == 'all':
            run_all_integration_tests()
        
    except Exception as e:
        print(f"\n❌ Integration test failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

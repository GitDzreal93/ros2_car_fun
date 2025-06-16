#!/usr/bin/env python3
"""
ROS2 Car Fun - Full System Test Script
完整系统测试脚本

Test the complete integrated system including hardware, sensors, and vision
"""

import sys
import time
import threading
import argparse

# 导入之前的测试模块
import test_sensors
import test_vision
import test_integration


def test_complete_system():
    """测试完整系统"""
    print("\n🚗 === Complete System Test ===")
    
    try:
        # 修补所有导入
        test_vision.patch_vision_imports()
        
        # 导入所有组件
        from ros2_car_fun.hardware import HardwareInterface, CarMode, LEDColor
        from ros2_car_fun.sensors.ultrasonic_node import UltrasonicNode
        from ros2_car_fun.sensors.line_sensor_node import LineSensorNode
        from ros2_car_fun.vision.camera_node import CameraNode
        from ros2_car_fun.vision.color_detection import ColorDetectionNode
        from ros2_car_fun.vision.face_tracking import FaceTrackingNode
        
        print("✅ All system components imported successfully")
        
        # 创建系统组件
        hardware = HardwareInterface(CarMode.SIMULATION, debug=False)
        ultrasonic = UltrasonicNode()
        line_sensor = LineSensorNode()
        camera = CameraNode()
        color_detector = ColorDetectionNode()
        face_tracker = FaceTrackingNode()
        
        print("✅ All system components created successfully")
        
        # 模拟完整的智能小车场景
        print("\n🎬 Simulating complete smart car scenarios...")
        
        # 场景1: 启动和自检
        print("\n📋 Scenario 1: System Startup and Self-Check")
        hardware.beep_pattern("101", 0.1)  # 启动提示音
        hardware.set_all_leds(True, LEDColor.BLUE)  # 蓝色表示启动
        camera.reset_camera_position()  # 摄像头回中
        time.sleep(1)
        
        # 场景2: 传感器检测
        print("\n🔍 Scenario 2: Sensor Detection")
        hardware.set_simulation_ultrasonic_distance(80.0)  # 80cm
        hardware.set_simulation_line_sensors(False, True, True, False)  # 中心线
        
        distance = hardware.read_ultrasonic_distance()
        line_sensors = hardware.read_line_sensors()
        line_position = hardware.get_line_position()
        
        print(f"  超声波距离: {distance}cm")
        print(f"  巡线传感器: {line_sensors}")
        print(f"  线位置: {line_position}")
        
        # 场景3: 巡线行为
        print("\n🛤️  Scenario 3: Line Following Behavior")
        hardware.set_all_leds(True, LEDColor.GREEN)  # 绿色表示巡线
        
        line_scenarios = [
            ("直线", (False, True, True, False), 100, 0),
            ("左偏", (False, True, False, False), 80, 20),
            ("右偏", (False, False, True, False), 80, -20),
            ("丢线", (False, False, False, False), 0, 0),
        ]
        
        for desc, sensors, forward, lateral in line_scenarios:
            print(f"  {desc}: 传感器={sensors} -> 前进={forward}, 侧移={lateral}")
            line_sensor.set_simulation_sensors(*sensors)
            hardware.move(lateral, forward, 0)
            time.sleep(0.5)
        
        # 场景4: 避障行为
        print("\n🚧 Scenario 4: Obstacle Avoidance Behavior")
        hardware.set_all_leds(True, LEDColor.YELLOW)  # 黄色表示避障
        
        obstacle_distances = [100, 50, 30, 15, 10]  # cm
        
        for distance in obstacle_distances:
            ultrasonic.set_simulation_distance(distance)
            
            if distance > 30:
                action = "前进"
                hardware.move_forward(100)
                led_color = LEDColor.GREEN
            elif distance > 15:
                action = "减速"
                hardware.move_forward(50)
                led_color = LEDColor.YELLOW
            else:
                action = "停止后退"
                hardware.move_backward(80)
                hardware.beep(0.1)
                led_color = LEDColor.RED
            
            hardware.set_all_leds(True, led_color)
            print(f"  距离={distance}cm -> {action}")
            time.sleep(0.3)
        
        # 场景5: 视觉跟踪
        print("\n👁️  Scenario 5: Vision Tracking")
        hardware.set_all_leds(True, LEDColor.PURPLE)  # 紫色表示视觉跟踪
        
        # 模拟颜色跟踪
        color_detector.set_target_color('red')
        print("  颜色跟踪: 目标颜色设置为红色")
        
        # 模拟人脸跟踪
        face_stats = face_tracker.get_face_statistics()
        print(f"  人脸跟踪: {face_stats}")
        
        # 场景6: 遥控模式
        print("\n🎮 Scenario 6: Remote Control Mode")
        hardware.set_all_leds(True, LEDColor.CYAN)  # 青色表示遥控
        
        remote_commands = [
            ("前进", lambda: hardware.move_forward(120)),
            ("后退", lambda: hardware.move_backward(120)),
            ("左移", lambda: hardware.move_left(120)),
            ("右移", lambda: hardware.move_right(120)),
            ("左转", lambda: hardware.rotate_left(120)),
            ("右转", lambda: hardware.rotate_right(120)),
        ]
        
        for cmd_name, cmd_func in remote_commands:
            print(f"  遥控命令: {cmd_name}")
            cmd_func()
            time.sleep(0.3)
        
        # 场景7: 系统关闭
        print("\n🔌 Scenario 7: System Shutdown")
        hardware.stop()
        hardware.turn_off_all_leds()
        hardware.beep_pattern("110", 0.2)  # 关闭提示音
        
        # 清理所有组件
        hardware.cleanup()
        ultrasonic.cleanup()
        line_sensor.cleanup()
        camera.cleanup()
        
        print("✅ Complete system test passed!")
        
        return True
        
    except Exception as e:
        print(f"❌ Complete system test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_system_performance():
    """测试系统性能"""
    print("\n⚡ === System Performance Test ===")
    
    try:
        test_vision.patch_vision_imports()
        
        from ros2_car_fun.hardware import HardwareInterface, CarMode
        
        hardware = HardwareInterface(CarMode.SIMULATION, debug=False)
        
        # 性能测试参数
        iterations = 1000
        start_time = time.time()
        
        print(f"Running {iterations} iterations...")
        
        for i in range(iterations):
            # 传感器读取
            distance = hardware.read_ultrasonic_distance()
            sensors = hardware.read_line_sensors()
            
            # 运动控制
            hardware.move(i % 50, (i * 2) % 50, (i * 3) % 50)
            
            # LED控制
            if i % 100 == 0:
                from ros2_car_fun.hardware import LEDColor
                colors = [LEDColor.RED, LEDColor.GREEN, LEDColor.BLUE]
                hardware.set_all_leds(True, colors[i // 100 % 3])
            
            if i % 200 == 0:
                print(f"  进度: {i}/{iterations} ({i/iterations*100:.1f}%)")
        
        end_time = time.time()
        total_time = end_time - start_time
        avg_time = total_time / iterations
        frequency = 1 / avg_time
        
        print(f"\n📊 Performance Results:")
        print(f"  总时间: {total_time:.3f}s")
        print(f"  平均时间: {avg_time:.6f}s/iteration")
        print(f"  处理频率: {frequency:.1f} Hz")
        print(f"  吞吐量: {iterations/total_time:.1f} ops/s")
        
        # 性能评估
        if frequency > 1000:
            print("🚀 性能评级: 优秀 (>1000 Hz)")
        elif frequency > 500:
            print("✅ 性能评级: 良好 (>500 Hz)")
        elif frequency > 100:
            print("⚠️  性能评级: 一般 (>100 Hz)")
        else:
            print("❌ 性能评级: 需要优化 (<100 Hz)")
        
        hardware.cleanup()
        
        print("✅ System performance test completed!")
        
        return True
        
    except Exception as e:
        print(f"❌ System performance test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_system_reliability():
    """测试系统可靠性"""
    print("\n🛡️  === System Reliability Test ===")
    
    try:
        test_vision.patch_vision_imports()
        
        from ros2_car_fun.hardware import HardwareInterface, CarMode
        
        # 可靠性测试
        test_duration = 10  # 秒
        error_count = 0
        success_count = 0
        
        hardware = HardwareInterface(CarMode.SIMULATION, debug=False)
        
        print(f"Running reliability test for {test_duration} seconds...")
        
        start_time = time.time()
        
        while time.time() - start_time < test_duration:
            try:
                # 随机操作
                import random
                
                operation = random.choice([
                    lambda: hardware.read_ultrasonic_distance(),
                    lambda: hardware.read_line_sensors(),
                    lambda: hardware.move_forward(random.randint(50, 150)),
                    lambda: hardware.move_left(random.randint(50, 150)),
                    lambda: hardware.rotate_right(random.randint(50, 150)),
                    lambda: hardware.stop(),
                ])
                
                operation()
                success_count += 1
                
            except Exception as e:
                error_count += 1
                if error_count <= 5:  # 只显示前5个错误
                    print(f"  错误 {error_count}: {e}")
            
            time.sleep(0.01)  # 100Hz测试频率
        
        total_operations = success_count + error_count
        success_rate = success_count / total_operations if total_operations > 0 else 0
        
        print(f"\n📊 Reliability Results:")
        print(f"  总操作数: {total_operations}")
        print(f"  成功操作: {success_count}")
        print(f"  失败操作: {error_count}")
        print(f"  成功率: {success_rate:.3%}")
        
        # 可靠性评估
        if success_rate >= 0.99:
            print("🛡️  可靠性评级: 优秀 (≥99%)")
        elif success_rate >= 0.95:
            print("✅ 可靠性评级: 良好 (≥95%)")
        elif success_rate >= 0.90:
            print("⚠️  可靠性评级: 一般 (≥90%)")
        else:
            print("❌ 可靠性评级: 需要改进 (<90%)")
        
        hardware.cleanup()
        
        print("✅ System reliability test completed!")
        
        return success_rate >= 0.90
        
    except Exception as e:
        print(f"❌ System reliability test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def run_all_system_tests():
    """运行所有系统测试"""
    print("🚗 ROS2 Car Fun - Full System Tests")
    print("=" * 60)
    
    tests = [
        ("Complete System Integration", test_complete_system),
        ("System Performance", test_system_performance),
        ("System Reliability", test_system_reliability),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n🧪 Running: {test_name}")
        print("-" * 50)
        
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
    
    print(f"📊 Full System Test Results: {passed}/{total} passed")
    
    if passed == total:
        print("🎉 All system tests passed!")
        print("\n🚀 System is ready for deployment!")
        print("\n✨ Congratulations! Your ROS2 smart car is fully integrated!")
    else:
        print("⚠️  Some system tests failed")
        print("🔧 Please check the error messages above")
    
    return passed == total


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Full System Test Script')
    parser.add_argument('--test', choices=['complete', 'performance', 'reliability', 'all'],
                       default='all', help='Test to run')
    
    args = parser.parse_args()
    
    print("🚗 ROS2 Car Fun - Full System Test")
    print("=" * 50)
    
    try:
        if args.test == 'complete':
            test_complete_system()
        elif args.test == 'performance':
            test_system_performance()
        elif args.test == 'reliability':
            test_system_reliability()
        elif args.test == 'all':
            run_all_system_tests()
        
    except Exception as e:
        print(f"\n❌ Full system test failed with error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

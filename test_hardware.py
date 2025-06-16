#!/usr/bin/env python3
"""
ROS2 Car Fun - Hardware Test Script
硬件测试脚本

Test script for the hardware abstraction layer.
Can run in both simulation and hardware modes.
"""

import sys
import time
import argparse
from ros2_car_fun.hardware import HardwareInterface, CarMode, LEDColor


def test_basic_movement(hw: HardwareInterface):
    """测试基本运动功能"""
    print("\n=== Testing Basic Movement ===")
    
    movements = [
        ("Forward", lambda: hw.move_forward(100)),
        ("Backward", lambda: hw.move_backward(100)),
        ("Left", lambda: hw.move_left(100)),
        ("Right", lambda: hw.move_right(100)),
        ("Rotate Left", lambda: hw.rotate_left(100)),
        ("Rotate Right", lambda: hw.rotate_right(100)),
    ]
    
    for name, movement in movements:
        print(f"Testing: {name}")
        movement()
        time.sleep(1.0)
        hw.stop()
        time.sleep(0.5)
    
    print("Basic movement test completed!")


def test_advanced_movement(hw: HardwareInterface):
    """测试高级运动功能"""
    print("\n=== Testing Advanced Movement ===")
    
    # 测试组合运动
    print("Testing combined motion (forward + rotate)")
    hw.move(0, 100, 50)  # 前进同时左转
    time.sleep(2.0)
    hw.stop()
    
    print("Testing diagonal movement")
    hw.move(70, 70, 0)  # 右前斜向
    time.sleep(1.0)
    hw.move(-70, 70, 0)  # 左前斜向
    time.sleep(1.0)
    hw.stop()
    
    print("Advanced movement test completed!")


def test_servo_control(hw: HardwareInterface):
    """测试舵机控制"""
    print("\n=== Testing Servo Control ===")
    
    # 测试摄像头云台
    print("Testing camera pan/tilt")
    
    # 水平扫描
    for angle in [0, 45, 90, 135, 180]:
        print(f"Pan to {angle} degrees")
        hw.set_camera_pan(angle)
        time.sleep(0.5)
    
    # 垂直扫描
    for angle in [0, 45, 90]:
        print(f"Tilt to {angle} degrees")
        hw.set_camera_tilt(angle)
        time.sleep(0.5)
    
    # 回中
    print("Centering camera")
    hw.center_camera()
    time.sleep(1.0)
    
    print("Servo control test completed!")


def test_led_control(hw: HardwareInterface):
    """测试LED控制"""
    print("\n=== Testing LED Control ===")
    
    # 测试所有颜色
    print("Testing all LED colors")
    for color in LEDColor:
        print(f"Setting all LEDs to {color.name}")
        hw.set_all_leds(True, color)
        time.sleep(0.8)
    
    hw.turn_off_all_leds()
    time.sleep(0.5)
    
    # 测试单个LED
    print("Testing individual LEDs")
    for i in range(5):  # 测试前5个LED
        hw.set_led(i, True, LEDColor.BLUE)
        time.sleep(0.2)
        hw.set_led(i, False)
    
    # 测试呼吸灯效果
    print("Testing breathing effect")
    hw.led_breathing_effect(LEDColor.GREEN, 2.0)
    
    print("LED control test completed!")


def test_sensor_reading(hw: HardwareInterface):
    """测试传感器读取"""
    print("\n=== Testing Sensor Reading ===")
    
    # 测试超声波传感器
    print("Testing ultrasonic sensor")
    for i in range(5):
        distance = hw.read_ultrasonic_distance()
        print(f"Ultrasonic distance: {distance} cm")
        time.sleep(0.5)
    
    # 测试巡线传感器
    print("Testing line sensors")
    for i in range(5):
        sensors = hw.read_line_sensors()
        if sensors:
            left_outer, left_inner, right_inner, right_outer = sensors
            print(f"Line sensors: L_OUT={left_outer} L_IN={left_inner} R_IN={right_inner} R_OUT={right_outer}")
            
            position = hw.get_line_position()
            if position is not None:
                print(f"Line position: {position:.2f}")
        time.sleep(0.5)
    
    print("Sensor reading test completed!")


def test_beeper(hw: HardwareInterface):
    """测试蜂鸣器"""
    print("\n=== Testing Beeper ===")
    
    # 简单蜂鸣
    print("Simple beep")
    hw.beep(0.2)
    time.sleep(0.5)
    
    # 蜂鸣模式
    patterns = [
        ("Short beeps", "101010"),
        ("Long-short", "1100110011"),
        ("SOS", "111000111000111"),
    ]
    
    for name, pattern in patterns:
        print(f"Playing pattern: {name}")
        hw.beep_pattern(pattern, 0.1)
        time.sleep(1.0)
    
    print("Beeper test completed!")


def test_simulation_features(hw: HardwareInterface):
    """测试仿真模式特有功能"""
    if not hw.is_simulation_mode():
        print("Skipping simulation tests (not in simulation mode)")
        return
    
    print("\n=== Testing Simulation Features ===")
    
    # 设置仿真传感器数据
    print("Setting simulation sensor data")
    
    # 设置超声波距离
    hw.set_simulation_ultrasonic_distance(50.0)
    distance = hw.read_ultrasonic_distance()
    print(f"Set distance to 50cm, read: {distance}cm")
    
    # 设置巡线传感器
    hw.set_simulation_line_sensors(True, False, False, True)
    sensors = hw.read_line_sensors()
    print(f"Set line sensors to outer only, read: {sensors}")
    
    position = hw.get_line_position()
    print(f"Line position: {position}")
    
    print("Simulation features test completed!")


def interactive_test(hw: HardwareInterface):
    """交互式测试"""
    print("\n=== Interactive Test Mode ===")
    print("Commands:")
    print("  w/s - forward/backward")
    print("  a/d - left/right")
    print("  q/e - rotate left/right")
    print("  space - stop")
    print("  l - toggle LEDs")
    print("  b - beep")
    print("  r - read sensors")
    print("  x - exit")
    
    led_state = False
    
    try:
        while True:
            cmd = input("Enter command: ").strip().lower()
            
            if cmd == 'w':
                hw.move_forward(150)
            elif cmd == 's':
                hw.move_backward(150)
            elif cmd == 'a':
                hw.move_left(150)
            elif cmd == 'd':
                hw.move_right(150)
            elif cmd == 'q':
                hw.rotate_left(150)
            elif cmd == 'e':
                hw.rotate_right(150)
            elif cmd == ' ' or cmd == 'stop':
                hw.stop()
            elif cmd == 'l':
                led_state = not led_state
                hw.set_all_leds(led_state, LEDColor.WHITE)
                print(f"LEDs {'ON' if led_state else 'OFF'}")
            elif cmd == 'b':
                hw.beep()
            elif cmd == 'r':
                distance = hw.read_ultrasonic_distance()
                sensors = hw.read_line_sensors()
                print(f"Distance: {distance}cm, Line sensors: {sensors}")
            elif cmd == 'x':
                break
            else:
                print("Unknown command")
                
    except KeyboardInterrupt:
        pass
    finally:
        hw.stop()
        hw.turn_off_all_leds()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Hardware Test Script')
    parser.add_argument('--mode', choices=['hardware', 'simulation'], 
                       default='simulation', help='Test mode')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--interactive', action='store_true', help='Interactive test mode')
    parser.add_argument('--test', choices=['movement', 'servo', 'led', 'sensor', 'beeper', 'all'],
                       default='all', help='Specific test to run')
    
    args = parser.parse_args()
    
    # 设置运行模式
    mode = CarMode.HARDWARE if args.mode == 'hardware' else CarMode.SIMULATION
    
    print(f"Starting hardware test in {mode.value} mode...")
    
    # 初始化硬件接口
    hw = HardwareInterface(mode, debug=args.debug)
    
    try:
        if args.interactive:
            interactive_test(hw)
        else:
            # 运行指定测试
            if args.test in ['movement', 'all']:
                test_basic_movement(hw)
                test_advanced_movement(hw)
            
            if args.test in ['servo', 'all']:
                test_servo_control(hw)
            
            if args.test in ['led', 'all']:
                test_led_control(hw)
            
            if args.test in ['sensor', 'all']:
                test_sensor_reading(hw)
            
            if args.test in ['beeper', 'all']:
                test_beeper(hw)
            
            if args.test == 'all' and mode == CarMode.SIMULATION:
                test_simulation_features(hw)
        
        # 显示状态
        status = hw.get_status()
        print(f"\nFinal status: {status}")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        hw.cleanup()
        print("Test completed!")


if __name__ == '__main__':
    main()

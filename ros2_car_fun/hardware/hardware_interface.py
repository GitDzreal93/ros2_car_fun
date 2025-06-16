#!/usr/bin/env python3
"""
ROS2 Car Fun - Hardware Interface
硬件抽象层接口

Provides a unified interface for all hardware components,
supporting both simulation and real hardware modes.
"""

import time
from typing import Optional, Tuple, Dict, Any
from enum import Enum

from .raspbot_driver import RaspbotDriver, CarMode
from .mecanum_controller import MecanumController


class LEDColor(Enum):
    """LED颜色枚举"""
    RED = 0
    GREEN = 1
    BLUE = 2
    YELLOW = 3
    PURPLE = 4
    CYAN = 5
    WHITE = 6


class HardwareInterface:
    """
    硬件抽象层接口
    
    统一管理所有硬件组件，提供高级控制接口
    """
    
    def __init__(self, mode: CarMode = CarMode.HARDWARE, debug: bool = False):
        """
        初始化硬件接口
        
        Args:
            mode: 运行模式
            debug: 调试模式
        """
        self.mode = mode
        self.debug = debug
        
        # 初始化底层驱动
        self.driver = RaspbotDriver(mode)
        
        # 初始化运动控制器
        self.mecanum = MecanumController(self.driver, debug)
        
        # 状态变量
        self._is_initialized = True
        self._last_ultrasonic_reading = None
        self._last_line_sensor_reading = None
        
        print(f"HardwareInterface initialized in {mode.value} mode")
    
    # ==================== 运动控制接口 ====================
    
    def move(self, vx: float, vy: float, omega: float = 0.0):
        """
        控制车辆运动
        
        Args:
            vx: X方向速度 (-255 to 255, 正值向右)
            vy: Y方向速度 (-255 to 255, 正值向前)
            omega: 角速度 (-255 to 255, 正值逆时针)
        """
        if not self._is_initialized:
            return False
        
        self.mecanum.move_velocity(vx, vy, omega)
        return True
    
    def move_forward(self, speed: float) -> bool:
        """前进"""
        return self.move(0, speed, 0)
    
    def move_backward(self, speed: float) -> bool:
        """后退"""
        return self.move(0, -speed, 0)
    
    def move_left(self, speed: float) -> bool:
        """左移"""
        return self.move(-speed, 0, 0)
    
    def move_right(self, speed: float) -> bool:
        """右移"""
        return self.move(speed, 0, 0)
    
    def rotate_left(self, speed: float) -> bool:
        """左转"""
        return self.move(0, 0, speed)
    
    def rotate_right(self, speed: float) -> bool:
        """右转"""
        return self.move(0, 0, -speed)
    
    def stop(self) -> bool:
        """停止运动"""
        if not self._is_initialized:
            return False
        
        self.mecanum.stop()
        return True
    
    def emergency_stop(self) -> bool:
        """紧急停止"""
        if not self._is_initialized:
            return False
        
        self.mecanum.emergency_stop()
        return True
    
    # ==================== 舵机控制接口 ====================
    
    def set_servo_angle(self, servo_id: int, angle: float) -> bool:
        """
        设置舵机角度
        
        Args:
            servo_id: 舵机ID (1或2)
            angle: 角度 (0-180度)
        """
        if not self._is_initialized:
            return False
        
        return self.driver.ctrl_servo(servo_id, int(angle))
    
    def set_camera_pan(self, angle: float) -> bool:
        """设置摄像头水平角度"""
        return self.set_servo_angle(1, angle)
    
    def set_camera_tilt(self, angle: float) -> bool:
        """设置摄像头垂直角度"""
        return self.set_servo_angle(2, angle)
    
    def center_camera(self) -> bool:
        """摄像头回中"""
        return (self.set_camera_pan(90) and self.set_camera_tilt(90))
    
    # ==================== LED控制接口 ====================
    
    def set_all_leds(self, state: bool, color: LEDColor = LEDColor.WHITE) -> bool:
        """设置所有LED"""
        if not self._is_initialized:
            return False
        
        return self.driver.ctrl_led_all(state, color.value)
    
    def set_led(self, led_id: int, state: bool, color: LEDColor = LEDColor.WHITE) -> bool:
        """设置单个LED"""
        if not self._is_initialized:
            return False
        
        return self.driver.ctrl_led_single(led_id, state, color.value)
    
    def turn_off_all_leds(self) -> bool:
        """关闭所有LED"""
        return self.set_all_leds(False)
    
    def led_breathing_effect(self, color: LEDColor, duration: float = 3.0):
        """LED呼吸灯效果"""
        if not self._is_initialized:
            return
        
        # 简化的呼吸灯效果
        steps = 20
        for cycle in range(int(duration)):
            # 渐亮
            for i in range(steps):
                self.set_all_leds(True, color)
                time.sleep(0.05)
            
            # 渐暗
            for i in range(steps):
                if i == steps - 1:
                    self.set_all_leds(False, color)
                time.sleep(0.05)
    
    # ==================== 传感器读取接口 ====================
    
    def read_ultrasonic_distance(self) -> Optional[float]:
        """
        读取超声波距离
        
        Returns:
            距离值(cm)，失败返回None
        """
        if not self._is_initialized:
            return None
        
        distance = self.driver.read_ultrasonic_distance()
        if distance is not None:
            self._last_ultrasonic_reading = distance
        
        return distance
    
    def read_line_sensors(self) -> Optional[Tuple[bool, bool, bool, bool]]:
        """
        读取巡线传感器
        
        Returns:
            (左外, 左内, 右内, 右外) 传感器状态
        """
        if not self._is_initialized:
            return None
        
        sensors = self.driver.read_line_sensor()
        if sensors is not None:
            self._last_line_sensor_reading = sensors
        
        return sensors
    
    def is_line_detected(self) -> bool:
        """检测是否有线"""
        sensors = self.read_line_sensors()
        if sensors is None:
            return False
        
        return any(sensors)
    
    def get_line_position(self) -> Optional[float]:
        """
        获取线的位置
        
        Returns:
            线的位置 (-1.0 到 1.0, 0为中心, 负值偏左, 正值偏右)
        """
        sensors = self.read_line_sensors()
        if sensors is None:
            return None
        
        left_outer, left_inner, right_inner, right_outer = sensors
        
        # 简单的位置计算
        if not any(sensors):
            return None  # 没有检测到线
        
        # 计算加权位置
        position = 0.0
        weight_sum = 0.0
        
        if left_outer:
            position += -1.5
            weight_sum += 1.0
        if left_inner:
            position += -0.5
            weight_sum += 1.0
        if right_inner:
            position += 0.5
            weight_sum += 1.0
        if right_outer:
            position += 1.5
            weight_sum += 1.0
        
        return position / weight_sum if weight_sum > 0 else 0.0
    
    # ==================== 音响控制接口 ====================
    
    def beep(self, duration: float = 0.1) -> bool:
        """蜂鸣器响一声"""
        if not self._is_initialized:
            return False
        
        self.driver.ctrl_beep(True)
        time.sleep(duration)
        self.driver.ctrl_beep(False)
        return True
    
    def beep_pattern(self, pattern: str, duration: float = 0.1):
        """
        蜂鸣器模式
        
        Args:
            pattern: 模式字符串，如 "101" 表示响-停-响
            duration: 每个音符的持续时间
        """
        for char in pattern:
            if char == '1':
                self.driver.ctrl_beep(True)
            else:
                self.driver.ctrl_beep(False)
            time.sleep(duration)
        
        self.driver.ctrl_beep(False)  # 确保最后关闭
    
    # ==================== 状态查询接口 ====================
    
    def get_status(self) -> Dict[str, Any]:
        """获取硬件状态"""
        status = {
            'mode': self.mode.value,
            'initialized': self._is_initialized,
            'last_ultrasonic_distance': self._last_ultrasonic_reading,
            'last_line_sensors': self._last_line_sensor_reading,
        }
        
        if self.mode == CarMode.SIMULATION:
            status['motor_speeds'] = self.mecanum.get_current_wheel_speeds()
        
        return status
    
    def is_hardware_available(self) -> bool:
        """检查硬件是否可用"""
        return self._is_initialized and (self.mode == CarMode.HARDWARE)
    
    def is_simulation_mode(self) -> bool:
        """检查是否为仿真模式"""
        return self.mode == CarMode.SIMULATION
    
    # ==================== 仿真模式专用接口 ====================
    
    def set_simulation_ultrasonic_distance(self, distance: float):
        """设置仿真超声波距离"""
        if self.mode == CarMode.SIMULATION:
            self.driver.set_simulation_distance(distance)
    
    def set_simulation_line_sensors(self, left_outer: bool, left_inner: bool, 
                                   right_inner: bool, right_outer: bool):
        """设置仿真巡线传感器"""
        if self.mode == CarMode.SIMULATION:
            sensor_data = 0
            if left_outer:
                sensor_data |= 0b1000
            if left_inner:
                sensor_data |= 0b0100
            if right_inner:
                sensor_data |= 0b0010
            if right_outer:
                sensor_data |= 0b0001
            
            self.driver.set_simulation_line_sensor(sensor_data)
    
    # ==================== 测试接口 ====================
    
    def run_hardware_test(self):
        """运行硬件测试"""
        print("Starting hardware test...")
        
        # 测试运动
        print("Testing movement...")
        self.mecanum.test_movement_pattern()
        
        # 测试舵机
        print("Testing servos...")
        self.center_camera()
        time.sleep(1)
        
        # 测试LED
        print("Testing LEDs...")
        for color in LEDColor:
            self.set_all_leds(True, color)
            time.sleep(0.5)
        self.turn_off_all_leds()
        
        # 测试蜂鸣器
        print("Testing beeper...")
        self.beep_pattern("101", 0.2)
        
        # 测试传感器
        print("Testing sensors...")
        distance = self.read_ultrasonic_distance()
        sensors = self.read_line_sensors()
        
        print(f"Ultrasonic distance: {distance}")
        print(f"Line sensors: {sensors}")
        
        print("Hardware test completed!")
    
    def cleanup(self):
        """清理资源"""
        if self._is_initialized:
            self.stop()
            self.turn_off_all_leds()
            self.driver.ctrl_beep(False)
            self._is_initialized = False
            print("Hardware interface cleaned up")

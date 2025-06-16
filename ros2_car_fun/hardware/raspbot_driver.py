#!/usr/bin/env python3
# coding: utf-8
"""
ROS2 Car Fun - Raspbot Driver
智能小车底层驱动模块

Based on the original Raspbot_Lib.py, adapted for ROS2 integration
with simulation mode support and enhanced error handling.
"""

import time
import math
import random
from typing import Optional, List, Tuple
from enum import Enum

try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False
    print("Warning: smbus not available, running in simulation mode")


class CarMode(Enum):
    """车辆运行模式"""
    SIMULATION = "simulation"
    HARDWARE = "hardware"


class RaspbotDriver:
    """
    智能小车底层驱动类
    
    提供I2C通信、电机控制、传感器读取等基础功能
    支持硬件模式和仿真模式
    """
    
    # I2C地址和寄存器定义
    PI5CAR_I2CADDR = 0x2B
    
    # 寄存器地址
    REG_MOTOR_CTRL = 0x01
    REG_SERVO_CTRL = 0x02
    REG_LED_ALL = 0x03
    REG_LED_SINGLE = 0x04
    REG_IR_SWITCH = 0x05
    REG_BEEP_SWITCH = 0x06
    REG_ULTRASONIC_SWITCH = 0x07
    REG_LED_BRIGHTNESS_ALL = 0x08
    REG_LED_BRIGHTNESS_SINGLE = 0x09
    REG_LINE_SENSOR = 0x0A
    REG_IR_DATA = 0x0C
    REG_ULTRASONIC_L = 0x1A
    REG_ULTRASONIC_H = 0x1B
    
    def __init__(self, mode: CarMode = CarMode.HARDWARE, i2c_bus: int = 1):
        """
        初始化驱动
        
        Args:
            mode: 运行模式（硬件或仿真）
            i2c_bus: I2C总线号
        """
        self.mode = mode
        self._addr = self.PI5CAR_I2CADDR
        self._device = None
        
        # 仿真状态变量
        self._sim_motor_states = [0, 0, 0, 0]  # 4个电机状态
        self._sim_servo_angles = [90, 90]  # 2个舵机角度
        self._sim_led_states = [False] * 14  # 14个LED状态
        self._sim_led_colors = [0] * 14  # LED颜色
        self._sim_ultrasonic_distance = 100  # 模拟超声波距离(cm)
        self._sim_line_sensor = 0b0000  # 模拟巡线传感器
        
        if mode == CarMode.HARDWARE:
            self._init_hardware(i2c_bus)
        else:
            print("RaspbotDriver: Running in simulation mode")
    
    def _init_hardware(self, i2c_bus: int):
        """初始化硬件I2C设备"""
        if not SMBUS_AVAILABLE:
            print("Warning: smbus not available, falling back to simulation mode")
            self.mode = CarMode.SIMULATION
            return
            
        try:
            self._device = smbus.SMBus(i2c_bus)
            print(f"RaspbotDriver: Hardware initialized on I2C bus {i2c_bus}")
        except Exception as e:
            print(f"Warning: Failed to initialize I2C device: {e}")
            print("Falling back to simulation mode")
            self.mode = CarMode.SIMULATION
    
    def _write_u8(self, reg: int, data: int) -> bool:
        """写入单字节数据"""
        if self.mode == CarMode.SIMULATION:
            return True
            
        try:
            self._device.write_byte_data(self._addr, reg, data)
            return True
        except Exception as e:
            print(f'write_u8 I2C error: {e}')
            return False
    
    def _write_array(self, reg: int, data: List[int]) -> bool:
        """写入数组数据"""
        if self.mode == CarMode.SIMULATION:
            return True
            
        try:
            self._device.write_i2c_block_data(self._addr, reg, data)
            return True
        except Exception as e:
            print(f'write_array I2C error: {e}')
            return False
    
    def _read_data_array(self, reg: int, length: int) -> Optional[List[int]]:
        """读取数组数据"""
        if self.mode == CarMode.SIMULATION:
            # 返回模拟数据
            if reg == self.REG_LINE_SENSOR:
                return [self._sim_line_sensor]
            elif reg == self.REG_ULTRASONIC_L:
                return [self._sim_ultrasonic_distance & 0xFF]
            elif reg == self.REG_ULTRASONIC_H:
                return [(self._sim_ultrasonic_distance >> 8) & 0xFF]
            else:
                return [0] * length
                
        try:
            buf = self._device.read_i2c_block_data(self._addr, reg, length)
            return buf
        except Exception as e:
            print(f'read_data_array I2C error: {e}')
            return None
    
    def ctrl_motor(self, motor_id: int, motor_dir: int, motor_speed: int) -> bool:
        """
        控制电机
        
        Args:
            motor_id: 电机ID (0-3)
            motor_dir: 方向 (0=前进, 1=后退)
            motor_speed: 速度 (0-255)
        """
        # 参数检查
        if motor_dir not in [0, 1]:
            motor_dir = 0
        motor_speed = max(0, min(255, motor_speed))
        
        if self.mode == CarMode.SIMULATION:
            if 0 <= motor_id <= 3:
                self._sim_motor_states[motor_id] = motor_speed if motor_dir == 0 else -motor_speed
            return True
        
        data = [motor_id, motor_dir, motor_speed]
        return self._write_array(self.REG_MOTOR_CTRL, data)
    
    def ctrl_motor_speed(self, motor_id: int, motor_speed: int) -> bool:
        """
        控制电机（支持正负速度）
        
        Args:
            motor_id: 电机ID (0-3)
            motor_speed: 速度 (-255 to 255, 负数为后退)
        """
        motor_speed = max(-255, min(255, motor_speed))
        motor_dir = 1 if motor_speed < 0 else 0
        
        if self.mode == CarMode.SIMULATION:
            if 0 <= motor_id <= 3:
                self._sim_motor_states[motor_id] = motor_speed
            return True
        
        data = [motor_id, motor_dir, abs(motor_speed)]
        return self._write_array(self.REG_MOTOR_CTRL, data)
    
    def ctrl_servo(self, servo_id: int, angle: int) -> bool:
        """
        控制舵机
        
        Args:
            servo_id: 舵机ID (1-2)
            angle: 角度 (0-180)
        """
        angle = max(0, min(180, angle))
        if servo_id == 2 and angle > 110:
            angle = 110
            
        if self.mode == CarMode.SIMULATION:
            if servo_id in [1, 2]:
                self._sim_servo_angles[servo_id - 1] = angle
            return True
        
        data = [servo_id, angle]
        return self._write_array(self.REG_SERVO_CTRL, data)
    
    def ctrl_led_all(self, state: bool, color: int) -> bool:
        """
        控制所有LED
        
        Args:
            state: 开关状态
            color: 颜色 (0=红, 1=绿, 2=蓝, 3=黄, 4=紫, 5=青, 6=白)
        """
        state_val = 1 if state else 0
        color = max(0, min(6, color))
        
        if self.mode == CarMode.SIMULATION:
            for i in range(14):
                self._sim_led_states[i] = state
                if state:
                    self._sim_led_colors[i] = color
            return True
        
        data = [state_val, color]
        return self._write_array(self.REG_LED_ALL, data)
    
    def ctrl_led_single(self, led_id: int, state: bool, color: int) -> bool:
        """
        控制单个LED
        
        Args:
            led_id: LED编号 (0-13)
            state: 开关状态
            color: 颜色
        """
        if not (0 <= led_id <= 13):
            return False
            
        state_val = 1 if state else 0
        color = max(0, min(6, color))
        
        if self.mode == CarMode.SIMULATION:
            self._sim_led_states[led_id] = state
            if state:
                self._sim_led_colors[led_id] = color
            return True
        
        data = [led_id, state_val, color]
        return self._write_array(self.REG_LED_SINGLE, data)
    
    def ctrl_beep(self, state: bool) -> bool:
        """控制蜂鸣器"""
        state_val = 1 if state else 0
        
        if self.mode == CarMode.SIMULATION:
            if state:
                print("BEEP: 蜂鸣器响")
            return True
        
        data = [state_val]
        return self._write_array(self.REG_BEEP_SWITCH, data)
    
    def ctrl_ultrasonic(self, state: bool) -> bool:
        """控制超声波传感器"""
        state_val = 1 if state else 0
        
        if self.mode == CarMode.SIMULATION:
            return True
        
        data = [state_val]
        return self._write_array(self.REG_ULTRASONIC_SWITCH, data)
    
    def read_ultrasonic_distance(self) -> Optional[float]:
        """
        读取超声波距离
        
        Returns:
            距离值(cm)，失败返回None
        """
        if self.mode == CarMode.SIMULATION:
            # 模拟距离变化
            self._sim_ultrasonic_distance += random.uniform(-5, 5)
            self._sim_ultrasonic_distance = max(5, min(200, self._sim_ultrasonic_distance))
            return self._sim_ultrasonic_distance
        
        try:
            # 启动测距
            self.ctrl_ultrasonic(True)
            time.sleep(0.1)  # 等待测量完成
            
            # 读取距离数据
            dist_l = self._read_data_array(self.REG_ULTRASONIC_L, 1)
            dist_h = self._read_data_array(self.REG_ULTRASONIC_H, 1)
            
            if dist_l is None or dist_h is None:
                return None
                
            distance = (dist_h[0] << 8) | dist_l[0]
            return distance / 10.0  # 转换为cm
            
        except Exception as e:
            print(f"读取超声波距离失败: {e}")
            return None
        finally:
            self.ctrl_ultrasonic(False)
    
    def read_line_sensor(self) -> Optional[Tuple[bool, bool, bool, bool]]:
        """
        读取巡线传感器
        
        Returns:
            (左外, 左内, 右内, 右外) 传感器状态，True表示检测到线
        """
        if self.mode == CarMode.SIMULATION:
            # 模拟巡线传感器数据
            track = self._sim_line_sensor
            x1 = bool((track >> 3) & 0x01)
            x2 = bool((track >> 2) & 0x01)
            x3 = bool((track >> 1) & 0x01)
            x4 = bool(track & 0x01)
            return (x1, x2, x3, x4)
        
        try:
            data = self._read_data_array(self.REG_LINE_SENSOR, 1)
            if data is None:
                return None
                
            track = data[0]
            x1 = bool((track >> 3) & 0x01)
            x2 = bool((track >> 2) & 0x01)
            x3 = bool((track >> 1) & 0x01)
            x4 = bool(track & 0x01)
            return (x1, x2, x3, x4)
            
        except Exception as e:
            print(f"读取巡线传感器失败: {e}")
            return None
    
    def stop_all_motors(self):
        """停止所有电机"""
        for i in range(4):
            self.ctrl_motor(i, 0, 0)
    
    def get_motor_states(self) -> List[int]:
        """获取电机状态（仅仿真模式）"""
        if self.mode == CarMode.SIMULATION:
            return self._sim_motor_states.copy()
        return [0, 0, 0, 0]
    
    def set_simulation_line_sensor(self, sensor_data: int):
        """设置仿真巡线传感器数据（仅仿真模式）"""
        if self.mode == CarMode.SIMULATION:
            self._sim_line_sensor = sensor_data & 0x0F
    
    def set_simulation_distance(self, distance: float):
        """设置仿真超声波距离（仅仿真模式）"""
        if self.mode == CarMode.SIMULATION:
            self._sim_ultrasonic_distance = max(0, distance)

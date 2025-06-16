#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
ROS2 Car Fun - Mecanum Wheel Controller
麦克纳姆轮运动控制器

Based on McLumk_Wheel_Sports.py, adapted for ROS2 integration
with enhanced kinematics and control algorithms.
"""

import math
import time
from typing import Tuple, Optional
from .raspbot_driver import RaspbotDriver, CarMode


class MecanumController:
    """
    麦克纳姆轮运动控制器
    
    提供全向运动控制，包括：
    - 前进/后退
    - 左右平移
    - 原地旋转
    - 斜向运动
    - 组合运动（平移+旋转）
    """
    
    def __init__(self, driver: RaspbotDriver, debug: bool = False):
        """
        初始化控制器
        
        Args:
            driver: 底层驱动实例
            debug: 是否启用调试输出
        """
        self.driver = driver
        self.debug = debug
        
        # 电机编号定义
        self.MOTOR_L1 = 0  # 左前
        self.MOTOR_L2 = 1  # 左后  
        self.MOTOR_R1 = 2  # 右前
        self.MOTOR_R2 = 3  # 右后
        
        # 运动学参数
        self.wheel_base = 0.15  # 轴距 (m)
        self.wheel_track = 0.13  # 轮距 (m)
        self.wheel_radius = 0.03  # 轮子半径 (m)
        
        # 速度限制
        self.max_speed = 255
        self.min_speed = 0
    
    def _limit_speed(self, speed: float) -> int:
        """限制速度范围"""
        return int(max(-self.max_speed, min(self.max_speed, speed)))
    
    def _calculate_wheel_speeds(self, vx: float, vy: float, omega: float) -> Tuple[int, int, int, int]:
        """
        计算四个轮子的速度
        
        麦克纳姆轮运动学正解：
        v_l1 = vy + vx - omega * (wheel_base + wheel_track) / 2
        v_l2 = vy - vx - omega * (wheel_base + wheel_track) / 2  
        v_r1 = vy - vx + omega * (wheel_base + wheel_track) / 2
        v_r2 = vy + vx + omega * (wheel_base + wheel_track) / 2
        
        Args:
            vx: X方向速度 (向右为正)
            vy: Y方向速度 (向前为正)
            omega: 角速度 (逆时针为正)
            
        Returns:
            (l1, l2, r1, r2) 四个轮子的速度
        """
        # 计算旋转半径
        rotation_radius = (self.wheel_base + self.wheel_track) / 2
        
        # 计算各轮速度
        l1 = vy + vx - omega * rotation_radius
        l2 = vy - vx - omega * rotation_radius
        r1 = vy - vx + omega * rotation_radius
        r2 = vy + vx + omega * rotation_radius
        
        # 限制速度范围
        l1 = self._limit_speed(l1)
        l2 = self._limit_speed(l2)
        r1 = self._limit_speed(r1)
        r2 = self._limit_speed(r2)
        
        if self.debug:
            print(f"Wheel speeds - L1:{l1:>4} L2:{l2:>4} R1:{r1:>4} R2:{r2:>4}")
        
        return l1, l2, r1, r2
    
    def _set_motor_speeds(self, l1: int, l2: int, r1: int, r2: int):
        """设置电机速度"""
        self.driver.ctrl_motor_speed(self.MOTOR_L1, l1)
        self.driver.ctrl_motor_speed(self.MOTOR_L2, l2)
        self.driver.ctrl_motor_speed(self.MOTOR_R1, r1)
        self.driver.ctrl_motor_speed(self.MOTOR_R2, r2)
    
    def move_velocity(self, vx: float, vy: float, omega: float = 0.0):
        """
        按速度控制运动
        
        Args:
            vx: X方向速度 (-255 to 255)
            vy: Y方向速度 (-255 to 255)  
            omega: 角速度 (-255 to 255)
        """
        l1, l2, r1, r2 = self._calculate_wheel_speeds(vx, vy, omega)
        self._set_motor_speeds(l1, l2, r1, r2)
    
    def move_forward(self, speed: float):
        """前进"""
        if self.debug:
            print(f"Moving forward at speed {speed}")
        self.move_velocity(0, speed, 0)
    
    def move_backward(self, speed: float):
        """后退"""
        if self.debug:
            print(f"Moving backward at speed {speed}")
        self.move_velocity(0, -speed, 0)
    
    def move_left(self, speed: float):
        """左移"""
        if self.debug:
            print(f"Moving left at speed {speed}")
        self.move_velocity(-speed, 0, 0)
    
    def move_right(self, speed: float):
        """右移"""
        if self.debug:
            print(f"Moving right at speed {speed}")
        self.move_velocity(speed, 0, 0)
    
    def rotate_left(self, speed: float):
        """左转（逆时针）"""
        if self.debug:
            print(f"Rotating left at speed {speed}")
        self.move_velocity(0, 0, speed)
    
    def rotate_right(self, speed: float):
        """右转（顺时针）"""
        if self.debug:
            print(f"Rotating right at speed {speed}")
        self.move_velocity(0, 0, -speed)
    
    def move_diagonal_front_left(self, speed: float):
        """左前斜向运动"""
        if self.debug:
            print(f"Moving diagonal front-left at speed {speed}")
        self.move_velocity(-speed * 0.707, speed * 0.707, 0)
    
    def move_diagonal_front_right(self, speed: float):
        """右前斜向运动"""
        if self.debug:
            print(f"Moving diagonal front-right at speed {speed}")
        self.move_velocity(speed * 0.707, speed * 0.707, 0)
    
    def move_diagonal_back_left(self, speed: float):
        """左后斜向运动"""
        if self.debug:
            print(f"Moving diagonal back-left at speed {speed}")
        self.move_velocity(-speed * 0.707, -speed * 0.707, 0)
    
    def move_diagonal_back_right(self, speed: float):
        """右后斜向运动"""
        if self.debug:
            print(f"Moving diagonal back-right at speed {speed}")
        self.move_velocity(speed * 0.707, -speed * 0.707, 0)
    
    def move_direction(self, speed: float, direction_deg: float):
        """
        按指定方向运动
        
        Args:
            speed: 运动速度
            direction_deg: 方向角度（度，0度为前进，90度为左移）
        """
        direction_rad = math.radians(direction_deg)
        vx = speed * math.cos(direction_rad)
        vy = speed * math.sin(direction_rad)
        
        if self.debug:
            print(f"Moving in direction {direction_deg}° at speed {speed}")
        
        self.move_velocity(vx, vy, 0)
    
    def move_with_rotation(self, vx: float, vy: float, omega: float):
        """
        组合运动（平移+旋转）
        
        Args:
            vx: X方向速度
            vy: Y方向速度
            omega: 角速度
        """
        if self.debug:
            print(f"Combined motion - vx:{vx} vy:{vy} omega:{omega}")
        
        self.move_velocity(vx, vy, omega)
    
    def stop(self):
        """停止运动"""
        if self.debug:
            print("Stopping all motors")
        
        self.driver.stop_all_motors()
    
    def emergency_stop(self):
        """紧急停止"""
        print("EMERGENCY STOP!")
        self.stop()
    
    def move_for_duration(self, vx: float, vy: float, omega: float, duration: float):
        """
        运动指定时间后停止
        
        Args:
            vx: X方向速度
            vy: Y方向速度  
            omega: 角速度
            duration: 运动时间（秒）
        """
        if self.debug:
            print(f"Moving for {duration}s - vx:{vx} vy:{vy} omega:{omega}")
        
        self.move_velocity(vx, vy, omega)
        time.sleep(duration)
        self.stop()
    
    def test_movement_pattern(self):
        """测试运动模式"""
        if self.driver.mode == CarMode.SIMULATION:
            print("Testing movement patterns in simulation mode...")
        else:
            print("Testing movement patterns on hardware...")
        
        test_speed = 100
        test_duration = 1.0
        
        movements = [
            ("Forward", lambda: self.move_forward(test_speed)),
            ("Backward", lambda: self.move_backward(test_speed)),
            ("Left", lambda: self.move_left(test_speed)),
            ("Right", lambda: self.move_right(test_speed)),
            ("Rotate Left", lambda: self.rotate_left(test_speed)),
            ("Rotate Right", lambda: self.rotate_right(test_speed)),
            ("Diagonal Front-Left", lambda: self.move_diagonal_front_left(test_speed)),
            ("Diagonal Front-Right", lambda: self.move_diagonal_front_right(test_speed)),
        ]
        
        for name, movement in movements:
            print(f"Testing: {name}")
            movement()
            time.sleep(test_duration)
            self.stop()
            time.sleep(0.5)
        
        print("Movement pattern test completed!")
    
    def get_current_wheel_speeds(self) -> Tuple[int, int, int, int]:
        """获取当前轮子速度（仅仿真模式）"""
        if self.driver.mode == CarMode.SIMULATION:
            states = self.driver.get_motor_states()
            return states[0], states[1], states[2], states[3]
        return 0, 0, 0, 0
    
    def set_debug(self, debug: bool):
        """设置调试模式"""
        self.debug = debug
    
    def set_speed_limits(self, max_speed: int, min_speed: int = 0):
        """设置速度限制"""
        self.max_speed = max(1, min(255, max_speed))
        self.min_speed = max(0, min(self.max_speed - 1, min_speed))

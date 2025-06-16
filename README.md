# ROS2 Car Fun - 车辆控制与仿真项目

这是一个基于ROS2的车辆控制和仿真项目，使用Python3开发，展示了ROS2的基本概念和最佳实践。

## 项目特性

- 🚗 **车辆控制器**: 实现差分驱动车辆的运动控制
- 🎮 **键盘遥控**: 通过键盘实时控制车辆运动
- 🌍 **环境模拟**: 模拟车辆在室内环境中的行为
- 📡 **传感器仿真**: 激光雷达和IMU传感器数据模拟
- 🗺️ **地图发布**: 静态环境地图生成和发布
- 📊 **实时数据**: 里程计、关节状态等实时数据发布

## 项目结构

```
ros2_car_fun/
├── package.xml              # ROS2包配置文件
├── setup.py                 # Python包安装配置
├── setup.cfg                # 安装选项配置
├── README.md                # 项目说明文档
├── resource/
│   └── ros2_car_fun         # 包资源标识文件
├── ros2_car_fun/            # Python源代码目录
│   ├── __init__.py          # Python包初始化文件
│   ├── car_controller.py    # 车辆控制器节点
│   ├── car_simulator.py     # 车辆模拟器节点
│   └── car_teleop.py        # 键盘遥控节点
├── launch/
│   └── car_demo.launch.py   # 启动文件
└── config/
    └── car_params.yaml      # 参数配置文件
```

## 节点说明

### 1. 车辆控制器 (car_controller)

**功能**: 接收速度命令，控制车辆运动，发布里程计和关节状态信息

**订阅话题**:
- `/cmd_vel` (geometry_msgs/Twist): 速度命令

**发布话题**:
- `/odom` (nav_msgs/Odometry): 里程计信息
- `/joint_states` (sensor_msgs/JointState): 关节状态

**参数**:
- `max_linear_speed`: 最大线速度 (默认: 2.0 m/s)
- `max_angular_speed`: 最大角速度 (默认: 1.0 rad/s)
- `wheel_base`: 轴距 (默认: 0.3 m)
- `wheel_radius`: 轮子半径 (默认: 0.05 m)

### 2. 车辆模拟器 (car_simulator)

**功能**: 模拟车辆在环境中的行为，生成传感器数据

**订阅话题**:
- `/cmd_vel` (geometry_msgs/Twist): 速度命令

**发布话题**:
- `/scan` (sensor_msgs/LaserScan): 激光雷达数据
- `/imu` (sensor_msgs/Imu): IMU数据
- `/pose` (geometry_msgs/PoseStamped): 位姿信息
- `/map` (nav_msgs/OccupancyGrid): 环境地图

**参数**:
- `simulation_frequency`: 模拟频率 (默认: 50.0 Hz)
- `laser_range_max`: 激光雷达最大距离 (默认: 10.0 m)
- `noise_level`: 噪声水平 (默认: 0.1)

### 3. 键盘遥控 (car_teleop)

**功能**: 通过键盘控制车辆运动

**发布话题**:
- `/cmd_vel` (geometry_msgs/Twist): 速度命令

**控制键位**:
- `w`: 前进
- `s`: 后退
- `a`: 左转
- `d`: 右转
- `q`: 前进+左转
- `e`: 前进+右转
- `z`: 后退+左转
- `c`: 后退+右转
- `空格`: 停止
- `Ctrl+C`: 退出

## 安装和使用

### 前置条件

- ROS2 (推荐 Humble 或更新版本)
- Python 3.8+
- 必要的ROS2 Python包:
  ```bash
  sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-geometry-msgs \
                   ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-tf2-ros \
                   ros-humble-robot-state-publisher
  ```

### 编译安装

1. **创建工作空间** (如果还没有):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **复制项目到工作空间**:
   ```bash
   # 如果项目已在其他位置，复制到src目录
   cp -r /path/to/ros2_car_fun ~/ros2_ws/src/
   ```

3. **编译项目**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_car_fun
   ```

4. **设置环境**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### 运行示例

#### 方法1: 使用Launch文件 (推荐)

启动完整的演示系统:
```bash
ros2 launch ros2_car_fun car_demo.launch.py
```

只启动控制器和遥控 (不启动模拟器):
```bash
ros2 launch ros2_car_fun car_demo.launch.py use_simulator:=false
```

只启动控制器和模拟器 (不启动遥控):
```bash
ros2 launch ros2_car_fun car_demo.launch.py use_teleop:=false
```

#### 方法2: 单独启动节点

在不同终端中分别运行:

```bash
# 终端1: 启动车辆控制器
ros2 run ros2_car_fun car_controller

# 终端2: 启动车辆模拟器
ros2 run ros2_car_fun car_simulator

# 终端3: 启动键盘遥控
ros2 run ros2_car_fun car_teleop
```

### 可视化

使用RViz2查看仿真效果:
```bash
rviz2
```

在RViz2中添加以下显示项:
- **Map**: 话题 `/map`
- **LaserScan**: 话题 `/scan`
- **Odometry**: 话题 `/odom`
- **RobotModel**: 显示机器人模型
- **TF**: 显示坐标变换

## 话题和服务

### 主要话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/cmd_vel` | geometry_msgs/Twist | 速度命令 |
| `/odom` | nav_msgs/Odometry | 里程计信息 |
| `/scan` | sensor_msgs/LaserScan | 激光雷达数据 |
| `/imu` | sensor_msgs/Imu | IMU数据 |
| `/map` | nav_msgs/OccupancyGrid | 环境地图 |
| `/joint_states` | sensor_msgs/JointState | 关节状态 |
| `/pose` | geometry_msgs/PoseStamped | 位姿信息 |

### 坐标系

- `map`: 世界坐标系
- `odom`: 里程计坐标系
- `base_link`: 机器人本体坐标系
- `laser_frame`: 激光雷达坐标系
- `imu_frame`: IMU坐标系

## 参数配置

项目参数可以通过以下方式配置:

1. **配置文件**: 编辑 `config/car_params.yaml`
2. **启动参数**: 在launch文件中修改参数
3. **命令行**: 使用 `--ros-args -p` 参数

示例:
```bash
ros2 run ros2_car_fun car_controller --ros-args -p max_linear_speed:=1.5
```

## 开发和扩展

### 添加新节点

1. 在 `ros2_car_fun/` 目录下创建新的Python文件
2. 在 `setup.py` 中添加新的入口点
3. 重新编译项目

### 添加新的传感器

1. 在模拟器节点中添加传感器模拟逻辑
2. 创建相应的发布器
3. 在launch文件中添加必要的变换

### 自定义环境

修改 `car_simulator.py` 中的环境参数:
- `room_width`, `room_height`: 房间尺寸
- `obstacles`: 障碍物列表

## 故障排除

### 常见问题

1. **节点无法启动**:
   - 检查ROS2环境是否正确设置
   - 确认所有依赖包已安装
   - 检查Python路径和权限

2. **键盘控制无响应**:
   - 确保终端有焦点
   - 检查终端权限设置
   - 尝试在不同终端中运行

3. **可视化问题**:
   - 检查话题是否正确发布: `ros2 topic list`
   - 确认坐标变换正常: `ros2 run tf2_tools view_frames`
   - 检查RViz2配置

### 调试命令

```bash
# 查看活动节点
ros2 node list

# 查看话题列表
ros2 topic list

# 查看话题数据
ros2 topic echo /cmd_vel

# 查看节点信息
ros2 node info /car_controller

# 查看参数
ros2 param list /car_controller
```

## 许可证

Apache License 2.0

## 贡献

欢迎提交Issue和Pull Request来改进这个项目！

## 联系方式

如有问题或建议，请通过以下方式联系:
- Email: developer@example.com
- GitHub Issues: [项目Issues页面]
# ROS2 Car Fun - 智能小车控制系统

一个完整的基于ROS2的智能小车控制系统，集成了硬件抽象层、传感器处理和视觉功能。

## ✨ 特性

- 🚗 **麦克纳姆轮支持**: 支持全向运动控制
- 🔧 **硬件抽象层**: 统一的硬件接口，支持仿真和实际硬件
- 📡 **传感器集成**: 超声波、巡线传感器、红外遥控
- 👁️ **视觉功能**: 摄像头、颜色检测、人脸跟踪、二维码识别
- 🎮 **多种控制方式**: 红外遥控、程序控制
- 📊 **完整测试**: 单元测试、集成测试、系统测试
- ⚙️ **参数化配置**: 灵活的配置系统
- 🔧 **模块化设计**: 易于扩展和定制

## 📁 项目结构

```
ros2_car_fun/
├── package.xml                    # ROS2包配置文件
├── setup.py                       # Python包安装配置
├── setup.cfg                      # 安装选项配置
├── README.md                      # 项目说明文档
├── .gitignore                     # Git忽略文件
├── resource/
│   └── ros2_car_fun               # 包资源标识文件
├── ros2_car_fun/                  # Python源代码目录
│   ├── __init__.py                # Python包初始化文件
│   ├── car_controller.py          # 车辆控制器节点
│   ├── hardware/                  # 硬件抽象层
│   │   ├── __init__.py
│   │   ├── hardware_interface.py  # 硬件接口
│   │   ├── mecanum_controller.py  # 麦克纳姆轮控制器
│   │   └── raspbot_driver.py      # 智能小车驱动
│   ├── sensors/                   # 传感器节点
│   │   ├── __init__.py
│   │   ├── ultrasonic_node.py     # 超声波传感器
│   │   ├── line_sensor_node.py    # 巡线传感器
│   │   └── ir_receiver_node.py    # 红外接收器
│   └── vision/                    # 视觉处理节点
│       ├── __init__.py
│       ├── camera_node.py         # 摄像头节点
│       ├── color_detection.py     # 颜色检测
│       ├── face_tracking.py       # 人脸跟踪
│       ├── qr_detection.py        # 二维码检测
│       └── object_detection.py    # 物体检测
├── launch/                        # 启动文件
│   ├── car_demo.launch.py         # 基础演示启动
│   ├── hardware_test.launch.py    # 硬件测试启动
│   ├── sensors.launch.py          # 传感器启动
│   └── vision.launch.py           # 视觉功能启动
├── config/                        # 配置文件
│   ├── car_params.yaml            # 基础参数
│   ├── hardware_config.yaml       # 硬件配置
│   ├── sensor_config.yaml         # 传感器配置
│   └── vision_config.yaml         # 视觉配置
└── test_*.py                      # 测试脚本
```

## 🚀 核心组件

### 1. 硬件抽象层
- `HardwareInterface`: 主要硬件接口
- `MecanumController`: 麦克纳姆轮控制器
- `RaspbotDriver`: 智能小车底层驱动

### 2. 传感器系统
- **超声波传感器**: 距离测量和避障
- **巡线传感器**: 4路红外传感器，线位置计算
- **红外遥控**: 遥控信号接收和命令解析

### 3. 视觉系统
- **摄像头节点**: 图像采集和云台控制
- **颜色检测**: HSV颜色空间检测和跟踪
- **人脸跟踪**: Haar级联分类器检测
- **二维码检测**: OpenCV + pyzbar双重检测
- **物体检测**: 基于轮廓的物体检测

### 4. 车辆控制器
- 接收速度命令，控制车辆运动
- 发布里程计和状态信息
- 集成硬件抽象层

## 🛠️ 安装和使用

### 前置条件
- ROS2 (推荐 Humble 或更新版本)
- Python 3.8+
- OpenCV, NumPy等依赖包

### 编译安装
```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 复制项目
cp -r /path/to/ros2_car_fun .

# 编译
cd ~/ros2_ws
colcon build --packages-select ros2_car_fun

# 设置环境
source ~/ros2_ws/install/setup.bash
```

### 运行示例
```bash
# 基础演示
ros2 launch ros2_car_fun car_demo.launch.py

# 硬件测试
ros2 launch ros2_car_fun hardware_test.launch.py

# 传感器功能
ros2 launch ros2_car_fun sensors.launch.py

# 视觉功能
ros2 launch ros2_car_fun vision.launch.py
```

## 🧪 测试

```bash
# 硬件测试
python3 test_hardware.py

# 传感器测试
python3 test_sensors.py

# 视觉测试
python3 test_vision.py

# 集成测试
python3 test_integration.py

# 完整系统测试
python3 test_full_system.py
```

## 📊 主要话题

| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/cmd_vel` | geometry_msgs/Twist | 速度命令 |
| `/odom` | nav_msgs/Odometry | 里程计信息 |
| `/ultrasonic/range` | sensor_msgs/Range | 超声波距离 |
| `/line_sensors/raw` | std_msgs/Bool[4] | 巡线传感器数据 |
| `/camera/image_raw` | sensor_msgs/Image | 摄像头图像 |
| `/color_detection/detected` | std_msgs/Bool | 颜色检测结果 |
| `/face_tracking/detected` | std_msgs/Bool | 人脸检测结果 |
| `/qr_detection/content` | std_msgs/String | 二维码内容 |

## ⚙️ 配置

项目参数可以通过配置文件进行调整：
- `config/hardware_config.yaml`: 硬件参数
- `config/sensor_config.yaml`: 传感器参数  
- `config/vision_config.yaml`: 视觉参数

## 🔧 开发和扩展

### 添加新传感器
1. 在 `sensors/` 目录下创建新的传感器节点
2. 在 `setup.py` 中添加入口点
3. 创建相应的配置文件

### 添加新的视觉功能
1. 在 `vision/` 目录下创建新的视觉节点
2. 继承基础视觉节点类
3. 实现特定的图像处理算法

## 📝 许可证

Apache License 2.0

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个项目！

## 📞 联系方式

如有问题或建议，请通过GitHub Issues联系。

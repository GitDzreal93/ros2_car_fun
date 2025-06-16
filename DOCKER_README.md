# ROS2 Car Fun - Docker 部署指南

## 概述

本项目提供了完整的 Docker 容器化解决方案，专门适配树莓派 Debian 12 (bookworm) 系统。支持硬件模式和仿真模式，可以轻松部署到树莓派或其他 ARM 设备上。

## 系统要求

### 硬件要求
- 树莓派 4B 或更高版本 (推荐 8GB 内存)
- microSD 卡 (32GB 或更大)
- 智能小车硬件套件 (可选，仅硬件模式需要)

### 软件要求
- Debian GNU/Linux 12 (bookworm)
- Docker 20.10+
- Docker Compose 2.0+

## 快速开始

### 1. 安装 Docker

```bash
# 安装 Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# 将用户添加到 docker 组
sudo usermod -aG docker $USER

# 重新登录或重启以使组权限生效
sudo reboot
```

### 2. 克隆项目

```bash
git clone <your-repo-url>
cd ros2_car_fun
```

### 3. 构建镜像

#### 本地构建 (推荐用于开发)
```bash
./build_local.sh
```

#### 构建并推送到 Docker Hub
```bash
./build_and_deploy.sh -u your-dockerhub-username
```

### 4. 运行容器

#### 硬件模式 (连接实际硬件)
```bash
docker-compose up ros2-car
```

#### 仿真模式 (无需硬件)
```bash
docker-compose --profile simulation up ros2-car-sim
```

#### 硬件测试
```bash
docker-compose --profile test up hardware-test
```

## 配置说明

### 环境变量

| 变量名 | 默认值 | 说明 |
|--------|--------|------|
| `ROS_DOMAIN_ID` | 0 | ROS2 域ID |
| `RMW_IMPLEMENTATION` | rmw_cyclonedx_cpp | ROS2 中间件实现 |
| `HARDWARE_MODE` | hardware | 硬件模式 (hardware/simulation) |

### 设备挂载

容器需要访问以下设备：
- `/dev/i2c-1` - I2C 通信
- `/dev/video0` - 摄像头
- `/dev/gpiomem` - GPIO 访问

### 权限设置

确保用户在相关组中：
```bash
sudo usermod -aG i2c,gpio,video $USER
```

## 使用示例

### 启动完整系统
```bash
# 后台运行
docker-compose up -d ros2-car

# 查看日志
docker-compose logs -f ros2-car

# 停止服务
docker-compose down
```

### 交互式运行
```bash
docker run -it --rm \
  --privileged \
  --network host \
  --device /dev/i2c-1:/dev/i2c-1 \
  --device /dev/video0:/dev/video0 \
  -v /dev:/dev \
  -v /sys:/sys \
  -e ROS_DOMAIN_ID=0 \
  ros2-car-fun:latest \
  bash
```

### 运行特定节点
```bash
# 启动车辆控制器
docker-compose exec ros2-car ros2 run ros2_car_fun car_controller

# 启动传感器节点
docker-compose exec ros2-car ros2 run ros2_car_fun ultrasonic_node

# 查看话题
docker-compose exec ros2-car ros2 topic list
```

## 故障排除

### 常见问题

#### 1. I2C 权限错误
```bash
# 检查 I2C 设备
ls -l /dev/i2c-*

# 设置权限
sudo chmod 666 /dev/i2c-1
```

#### 2. 摄像头无法访问
```bash
# 检查摄像头设备
ls -l /dev/video*

# 设置权限
sudo chmod 666 /dev/video0
```

#### 3. GPIO 权限问题
```bash
# 检查 GPIO 组
groups $USER

# 添加到 gpio 组
sudo usermod -aG gpio $USER
```

#### 4. 容器无法启动
```bash
# 查看详细日志
docker-compose logs ros2-car

# 检查容器状态
docker-compose ps

# 重新构建镜像
docker-compose build --no-cache
```

### 调试命令

```bash
# 进入运行中的容器
docker-compose exec ros2-car bash

# 检查 ROS2 节点
docker-compose exec ros2-car ros2 node list

# 检查话题
docker-compose exec ros2-car ros2 topic list

# 监听话题数据
docker-compose exec ros2-car ros2 topic echo /cmd_vel

# 检查硬件连接
docker-compose exec ros2-car i2cdetect -y 1
```

## 开发指南

### 修改配置
配置文件位于 `config/` 目录下，可以通过卷挂载进行修改：
```bash
# 编辑配置文件
nano config/car_params.yaml

# 重启容器以应用更改
docker-compose restart ros2-car
```

### 添加新功能
1. 修改源代码
2. 重新构建镜像：`./build_local.sh`
3. 重启容器：`docker-compose up -d ros2-car`

### 查看性能
```bash
# 查看容器资源使用
docker stats ros2-car-system

# 查看系统资源
htop
```

## 部署到生产环境

### 1. 使用预构建镜像
```bash
# 从 Docker Hub 拉取
docker pull your-username/ros2-car-fun:latest

# 更新 docker-compose.yml 中的镜像名
# image: your-username/ros2-car-fun:latest
```

### 2. 自动启动
```bash
# 设置开机自启
sudo systemctl enable docker

# 创建 systemd 服务
sudo tee /etc/systemd/system/ros2-car.service > /dev/null <<EOF
[Unit]
Description=ROS2 Car Fun Service
Requires=docker.service
After=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/home/pi/ros2_car_fun
ExecStart=/usr/local/bin/docker-compose up -d ros2-car
ExecStop=/usr/local/bin/docker-compose down
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF

# 启用服务
sudo systemctl enable ros2-car.service
sudo systemctl start ros2-car.service
```

## 更多资源

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [Docker 官方文档](https://docs.docker.com/)
- [树莓派官方文档](https://www.raspberrypi.org/documentation/)

## 支持

如有问题，请查看：
1. 项目 Issues
2. ROS2 社区论坛
3. Docker 社区论坛

#!/bin/bash
# ROS2 Car Fun - 本地构建脚本
# 用于在树莓派上本地构建镜像

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_message() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查 Docker
if ! command -v docker &> /dev/null; then
    print_error "Docker 未安装，请先安装 Docker"
    exit 1
fi

print_message "开始本地构建 ROS2 Car Fun 镜像..."

# 构建镜像
docker build -t ros2-car-fun:latest .

if [ $? -eq 0 ]; then
    print_message "镜像构建成功!"
    print_message "使用以下命令运行:"
    echo ""
    echo "# 硬件模式:"
    echo "docker-compose up ros2-car"
    echo ""
    echo "# 仿真模式:"
    echo "docker-compose --profile simulation up ros2-car-sim"
    echo ""
    echo "# 硬件测试:"
    echo "docker-compose --profile test up hardware-test"
else
    print_error "镜像构建失败!"
    exit 1
fi

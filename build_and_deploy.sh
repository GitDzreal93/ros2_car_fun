#!/bin/bash
# ROS2 Car Fun - 构建和部署脚本
# 适配树莓派 Debian 12 (bookworm)

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置变量
DOCKER_HUB_USERNAME=""
IMAGE_NAME="ros2-car-fun"
IMAGE_TAG="latest"
PLATFORMS="linux/arm64,linux/arm/v7"

# 函数：打印彩色消息
print_message() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# 函数：检查 Docker 是否安装
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装，请先安装 Docker"
        exit 1
    fi
    print_message "Docker 已安装: $(docker --version)"
}

# 函数：检查 Docker Buildx
check_buildx() {
    if ! docker buildx version &> /dev/null; then
        print_error "Docker Buildx 未安装，请升级 Docker 到最新版本"
        exit 1
    fi
    print_message "Docker Buildx 已安装: $(docker buildx version)"
}

# 函数：设置 Docker Hub 用户名
setup_docker_hub() {
    if [ -z "$DOCKER_HUB_USERNAME" ]; then
        read -p "请输入您的 Docker Hub 用户名: " DOCKER_HUB_USERNAME
        if [ -z "$DOCKER_HUB_USERNAME" ]; then
            print_error "Docker Hub 用户名不能为空"
            exit 1
        fi
    fi
    print_message "Docker Hub 用户名: $DOCKER_HUB_USERNAME"
}

# 函数：登录 Docker Hub
docker_login() {
    print_step "登录 Docker Hub..."
    if ! docker login; then
        print_error "Docker Hub 登录失败"
        exit 1
    fi
    print_message "Docker Hub 登录成功"
}

# 函数：创建 buildx builder
setup_builder() {
    print_step "设置 Docker Buildx builder..."
    
    # 检查是否已存在 builder
    if docker buildx ls | grep -q "ros2-car-builder"; then
        print_message "使用现有的 ros2-car-builder"
        docker buildx use ros2-car-builder
    else
        print_message "创建新的 ros2-car-builder"
        docker buildx create --name ros2-car-builder --use
    fi
    
    # 启动 builder
    docker buildx inspect --bootstrap
}

# 函数：构建多架构镜像
build_image() {
    print_step "构建多架构 Docker 镜像..."
    
    FULL_IMAGE_NAME="$DOCKER_HUB_USERNAME/$IMAGE_NAME:$IMAGE_TAG"
    
    print_message "构建镜像: $FULL_IMAGE_NAME"
    print_message "支持平台: $PLATFORMS"
    
    docker buildx build \
        --platform $PLATFORMS \
        --tag $FULL_IMAGE_NAME \
        --tag "$DOCKER_HUB_USERNAME/$IMAGE_NAME:v1.0.0" \
        --push \
        .
    
    print_message "镜像构建并推送成功!"
}

# 函数：生成部署文档
generate_deploy_docs() {
    print_step "生成部署文档..."
    
    cat > DEPLOYMENT.md << EOF
# ROS2 Car Fun - 部署指南

## 系统要求
- 树莓派 4B 或更高版本
- Debian GNU/Linux 12 (bookworm)
- Docker 20.10+ 
- Docker Compose 2.0+

## 快速部署

### 1. 安装 Docker (如果未安装)
\`\`\`bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker \$USER
\`\`\`

### 2. 拉取镜像
\`\`\`bash
docker pull $DOCKER_HUB_USERNAME/$IMAGE_NAME:$IMAGE_TAG
\`\`\`

### 3. 运行容器

#### 硬件模式 (推荐)
\`\`\`bash
docker-compose up ros2-car
\`\`\`

#### 仿真模式
\`\`\`bash
docker-compose --profile simulation up ros2-car-sim
\`\`\`

#### 硬件测试
\`\`\`bash
docker-compose --profile test up hardware-test
\`\`\`

### 4. 手动运行
\`\`\`bash
docker run -it --rm \\
  --privileged \\
  --network host \\
  --device /dev/i2c-1:/dev/i2c-1 \\
  --device /dev/video0:/dev/video0 \\
  -v /dev:/dev \\
  -v /sys:/sys \\
  -e ROS_DOMAIN_ID=0 \\
  $DOCKER_HUB_USERNAME/$IMAGE_NAME:$IMAGE_TAG \\
  ros2 launch ros2_car_fun car_system.launch.py
\`\`\`

## 配置说明

### 环境变量
- \`ROS_DOMAIN_ID\`: ROS2 域ID (默认: 0)
- \`HARDWARE_MODE\`: 硬件模式 (hardware/simulation)
- \`RMW_IMPLEMENTATION\`: ROS2 中间件实现

### 设备权限
确保当前用户在以下组中:
\`\`\`bash
sudo usermod -aG i2c,gpio,video \$USER
\`\`\`

## 故障排除

### I2C 权限问题
\`\`\`bash
sudo chmod 666 /dev/i2c-1
\`\`\`

### 摄像头权限问题
\`\`\`bash
sudo chmod 666 /dev/video0
\`\`\`

### 查看日志
\`\`\`bash
docker-compose logs -f ros2-car
\`\`\`

## 更多信息
- 镜像地址: https://hub.docker.com/r/$DOCKER_HUB_USERNAME/$IMAGE_NAME
- 项目地址: https://github.com/your-username/ros2_car_fun
EOF

    print_message "部署文档已生成: DEPLOYMENT.md"
}

# 函数：显示使用说明
show_usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -u, --username USERNAME    设置 Docker Hub 用户名"
    echo "  -t, --tag TAG             设置镜像标签 (默认: latest)"
    echo "  -h, --help                显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 -u myusername -t v1.0.0"
    echo "  $0 --username myusername"
}

# 主函数
main() {
    print_message "ROS2 Car Fun - 构建和部署脚本"
    print_message "适配树莓派 Debian 12 (bookworm)"
    echo ""
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -u|--username)
                DOCKER_HUB_USERNAME="$2"
                shift 2
                ;;
            -t|--tag)
                IMAGE_TAG="$2"
                shift 2
                ;;
            -h|--help)
                show_usage
                exit 0
                ;;
            *)
                print_error "未知选项: $1"
                show_usage
                exit 1
                ;;
        esac
    done
    
    # 执行构建流程
    check_docker
    check_buildx
    setup_docker_hub
    docker_login
    setup_builder
    build_image
    generate_deploy_docs
    
    print_message ""
    print_message "🎉 构建和部署完成!"
    print_message "镜像地址: $DOCKER_HUB_USERNAME/$IMAGE_NAME:$IMAGE_TAG"
    print_message "请查看 DEPLOYMENT.md 了解部署说明"
}

# 运行主函数
main "$@"

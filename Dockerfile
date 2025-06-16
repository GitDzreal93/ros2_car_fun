# ROS2 Car Fun - Raspberry Pi Dockerfile
# 适配树莓派 Debian 12 (bookworm) 系统
# 支持 ARM64 和 ARM32 架构

FROM ros:humble-ros-base-jammy

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONUNBUFFERED=1

# 设置工作目录
WORKDIR /ros2_ws

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    # 基础工具
    curl \
    wget \
    git \
    vim \
    nano \
    htop \
    # Python 相关
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    # ROS2 相关
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # 硬件接口
    python3-smbus \
    i2c-tools \
    # 计算机视觉
    python3-opencv \
    libopencv-dev \
    # 数值计算
    python3-numpy \
    python3-scipy \
    # 网络工具
    net-tools \
    iputils-ping \
    # 清理缓存
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖
RUN pip3 install --no-cache-dir \
    smbus2 \
    opencv-python \
    numpy \
    scipy \
    matplotlib \
    Pillow

# 创建 ROS2 工作空间
RUN mkdir -p /ros2_ws/src

# 复制项目文件到容器
COPY . /ros2_ws/src/ros2_car_fun/

# 设置正确的权限
RUN chmod +x /ros2_ws/src/ros2_car_fun/launch/*.py

# 初始化 rosdep
RUN rosdep init || true
RUN rosdep update

# 安装 ROS2 依赖
RUN cd /ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y

# 构建 ROS2 包
RUN cd /ros2_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select ros2_car_fun

# 设置环境变量脚本
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> /root/.bashrc

# 创建启动脚本
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2 environment\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
\n\
# Set ROS2 environment variables\n\
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}\n\
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedx_cpp}\n\
\n\
# Execute command\n\
exec "$@"' > /ros2_entrypoint.sh && \
    chmod +x /ros2_entrypoint.sh

# 创建硬件权限设置脚本
RUN echo '#!/bin/bash\n\
# 设置 I2C 设备权限\n\
if [ -e /dev/i2c-1 ]; then\n\
    chmod 666 /dev/i2c-1\n\
    echo "I2C device permissions set"\n\
fi\n\
\n\
# 设置 GPIO 权限\n\
if [ -d /sys/class/gpio ]; then\n\
    chmod -R 666 /sys/class/gpio/ 2>/dev/null || true\n\
    echo "GPIO permissions set"\n\
fi\n\
\n\
# 设置摄像头权限\n\
if [ -e /dev/video0 ]; then\n\
    chmod 666 /dev/video0\n\
    echo "Camera permissions set"\n\
fi' > /setup_hardware.sh && \
    chmod +x /setup_hardware.sh

# 暴露端口 (ROS2 DDS 通信)
EXPOSE 7400-7500/udp
EXPOSE 11811/udp

# 设置卷挂载点
VOLUME ["/dev", "/sys"]

# 设置入口点
ENTRYPOINT ["/ros2_entrypoint.sh"]

# 默认命令
CMD ["bash"]

# 添加标签
LABEL maintainer="ROS2 Car Fun Developer"
LABEL description="ROS2 智能小车控制系统 - 适配树莓派 Debian 12"
LABEL version="1.0.0"
LABEL architecture="arm64,armhf"

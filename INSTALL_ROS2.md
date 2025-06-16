# macOS 上安装 ROS2 指南

由于您的系统上尚未安装 ROS2，以下是在 macOS 上安装 ROS2 的详细指南。

## 方法1: 使用 Homebrew 安装 (推荐)

### 1. 安装 Homebrew (如果尚未安装)
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### 2. 添加 ROS2 tap
```bash
brew tap ros/deps
brew tap osrf/simulation
```

### 3. 安装 ROS2 Humble
```bash
brew install ros-humble-desktop
```

### 4. 安装 colcon 构建工具
```bash
brew install python@3.11
pip3 install colcon-common-extensions
```

### 5. 设置环境变量
将以下内容添加到您的 shell 配置文件 (`~/.zshrc` 或 `~/.bash_profile`):
```bash
# ROS2 环境设置
source /opt/homebrew/share/ros2-humble/setup.zsh  # 对于 zsh
# 或者
# source /opt/homebrew/share/ros2-humble/setup.bash  # 对于 bash

# 添加 colcon 到 PATH
export PATH="$PATH:$(python3 -m site --user-base)/bin"
```

### 6. 重新加载 shell 配置
```bash
source ~/.zshrc  # 或 source ~/.bash_profile
```

## 方法2: 使用 Docker (替代方案)

如果 Homebrew 安装遇到问题，可以使用 Docker:

### 1. 安装 Docker Desktop for Mac
从 [Docker 官网](https://www.docker.com/products/docker-desktop) 下载并安装。

### 2. 拉取 ROS2 镜像
```bash
docker pull osrf/ros:humble-desktop
```

### 3. 创建 Docker 容器运行脚本
创建文件 `run_ros2_docker.sh`:
```bash
#!/bin/bash
docker run -it --rm \
  -v $(pwd):/workspace \
  -w /workspace \
  osrf/ros:humble-desktop \
  bash
```

### 4. 使用 Docker 运行项目
```bash
chmod +x run_ros2_docker.sh
./run_ros2_docker.sh
```

## 验证安装

安装完成后，验证 ROS2 是否正确安装:

```bash
# 检查 ROS2 命令
ros2 --help

# 检查 colcon 命令
colcon --help

# 运行简单的 ROS2 示例
ros2 run demo_nodes_cpp talker
```

## 编译和运行项目

安装 ROS2 后，您可以编译和运行此项目:

### 1. 创建工作空间
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. 复制项目
```bash
cp -r /Volumes/dz/code/ros2_car_fun ~/ros2_ws/src/
```

### 3. 安装依赖
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 编译项目
```bash
colcon build --packages-select ros2_car_fun
```

### 5. 设置环境
```bash
source install/setup.bash
```

### 6. 运行项目
```bash
# 启动完整演示
ros2 launch ros2_car_fun car_demo.launch.py

# 或单独运行节点
ros2 run ros2_car_fun car_controller
```

## 故障排除

### 常见问题

1. **权限问题**:
   ```bash
   sudo chown -R $(whoami) /opt/homebrew
   ```

2. **Python 路径问题**:
   ```bash
   export PYTHONPATH="$PYTHONPATH:$(python3 -c 'import site; print(site.getsitepackages()[0])')"
   ```

3. **colcon 找不到**:
   ```bash
   pip3 install --user colcon-common-extensions
   export PATH="$PATH:$(python3 -m site --user-base)/bin"
   ```

### 检查安装状态

```bash
# 检查 ROS2 版本
ros2 doctor

# 列出可用的 ROS2 包
ros2 pkg list

# 检查环境变量
echo $ROS_DISTRO
echo $AMENT_PREFIX_PATH
```

## 其他资源

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [ROS2 macOS 安装指南](https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html)
- [colcon 文档](https://colcon.readthedocs.io/)

## 注意事项

- macOS 上的 ROS2 支持可能不如 Linux 完整
- 某些功能可能需要额外配置
- 建议使用最新版本的 macOS 和 Xcode Command Line Tools
- 如果遇到问题，Docker 方案通常更稳定
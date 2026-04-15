# 快速部署与上手指南

> 如需了解系统各模块的详细架构设计，请参阅 [系统架构详解](ARCHITECTURE.md)。

## 0. 一键配置（推荐）
如果你使用全新的 Ubuntu 22.04 系统，可以直接运行一键配置脚本完成所有环境安装和编译：
```bash
bash scripts/setup_env.sh
```

该脚本会依次执行以下步骤：
1. **安装 ROS2 Humble**: 添加官方 APT 源，安装 `ros-humble-desktop` 和 `ros-dev-tools`
2. **安装 Gazebo Ignition Fortress**: 通过 `ros-humble-ros-gz` 安装仿真器
3. **安装系统依赖**: Eigen3、OpenMP、PCL、Nav2、SLAM Toolbox、serial-driver 等
4. **编译安装 small_gicp v1.0.0**: 从 GitHub 克隆并编译（需要 C++17）
5. **初始化 rosdep**: 配置 ROS 包依赖管理
6. **创建工作空间并编译**: 在 `~/sentry_ws` 下创建工作空间并执行 `colcon build`
7. **配置 bashrc（可选）**: 询问是否将工作空间环境写入 `~/.bashrc`

如果你希望手动逐步配置，请继续阅读以下章节。

## 1. 环境要求
- Ubuntu 22.04
- ROS2 Humble
- Gazebo Ignition Fortress (仿真可选)
- 配套仿真包: rmu_gazebo_simulator (已包含在 src/simulator 中)

## 2. Docker 部署 (最快方式)
Docker 是快速体验本项目的推荐方式。

### Docker 安装
请参考 Docker 官方文档安装 Docker Engine 和 Docker Compose。

### 允许 Docker 访问显示器
在宿主机执行：
```bash
xhost +local:docker
```

### 运行容器
使用以下命令启动容器：
```bash
docker run -it --rm \
  --name sentry_nav_container \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  sentry_nav:latest
```
注意：镜像名可能需要根据实际构建的标签进行更新。

## 3. 源码编译部署

### 3.1 安装依赖
本项目依赖 small_gicp 进行点云配准，请先编译安装：
```bash
sudo apt install -y libeigen3-dev libomp-dev
git clone https://github.com/koide3/small_gicp.git
cd small_gicp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### 3.2 创建工作空间
```bash
mkdir -p ~/sentry_ws/src
cd ~/sentry_ws
# 将本项目的 src/ 内容链接到工作空间
ln -sf <path_to_sentry>/src/* src/
```

### 3.3 安装 ROS 依赖
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

### 3.4 编译
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
说明：推荐使用 --symlink-install 选项，修改参数文件后无需重新编译。

## 4. 仿真模式

> **Wayland 用户注意**: 如果使用 Wayland 桌面环境（Ubuntu 22.04 默认），Gazebo GUI 的 Play 按钮可能无法点击。解决方法：
> - 方法 1：启动 Gazebo 前设置 `export QT_QPA_PLATFORM=xcb`
> - 方法 2：使用命令行 unpause：`ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 5000 --req 'pause: false'`

### 4.0 启动 Gazebo 仿真器（必须先启动）
```bash
# Wayland 环境下加 QT_QPA_PLATFORM=xcb
QT_QPA_PLATFORM=xcb ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```
等待 Gazebo 窗口出现后，点击 Play 按钮或使用上述命令行 unpause。确认仿真时钟开始运行后，再在新终端启动导航。

### 4.1 单机器人导航
```bash
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmul_2026 slam:=False
```

### 4.2 单机器人建图
```bash
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py slam:=True
```
保存地图：
```bash
ros2 run nav2_map_server map_saver_cli -f <MAP_NAME> --ros-args -r __ns:=/red_standard_robot1
```

### 4.3 多机器人 (实验性)
```bash
ros2 launch sentry_nav_bringup rm_multi_navigation_simulation_launch.py world:=rmul_2024 robots:="red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14};"
```

## 5. 实车模式

### 5.0 一键启动（推荐）
```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py
```
该脚本自动启动串口驱动、Point-LIO、导航栈和行为树，无需手动逐个启动。

### 5.1 建图
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True
```

### 5.2 导航
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py world:=<YOUR_WORLD_NAME> slam:=False use_robot_state_pub:=True
```

## 6. 行为树决策
启动行为树节点以实现自主决策：
```bash
ros2 launch sentry_behavior sentry_behavior_launch.py
```

## 7. 手柄控制
- 默认开启 PS4 手柄支持。
- 键位映射：详见 sentry_nav_bringup/config/simulation/nav2_params.yaml 中 teleop_twist_joy_node 部分。

## 8. 常见问题
- **编译错误**: 请确保 small_gicp (>= v1.0.0) 已正确安装并位于系统路径中。
- **仿真器无法启动**: 请检查 Gazebo Ignition Fortress 是否已正确安装并能独立运行。
- **Gazebo Play 按钮无响应**: Wayland 环境下的已知问题，设置 `QT_QPA_PLATFORM=xcb` 后重启 Gazebo，或使用命令行 unpause（见第 4.0 节）。
- **Point-LIO 报 `lidar loop back, clear buffer`**: 仿真启动时序问题。确保先启动 Gazebo 并 unpause，等仿真时钟稳定后再启动导航栈。通常 IMU 初始化完成后会自行恢复。
- **RViz 中无法显示**: 请检查 namespace 是否与启动参数中的机器人名称一致。
- **先验点云**: point_lio 和 small_gicp 需要先验点云文件，由于文件体积较大，不包含在基础仓库中，请自行准备。
- **重定位后机器人在 RViz 中位置异常**: 请确认先验 PCD 地图与 2D 占用栅格地图使用相同的坐标原点。

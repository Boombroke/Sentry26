[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# Sentry Nav - ROS2 哨兵导航系统

## 项目概述
本项目是为 RoboMaster 比赛哨兵机器人开发的 ROS2 导航系统。系统基于 ROS2 Humble 和 Ubuntu 22.04 构建，集成了定位、感知、规划与决策功能。该系统参考深圳北理莫斯科大学 PolarBear 战队开发。

## 系统架构
系统架构包含以下核心模块：
- 定位模块：使用 point_lio 提供高频里程计，结合 small_gicp (v1.0.0) 实现全局重定位（含 GICP 质量门控、可配置参数、可选周期性重定位）。
- 感知模块：针对倾斜安装的 Livox mid360 雷达进行点云处理，支持地形分析与障碍物检测。
- 规划模块：基于 Nav2 框架，使用全局规划器配合自定义的 omni_pid_pursuit_controller 局部控制器。
- 决策模块：利用 BehaviorTree.CPP 构建行为树，实现复杂的比赛逻辑调度。
- 仿真环境：支持 Gazebo Ignition Fortress 仿真，实现算法的快速迭代与验证。

## 功能特性
- 完整支持 ROS2 Humble 架构。
- 支持 SLAM 建图模式与导航模式切换。
- 集成 PS4 手柄控制功能。
- 适配倾斜安装的 Livox mid360 激光雷达。
- 提供多机器人协作支持（实验性）。
- 包含完整的 Gazebo Ignition 仿真环境。
- 针对全向移动底盘优化的 PID 追踪控制器。

## 目录结构
```
src/
├── scripts/                     # 自动化脚本（环境配置等）
├── docs/                        # 项目文档（部署指南等）
├── sentry_nav/                  # 顶层元包（含子包：fake_vel_transform, ign_sim_pointcloud_tool, livox_ros_driver2, loam_interface, teleop_twist_joy, sentry_nav(元描述), nav2_plugins, omni_pid_pursuit_controller, point_lio, pointcloud_to_laserscan, sensor_scan_generation, terrain_analysis, terrain_analysis_ext）
├── sentry_nav_bringup/          # 启动文件、配置、地图、行为树
├── sentry_robot_description/    # 机器人URDF/SDF描述
├── sentry_behavior/             # 基于BehaviorTree.CPP的行为决策树
├── rm_interfaces/               # RoboMaster自定义消息接口
├── auto_aim_interfaces/         # 自动瞄准接口
├── serial/                      # 串口通信驱动
├── sentry_tools/                # 开发调试工具集（串口Mock/地图坐标拾取/数据可视化）
├── odom_interpolator/           # 里程计插值
├── small_gicp_relocalization/   # 基于small_gicp的重定位
├── BehaviorTree.ROS2/           # 行为树ROS2集成
├── simulator/                   # Gazebo仿真（rmoss_core, rmoss_gazebo, rmu_gazebo_simulator, sdformat_tools等）
├── costmap_converter/           # 代价地图转换
└── teb_local_planner/           # TEB局部规划器
```

## 依赖环境
- 操作系统：Ubuntu 22.04
- ROS 版本：ROS2 Humble
- 仿真器：Gazebo Ignition Fortress
- 硬件要求：Livox mid360 激光雷达，全向移动底盘

## 编译

### 一键配置环境（推荐）
```bash
bash src/scripts/setup_env.sh
```

### 手动编译
```bash
# 安装 ROS2 依赖
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y

# 全量编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载环境
source install/setup.bash

# 单包编译（示例）
colcon build --packages-select sentry_behavior --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 快速开始

### 仿真模式
```bash
# 启动 Gazebo 仿真器（Wayland 环境需加 QT_QPA_PLATFORM=xcb）
QT_QPA_PLATFORM=xcb ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# 确保 Gazebo 已 unpause 后，启动导航（另一终端）
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmul_2026 slam:=False
```

### 实车模式
```bash
# 建图
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True

# 导航
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py world:=<WORLD_NAME> slam:=False use_robot_state_pub:=True
```

详细的部署步骤请参考 [快速部署与上手指南](src/docs/QUICKSTART.md)。系统各模块的详细架构设计请参阅 [系统架构详解](src/docs/ARCHITECTURE.md)。

## 参数说明
启动文件支持以下主要参数配置：

| 参数名称 | 说明 | 默认值 |
| :--- | :--- | :--- |
| namespace | 机器人命名空间 | "" |
| use_sim_time | 是否使用仿真时间 | false |
| slam | 是否开启 SLAM 模式 | false |
| world | 仿真世界文件路径 | "" |
| map | 导航地图文件路径 | "" |
| params_file | 导航参数配置文件路径 | "" |
| use_rviz | 是否启动 RViz 可视化 | true |
| use_robot_state_pub | 是否发布机器人状态 | true |

## 开发调试工具

项目内置了一套独立的调试工具集（`src/sentry_tools/`），无需启动完整 ROS 节点即可进行协议联调和导航配置。

### 工具箱（串口 Mock + 地图坐标拾取）
```bash
python3 src/sentry_tools/sentry_toolbox.py
```
- **串口 Mock**：模拟电控发送 IMU/状态/血量三类协议包，接收并显示 ROS 下发的速度指令
- **地图坐标拾取**：加载 `.yaml + .pgm` 地图，鼠标拾取导航目标点坐标，支持复制到剪贴板

### 串口数据可视化（需 ROS 环境）
```bash
source install/setup.bash
python3 src/sentry_tools/serial_visualizer.py
```
- 实时滚动曲线：云台 pitch/yaw、导航速度 vx/vy/vw
- 仪表盘：比赛阶段、HP 进度条、弹量、全队血量柱状图
- 暗色主题，类似轻量版 Foxglove

详见 [sentry_tools 使用文档](src/sentry_tools/README.md)。

## 许可证
本项目采用 Apache-2.0 许可证。

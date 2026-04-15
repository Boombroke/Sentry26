# sentry_nav_bringup

## 简介
sentry_nav_bringup 是哨兵机器人导航系统的核心启动配置包。它整合了所有 launch 脚本、Nav2 参数配置、地图数据、点云文件、RViz 配置文件以及行为树 XML 定义。

## 主要功能
本包负责协调导航系统内各个功能模块的启动顺序与参数传递。它提供了仿真环境与实车环境两套独立的配置方案，并支持多机器人协同部署模式。

## 主要 Launch 文件
- **rm_navigation_simulation_launch.py**: 仿真环境导航主入口。
  - 参数: namespace (命名空间), slam (是否开启建图), world (仿真地图名称), map (地图文件路径), params_file (参数文件), use_rviz (是否启动可视化)。
- **rm_navigation_reality_launch.py**: 实车环境导航主入口。
- **rm_sentry_launch.py**: 实车一键启动（导航 + 串口驱动），整合 rm_navigation_reality_launch.py 和 rm_serial_driver。
- **rm_multi_navigation_simulation_launch.py**: 多机器人仿真启动脚本。
- **bringup_launch.py**: 基础导航功能的一键启动。
- **模块化启动脚本**:
  - slam_launch.py: 启动 SLAM 建图模块。
  - localization_launch.py: 启动基于先验地图的定位模块。
  - navigation_launch.py: 启动 Nav2 导航框架。
  - robot_state_publisher_launch.py: 发布机器人 TF 变换与模型状态。
  - rviz_launch.py: 启动预配置的可视化界面。
  - joy_teleop_launch.py: 启动手柄远程控制节点。

## 订阅话题
- /initialpose: 接收初始位姿估计。
- /goal_pose: 接收导航目标点。

## 发布话题
- /tf, /tf_static: 发布坐标变换树。
- /map: 发布当前使用的环境地图。

## 配置与资源目录
- **config/**: 包含 simulation/ 和 reality/ 目录，存放 nav2_params.yaml 等核心配置文件。
- **map/**: 存放仿真与实车环境的二维地图文件。注意：部分大尺寸地图文件未包含在 Git 仓库中。
- **pcd/**: 存放用于定位的先验点云文件。注意：大尺寸点云文件需手动下载至对应目录。
- **rviz/**: 存放不同调试场景下的 RViz 配置文件。
- **behavior_trees/**: 存放导航专用的行为树逻辑定义。

## 使用示例
在仿真环境中启动导航并加载指定地图：
```bash
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmul_2026 slam:=False
```

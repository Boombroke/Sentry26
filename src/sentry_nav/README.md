# sentry_nav

## 简介
sentry_nav 是哨兵机器人自主导航系统的顶层元包。它通过依赖管理整合了多个功能子包，构建了从感知、定位到规划与控制的完整技术栈。

## 子包列表及功能说明
- **sentry_nav**: 元包描述文件，定义了整个导航系统的依赖关系。
- **point_lio**: 基于 Point-LIO 算法的激光惯性里程计，提供高精度、高频的位姿估计。
- **livox_ros_driver2**: Livox 激光雷达的 ROS2 驱动程序，负责原始数据采集。
- **odom_bridge**: 里程计桥接节点。合并了原 loam_interface 和 sensor_scan_generation，在单次同步回调中完成 lidar_odom→odom 变换、2D 约束、TF 广播、速度计算和点云转换。
- **nav2_plugins**: 自定义 Nav2 插件集合，包含强度体素层 (intensity_voxel_layer) 等。
- **omni_pid_pursuit_controller**: 专为全向轮底盘设计的 PID 路径追踪控制器。
- **teleop_twist_joy**: 支持 PS4 等标准手柄的远程控制模块。
- **fake_vel_transform**: 速度矢量在不同坐标系之间的转换工具。
- **ign_sim_pointcloud_tool**: 将 Ignition 仿真环境中的原始数据转换为标准点云格式。
- **pointcloud_to_laserscan**: 将三维点云数据投影为二维激光扫描数据，兼容传统导航算法。
- **terrain_analysis**: 基础地形分析模块，用于实时检测环境中的障碍物。
- **terrain_analysis_ext**: 地形分析扩展模块，提升了机器人在复杂坡道与障碍物环境下的通过性。

## 系统核心话题
- **订阅**:
  - /cmd_vel: 接收上层下发的底盘控制速度。
  - /livox/lidar: 接收激光雷达原始点云数据。
- **发布**:
  - /odom: 发布融合后的机器人里程计信息。
  - /scan: 发布投影后的二维激光扫描数据。

## 参数说明
系统参数主要通过 sentry_nav_bringup 包中的 YAML 文件进行统一管理。各子包通过 ROS2 参数服务器读取各自的运行配置。

## 使用方法
本元包主要用于工作空间的统一编译。在工作空间根目录下执行：
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 目录结构概述
本仓库遵循标准的 ROS2 工作空间布局。所有核心功能包均位于 src/ 目录下，便于统一编译与管理。

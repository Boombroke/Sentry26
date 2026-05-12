# sentry_nav

## 简介
sentry_nav 是哨兵机器人自主导航系统的顶层元包。它通过依赖管理整合了多个功能子包，构建了从感知、定位到规划与控制的完整技术栈。

## 子包列表及功能说明
- **sentry_nav**: 元包描述文件，定义了整个导航系统的依赖关系。
- **point_lio**: 基于 Point-LIO 算法的激光惯性里程计，提供高精度、高频的位姿估计。
- **livox_ros_driver2**: Livox 激光雷达的 ROS2 驱动程序，负责原始数据采集。
- **odom_bridge**: 里程计桥接节点。合并了原 loam_interface 和 sensor_scan_generation，在单次同步回调中完成 lidar_odom→odom 变换、2D 约束、`odom → base_footprint` TF 广播、速度计算和点云转换。云台雷达方案下通过每帧 `lookupTransform(lidar_frame → base_frame)` 消化云台旋转和 Mid360 安装外参对底盘位姿的影响；发布给 Nav2 的 odometry 始终保持 2D 底盘语义。
- **nav2_plugins**: 自定义 Nav2 插件集合，提供 `IntensityVoxelLayer` 强度体素代价地图层。`BackUpFreeSpace` 恢复行为已删除，贴墙脱困由 `sentry_motion_manager` recovery 状态机接管。
- **ign_sim_pointcloud_tool**: 将 Ignition 仿真环境中的原始数据转换为标准点云格式。
- **pointcloud_to_laserscan**: 将三维点云数据投影为二维激光扫描数据，兼容传统导航算法。
- **terrain_analysis**: 基础地形分析模块，用于实时检测环境中的障碍物。
- **terrain_analysis_ext**: 地形分析扩展模块，提升了机器人在复杂坡道与障碍物环境下的通过性。

> 局部控制器当前为 MPPI DiffDrive，由 Nav2 Jazzy apt 包 `nav2_mppi_controller` 提供，仿真 Phase A 与实车 Phase R-A 首版均使用。`nav2_regulated_pure_pursuit_controller`、`nav2_rotation_shim_controller`（旧 RPP + RotationShim 组合）仅作为回滚依赖保留，不是当前运行链路。所有控制器插件均由 Nav2 apt 提供，不在本元包内实现。

## 系统核心话题
- **订阅**:
  - /livox/lidar: 接收激光雷达原始点云数据。
  - /livox/imu: 接收 IMU 数据。
- **发布**:
- /odometry: 发布融合后的机器人里程计信息（odom_bridge 2D 底盘位姿 + 差分速度）。
  - /obstacle_scan: 发布投影后的二维激光扫描数据（供 SLAM toolbox 使用）。
  - /registered_scan / /lidar_odometry: 供 terrain_analysis / terrain_analysis_ext 使用。

## 参数说明
系统参数主要通过 sentry_nav_bringup 包中的 YAML 文件进行统一管理。各子包通过 ROS2 参数服务器读取各自的运行配置。

## 使用方法
本元包主要用于工作空间的统一编译。在工作空间根目录下执行：
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 目录结构概述
本仓库遵循标准的 ROS2 工作空间布局。所有核心功能包均位于 src/ 目录下，便于统一编译与管理。

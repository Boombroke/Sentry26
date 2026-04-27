# odom_bridge

## 简介

`odom_bridge` 将原 `loam_interface` 与 `sensor_scan_generation` 的功能合并为单一 Composable Node。
节点直接订阅 Point-LIO 的原始里程计和注册点云，在一次同步回调中完成：

1. `lidar_odom -> odom` 变换
2. `odom -> chassis(base_frame)` 2D 约束（z=0, roll=0, pitch=0）
3. `odom -> robot_base_frame` 里程计发布（`pose` 在 `odom` 系，只保留 x、y、yaw；`twist` 投影到 `robot_base_frame` 机体系，只保留差速可执行的 `vx + wz`）
4. 点云转换并发布 `sensor_scan`（lidar 系）
5. 发布 `registered_scan`（odom 系点云）和 `lidar_odometry`（odom 系里程计），供 terrain_analysis/ext 使用
6. TF 广播 `odom -> base_frame`

相较旧链路，去除了独立的 loam_interface / sensor_scan_generation 节点和额外一次同步步骤，减少一次发布-订阅 hop。`registered_scan` 和 `lidar_odometry` 话题保留以兼容下游 terrain_analysis/ext。

## 订阅话题

- `state_estimation_topic` (`nav_msgs/msg/Odometry`，默认 `aft_mapped_to_init`)
- `registered_scan_topic` (`sensor_msgs/msg/PointCloud2`，默认 `cloud_registered`)

两路输入使用 `message_filters::ApproximateTime` 同步。

## 发布话题

- `sensor_scan` (`sensor_msgs/msg/PointCloud2`，lidar 系)
- `odometry` (`nav_msgs/msg/Odometry`，odom -> robot_base_frame，2D pose + 差速速度语义 `vx / wz`)
- `registered_scan` (`sensor_msgs/msg/PointCloud2`，odom 系点云，供 terrain_analysis/ext)
- `lidar_odometry` (`nav_msgs/msg/Odometry`，odom -> lidar_frame，供 terrain_analysis/ext)
- `odom_to_lidar_odom` (`geometry_msgs/msg/TransformStamped`，latched，odom -> lidar_odom 的静态偏移，主要供 Point-LIO 退出保存 PCD 时转换到 odom/map 系，以及诊断坐标一致性使用；small_gicp_relocalization 不再依赖该话题)
- `robot_flipped` (`std_msgs/msg/Bool`，底盘翻车检测结果；`true` 表示 roll 或 pitch 超过阈值，与雷达安装角无关)

## TF 发布

- `odom -> base_frame`

## 参数

- `state_estimation_topic` (string, 默认: `aft_mapped_to_init`)
- `registered_scan_topic` (string, 默认: `cloud_registered`)
- `odom_frame` (string, 默认: `odom`)
- `base_frame` (string, 默认: `base_footprint`)
- `lidar_frame` (string, 默认: `front_mid360`)
- `robot_base_frame` (string, 默认: `base_footprint`)
- `min_twist_dt` (double, 默认: `0.001`)：小于该间隔的重复时间戳不计算速度
- `max_twist_dt` (double, 默认: `0.1`)：大于该间隔的 LIO 空窗不跨帧差分，避免速度爆点
- `max_valid_linear_speed` (double, 默认: `3.0`)：机体系 `vx` 合法上限，超过视为 LIO/TF 跳变
- `max_valid_lateral_speed` (double, 默认: `0.35`)：机体系 `vy` 合法上限，差速底盘正常应接近 0
- `max_valid_angular_speed` (double, 默认: `6.3`)：`wz` 合法上限，保留实车约 `2pi rad/s` 转速能力
- `max_valid_linear_accel` (double, 默认: `12.0`)：`vx` 单帧加速度门限
- `max_valid_angular_accel` (double, 默认: `40.0`)：`wz` 单帧角加速度门限
- `twist_filter_alpha` (double, 默认: `0.45`)：合法速度的一阶低通系数，范围 `[0, 1]`
- `flip_roll_threshold` (double, 默认: `0.5236`)：底盘 roll 超过此值（弧度）时发布翻车告警，默认 30°
- `flip_pitch_threshold` (double, 默认: `0.5236`)：底盘 pitch 超过此值（弧度）时发布翻车告警，默认 30°

## `/odometry` 语义

`/odometry.pose` 表示 `odom -> robot_base_frame`，因此位置和朝向属于 `odom` 里程计世界系。`/odometry.twist` 按 ROS Odometry 约定表达在 `child_frame_id`，本包发布的是 `robot_base_frame` 机体系速度：差速底盘只填 `linear.x` 和 `angular.z`，`linear.y` 始终置零。

速度由相邻 2D pose 差分得到，因此 LIO 单帧位姿跳变、TF 时间戳异常或长时间空窗都会放大成速度尖峰。本节点会按 dt、速度上限、加速度上限过滤异常值，并对合法速度做轻量低通；过滤只影响 `/odometry.twist`，不会篡改 `/odometry.pose` 或 `odom -> base_frame` TF。

## PCD 坐标系约定

`registered_scan` 始终发布为 odom 系点云。当前建图流程中，Point-LIO 在退出保存 PCD 时订阅本节点 latched 的 `odom_to_lidar_odom`，将原始 `lidar_odom` 系点云转换到 odom/map 系后保存。后续 small_gicp_relocalization 会直接把 `prior_pcd_file` 当作 odom/map 系目标地图使用，不再对旧 `lidar_odom` 系 PCD 做兼容转换；旧 PCD 必须重新建图生成。

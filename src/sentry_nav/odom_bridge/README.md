# odom_bridge

## 简介

`odom_bridge` 将原 `loam_interface` 与 `sensor_scan_generation` 的功能合并为单一 Composable Node。
节点直接订阅 Point-LIO 的原始里程计和注册点云，在一次同步回调中完成：

1. `lidar_odom -> odom` 变换
2. `odom -> chassis(base_frame)` 2D 约束（z=0, roll=0, pitch=0）
3. `odom -> robot_base_frame` 里程计发布（含位置差分速度）
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
- `odometry` (`nav_msgs/msg/Odometry`，odom -> robot_base_frame，含位置差分速度)
- `registered_scan` (`sensor_msgs/msg/PointCloud2`，odom 系点云，供 terrain_analysis/ext)
- `lidar_odometry` (`nav_msgs/msg/Odometry`，odom -> lidar_frame，供 terrain_analysis/ext)
- `odom_to_lidar_odom` (`geometry_msgs/msg/TransformStamped`，latched，odom -> lidar_odom 的静态偏移，供 small_gicp_relocalization 使用，确保与 registered_scan 的坐标变换一致)

## TF 发布

- `odom -> base_frame`

## 参数

- `state_estimation_topic` (string, 默认: `aft_mapped_to_init`)
- `registered_scan_topic` (string, 默认: `cloud_registered`)
- `odom_frame` (string, 默认: `odom`)
- `base_frame` (string, 默认: `base_footprint`)
- `lidar_frame` (string, 默认: `front_mid360`)
- `robot_base_frame` (string, 默认: `gimbal_yaw`)

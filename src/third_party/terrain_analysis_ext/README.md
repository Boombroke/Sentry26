# terrain_analysis_ext

## 简介
terrain_analysis的扩展模块,旨在对地形分析结果进行深度处理和范围扩展。

## 功能
- 地图维护: 维护更大范围的局部地形地图。
- 代价地图支持: 为Nav2代价地图的生成提供预处理数据。
- 结果增强: 与terrain_analysis配合,提升地形感知的稳定性。

## 话题
### 订阅
- `terrain_map` (sensor_msgs/PointCloud2): terrain_analysis输出的地形点云。
- `registered_scan` (sensor_msgs/PointCloud2): odom 系点云（来自 odom_bridge）。
- `lidar_odometry` (nav_msgs/Odometry): odom 系里程计（来自 odom_bridge）。
- `joy` (sensor_msgs/Joy): 手柄输入。
- `cloud_clearing` (std_msgs/Float32): 点云清除触发。

### 发布
- `terrain_map_ext` (sensor_msgs/PointCloud2): 经过扩展和处理后的地形点云。

## 使用
该节点通常紧随terrain_analysis运行,为高层规划器提供更全面的环境信息。

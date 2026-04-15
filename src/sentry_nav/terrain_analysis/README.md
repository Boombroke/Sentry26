# terrain_analysis

## 简介
基于CMU exploration项目的地形分析模块,用于评估机器人周围环境的可通行性。原始代码由 Hongbiao Zhu 开发。

## 功能
- 点云体素化: 对输入点云进行降采样处理。
- 地形评估: 分析地形的高度、坡度等特征。
- 可通行性生成: 标注点云中的可通行区域与障碍物。

## 话题
### 订阅
- `registered_scan` (sensor_msgs/PointCloud2): odom 系点云（来自 odom_bridge）。
- `lidar_odometry` (nav_msgs/Odometry): odom 系里程计（来自 odom_bridge）。
- `joy` (sensor_msgs/Joy): 手柄输入。
- `map_clearing` (std_msgs/Float32): 地图清除触发。

### 发布
- `terrain_map` (sensor_msgs/PointCloud2): 标注了地形可通行性的点云地图。

## 参数
- `scanVoxelSize`: 扫描点云的体素大小。
- `decayTime`: 点云衰减时间。
- `vehicleHeight`: 机器人高度。
- `minRelZ`: 相对地面最小高度。
- `maxRelZ`: 相对地面最大高度。

## 使用
该模块输出的地形地图是自主导航中避障和路径规划的重要依据。

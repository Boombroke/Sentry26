# 哨兵导航系统架构详解

> **Maintainer**: Boombroke <boombroke@icloud.com>
> **基于**: [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) by Lihan Chen，2026 差速轮足架构迁移由 Boombroke 主导
> 相关文档：[快速部署](QUICKSTART.md) · [运行模式](RUNNING_MODES.md) · [参数调优](TUNING_GUIDE.md)

## 第1章: 系统总览

### 1.1 项目背景与功能描述
本项目是为 RoboMaster 哨兵机器人量身定制的自主导航系统。哨兵机器人在比赛中承担着全自动防御和进攻的核心任务，要求系统具备极高的定位精度、环境感知能力以及复杂的战术决策逻辑。本系统集成了从底层传感器驱动到高层行为树决策的全栈功能，确保机器人在动态变化的赛场环境中能够稳定运行。

系统核心目标包括：
- **高精度定位**: 在剧烈运动和碰撞中保持厘米级的定位精度。
- **差速轮足控制**: 基于差速轮足底盘（仅 vx/wz 自由度，无侧向 vy），仿真与实车均使用 Nav2 MPPI DiffDrive 单控制器。仿真为 Phase A（`vx_max=1.5、wz_max=6.3、az_max=8.0`，20Hz），实车为 Phase R-A 首版（`vx_min=0.0` 前向优先、`wz_max=3.0`、`az_max=5.0`、30Hz），全场上场验证依赖后续台架/上场 smoke。
- **智能战术决策**: 根据裁判系统实时数据，自动切换进攻、防守、补给等状态。
- **环境自适应**: 能够识别坡道、台阶等复杂地形，并生成准确的代价地图。

### 1.2 系统数据流图
以下展示了从原始传感器数据输入到最终底盘执行器输出的完整数据通路。系统采用模块化设计，各组件通过 ROS2 话题和服务进行解耦。**感知层在 2026 赛季升级为前后反向双 Mid360**：两颗 Mid360 都挂 `gimbal_pitch`，在 Point-LIO 之前由 `sentry_dual_mid360::MergerNode` 做外部前融合；Point-LIO 当单雷达消费融合结果，源码零改动。

```text
[Front Mid360 192.168.1.144] --+--> [/livox/lidar_front (CustomMsg)] --+
                               |                                       |
                               +--> [/livox/imu]  (唯一进 LIO 的 IMU)  |--> [pointcloud_merger
                                                                       |     ApproximateTime 同步
[Back  Mid360 192.168.1.145] --+--> [/livox/lidar_back  (CustomMsg)] --+     back→front 刚体变换
                               |                                             stable_sort offset_time
                               +--> [/livox/imu_back] (仅诊断，不进 LIO)      header.stamp 单调]
                                                                             |
                                                                             v
                                                             [/livox/lidar (CustomMsg, frame=front_mid360)]
                                                                             |
+-----------------------+      +-----------------------+      +-----------+-+---------+
|       感知层 (Perception)     |       定位层 (Localization)    |       规划层 (Planning)       |
+-----------+-----------+      +-----------+-----------+      +-----------+-----------+
            |                              |                              |
[融合后 CustomMsg] --------> [Point-LIO 里程计] ----+------> [Nav2 全局规划器]
                              (lidar_odom -> lidar)  |        (SmacPlannerHybrid)
                                     |               |               |
                                     v               |               v
                                     |               |        [Nav2 局部控制器]
[先验 PCD 地图 (双 Mid360)] --> [Small GICP 重定位] --+        (仿真: MPPI DiffDrive
                              (map -> odom 修正)      |         实车: MPPI DiffDrive 首版)
                                     |               |               v
[视觉识别 (Armors)] ------> [BehaviorTree.CPP] <---+        [Velocity Smoother]
                            (高层战术决策)          |        (vy 锁 0, 输出 cmd_vel_nav)
                                    ^               |               |
[裁判系统 (Referee)] ------+       |               |               v
                            |       |               |        [sentry_motion_manager]
[底盘反馈 (Odom)] ---------+-------+               |        (仲裁后发布 /cmd_vel)
                                                    |               |
                                                    |               v
                                                   |        Gazebo DiffDrive / rm_serial_driver
                                                   +------> [云台独立接口]
                                                            (下位机上报 yaw/pitch)
```

### 1.2.1 双 Mid360 数据流

双 Mid360 升级后，原始传感器层到 Point-LIO 之间新增一层外部前融合：

- **硬件布局**：前后两颗 Livox Mid360 都固定到 `gimbal_pitch`，前雷达沿安装坐标系朝前、后雷达通过 `yaw=π` 反向安装，水平 FOV 互补近 360°。
- **IMU 路由**：只有前 Mid360 的 IMU 进 Point-LIO（话题 `/livox/imu`）；后 Mid360 的 IMU 仅作诊断（`/livox/imu_back`），不进 LIO 状态估计，避免双 IMU 冲突。
- **驱动模式**：Livox driver 运行在 `multi_topic=1 / xfer_format=1` 下，发布两路独立 CustomMsg；launch 层用 per-device IP 后缀 remap 出 `/livox/lidar_front` 与 `/livox/lidar_back`。
- **Merger 节点**：`sentry_dual_mid360::MergerNode` 用 `message_filters::ApproximateTime` + `setMaxIntervalDuration` 双重门控同步，back 点云在原生系做 min-distance 过滤后，经 `back_mid360 → front_mid360` 刚体变换合并到前雷达系；逐点 `offset_time` 经 `std::stable_sort` 保持单调，输出 `/livox/lidar` 的 `header.stamp` 强制单调递增、`timebase=0`、保留 `tag / reflectivity / line` 字段。
- **Point-LIO 零改动**：Point-LIO 订阅 `/livox/lidar` 当单雷达消费，源码不分叉。`extrinsic_T / extrinsic_R / mapping.gravity / gravity_init` 由 `sentry_dual_mid360/scripts/generate_pointlio_overrides.py` 在 `colcon build` 时从 xmacro 单源派生到 `install/sentry_dual_mid360/share/sentry_dual_mid360/config/pointlio_dual_overrides.yaml`，launch 以 `IfCondition(use_dual_mid360)` 叠加该 override。
- **顶层开关**：`use_dual_mid360` 默认 `True`，逐层透传到 slam / localization 两条分支。传 `False` 时 merger 不启动、生成的 override YAML 不加载、dual Livox override 与 per-device remap 均不应用，Livox driver 走基础 `configured_params`，Point-LIO 保持基础 YAML 的 `common.lid_topic: livox/lidar`，等价于升级前单 Mid360 链路（向后兼容）。
- **本地 BLOCKED**：双 Mid360 硬件同步质量判定、实车台架 smoke、双雷达 PCD 重建等 live 验证项依赖目标硬件。本地开发机缺硬件时这些项目走 BLOCKED。仿真侧 CustomMsg 桥已落地（`sentry_dual_mid360::SimPointCloudToCustomMsgNode` + `sim_custommsg_bridge_launch.py`，`use_dual_mid360:=True` 在 `rm_navigation_simulation_launch.py` 中自动拉起两实例），实际 Gazebo smoke 需要能启动 Gazebo 的机器跑（详见 `src/sentry_nav/sentry_dual_mid360/docs/ARCHITECTURE.md` §13）。

### 1.3 核心框架与版本
- **操作系统**: Ubuntu 24.04 LTS (Noble Numbat)
- **中间件**: ROS2 Jazzy Jalisco
- **定位算法**: Point-LIO (激光惯性紧耦合) + Small GICP (点云配准重定位)
- **导航框架**: Nav2 (Navigation2) 及其自定义插件
- **决策系统**: BehaviorTree.CPP 4.x + BehaviorTree.ROS2 0.3.0
- **仿真平台**: Gazebo Harmonic
- **构建系统**: ament_cmake / colcon (Release 模式优化)

---

## 第2章: 感知与定位模块

### 2.1 Point-LIO 里程计
Point-LIO 是系统的核心里程计方案，通过紧耦合 IMU 和激光雷达数据，提供高频、低延迟的位姿估计。**源码零改动**：双 Mid360 升级后仍把 Point-LIO 当单雷达消费，所有融合逻辑在 Point-LIO 之前由 `sentry_dual_mid360::MergerNode` 完成。

- **功能**: 实现激光惯性里程计，输出机器人相对于起始点的位姿。
- **输入话题（双雷达实车链路，`use_dual_mid360:=True` 默认）**:
    - `livox/lidar` (`livox_ros_driver2/msg/CustomMsg`): `pointcloud_merger` 输出的融合 CustomMsg，`frame_id=front_mid360`，逐点保留 `offset_time / reflectivity / tag / line`。
    - `livox/imu` (sensor_msgs/msg/Imu): 前 Mid360 IMU（唯一进 LIO 的 IMU；后 Mid360 IMU 走 `/livox/imu_back` 仅诊断）。
- **输入话题（单雷达回退，`use_dual_mid360:=False`）**:
    - `livox/lidar` (`livox_ros_driver2/msg/CustomMsg`): 基础单雷达 Livox driver 发布的默认话题；此模式下 merger 不启动、dual override 不加载、per-device remap 不应用，等价于升级前的单 Mid360 链路。
- **输入话题（仿真，`use_dual_mid360:=True`，默认）**:
    - Gazebo gz→ROS 桥发布 `livox/lidar_front_points` + `livox/lidar_back_points`（`sensor_msgs/msg/PointCloud2`）以及 `livox/imu` + `livox/imu_back`（`sensor_msgs/msg/Imu`）。
    - `sentry_dual_mid360::SimPointCloudToCustomMsgNode` 前后两实例把两路 PC2 转换成 `livox_ros_driver2/msg/CustomMsg`（`livox/lidar_front` / `livox/lidar_back`），再由 `MergerNode` 合并成 `livox/lidar` CustomMsg 喂 Point-LIO；与实车共用同一条 CustomMsg + merger 链路，Point-LIO 源码零改动。
- **输入话题（仿真，`use_dual_mid360:=False`）**:
    - `ign_sim_pointcloud_tool` 订阅 `livox/lidar_front_points`（base nav2_params.yaml 指定），转换成 `velodyne_points`（`sensor_msgs/msg/PointCloud2`）供 Point-LIO 单雷达回退链路使用。
- **输出话题**:
    - `aft_mapped_to_init` (nav_msgs/msg/Odometry): 优化后的里程计位姿。
    - `cloud_registered` (sensor_msgs/msg/PointCloud2): 投影到世界坐标系下的实时点云。
- **坐标变换 (TF)**: 发布 `lidar_odom -> front_mid360`（`front_mid360` 是融合输出的 common frame，Point-LIO 只看到一条主雷达链路）。
- **仿真适配**: `use_dual_mid360:=True`（默认）下由 `sentry_dual_mid360::SimPointCloudToCustomMsgNode` 前后两实例把 Gazebo `PointCloudPacked → sensor_msgs/PointCloud2` 转换成 Livox `CustomMsg` 喂 `MergerNode`，Point-LIO 当作单雷达消费合并后的 `livox/lidar`。`use_dual_mid360:=False` 回退链路沿用 `ign_sim_pointcloud_tool` 把前 Mid360 的 `PointCloudPacked` 转成 `velodyne_points`，Point-LIO 走 Velodyne 模式处理。
- **关键参数配置**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | lidar_type | 2 | 适配 Velodyne 格式的点云输入 |
    | filter_size_surf | 0.5 | 表面特征点云降采样步长 |
    | filter_size_map | 0.5 | 地图点云降采样步长 |
    | gravity | [0, 0, -9.81] (仿真) / codegen 派生（实车 dual 模式） | Point-LIO 重力状态初值；机体系姿态由 IMU 初始化自动对齐 |
    | extrinsic_T / extrinsic_R | codegen 派生（dual 模式） / YAML 默认（单雷达回退） | IMU 相对 LiDAR 外参，dual 模式下由 xmacro 单源 codegen 生成 |

- **实车双 Mid360 外参（当前值）**：两颗 Mid360 均相对 `gimbal_pitch` 下俯 30°（xmacro 顶层 pitch = `0.5235987755982988` rad），前雷达 `front_lidar_pose="-0.17 0 0.10 0.0 0.5235987755982988 3.141592653589793"`、后雷达 `back_lidar_pose="0.05 0 0.05 0.0 0.5235987755982988 3.141592653589793"`（yaw=π 反向安装）。安装角只留在机器人描述 / TF，**不**直接写入 Point-LIO `gravity`；dual 模式下 `gravity / gravity_init / extrinsic_T / extrinsic_R` 都由 codegen 从 xmacro 派生。仿真 `wheeled_biped_sim.sdf.xmacro` 的 Mid360 仍保留 30° 下俯用于覆盖近场低矮底座。
- **Point-LIO 零改动原则**: 双 Mid360 融合发生在 Point-LIO 之前；不对 `src/third_party/point_lio/` 做任何源码分叉。参数层面的切换通过 launch `IfCondition(use_dual_mid360)` 把 `install/sentry_dual_mid360/share/sentry_dual_mid360/config/pointlio_dual_overrides.yaml` 作为后置层叠加到单雷达默认 YAML 上；该 override YAML 是 build 产物，不在源码 `config/` 提交、不可手编辑（见 [参数调优 §1.5 / §七](TUNING_GUIDE.md) 与 `scripts/verify_pointlio_overrides_fresh.py`）。

### 2.2 Odom Bridge (里程计桥接)

`odom_bridge` 合并了原 `loam_interface` 与 `sensor_scan_generation`，在单次同步回调中完成完整链路：

**输入**：Point-LIO 的 `aft_mapped_to_init`（Odometry）+ `cloud_registered`（PointCloud2），均在 lidar_odom 坐标系。

**处理流程**：
1. 一次性标定 `base_frame → lidar_frame` 静态变换，得到 `odom ↔ lidar_odom` 的旋转偏移
2. 将 Point-LIO 输出从 lidar_odom 系变换到 odom 系
3. 对 `odom → chassis` 和 `odom → robot_base_frame` 都施加 2D 约束（z=0, roll=0, pitch=0），把传感器 6D 姿态与底盘 2D 导航语义解耦
4. 通过位置差分计算线速度和角速度，只保留 `vx / vy / wz` 的平面底盘语义，`z / wx / wy` 强制为 0

**输出**：
- TF: `odom → base_footprint`
- `odometry`（odom → gimbal_yaw，含差分速度）
- `sensor_scan`（lidar 系点云）
- `registered_scan`（odom 系点云，供 terrain_analysis/ext）
- `lidar_odometry`（odom 系里程计，供 terrain_analysis/ext）

### 2.3 Small GICP 重定位
- **功能**: 解决里程计累积漂移问题。通过将实时点云与预先构建的 PCD 地图进行配准，计算 `map -> odom` 的修正量。
- **PCD 坐标系**: `prior_pcd_file` 必须已经是与 `registered_scan` 一致的 odom/map 系点云。small_gicp_relocalization 加载成功后立即准备目标地图，不再订阅或应用 `odom_to_lidar_odom` 兼容旧 `lidar_odom` 系 PCD；旧 PCD 必须重新建图生成。
- **2D 约束逻辑**: 考虑到哨兵机器人在平整地面运行，GICP 计算出的 6DOF 变换被强制约束。修正量仅包含 x, y 和 yaw，而 z, roll, pitch 被置为 0，确保定位在平面上的稳定性。
- **质量门控机制**:
    - `inlier_ratio`: 匹配点比例，低于阈值则舍弃该帧。
    - `fitness_error`: 平均匹配误差，高于阈值则认为定位失效。
- **关键参数**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | max_iterations | 20 | GICP 最大迭代次数 |
    | accumulated_count_threshold | 10 | 累积多少帧点云后触发一次配准 |
    | min_inlier_ratio | 0.5 | 最小重合率要求 |
    | max_fitness_error | 0.15 | 最大允许匹配误差 |
    | enable_periodic_relocalization | true | 是否开启基于时间的周期性重定位 |
    | relocalization_interval | 5.0 | 周期性重定位的时间间隔（秒） |

### 2.4 TF 树完整路径
系统维护的完整 TF 树（双 Mid360 实装）如下：

```text
map (全局地图坐标系)
 └─ odom (里程计坐标系，由 small_gicp 修正)
     └─ base_footprint (Nav2 规划坐标系，水平 2D 投影)
         └─ chassis (底盘刚体原点，yaw ≡ base_footprint)
                 └─ gimbal_yaw (当前实车 profile 为静态 TF；仿真可动态控制)
                     └─ gimbal_pitch (当前实车 profile 为静态 TF；仿真可动态控制)
                         ├─ front_mid360 (前 Livox Mid360，随云台旋转)
                         │   └─ front_mid360_imu (Layer B IMU 出厂 anchor，进 Point-LIO)
                         └─ back_mid360  (后 Livox Mid360，yaw=π 反向安装，随云台旋转)
                             └─ back_mid360_imu  (Layer B IMU 出厂 anchor，仅诊断，不进 Point-LIO)
```

- `base_footprint` 与 `chassis` 为同一刚体在不同投影层级，yaw 一致，仅 z 抬升。
- 当前实车配置里，云台链与底盘是静态关系，`robot_state_publisher` 默认不再消费 `serial/gimbal_joint_state`。仿真 profile 仍保留动态 `gimbal_yaw_joint / gimbal_pitch_joint`。
- **前后两颗 Mid360 均挂在 `gimbal_pitch` 上**，随云台同步旋转；`back_mid360` 通过 `yaw=π` 反向安装补盲区。`odom_bridge` 通过每帧 `lookupTransform(lidar_frame → base_frame)` 消化云台旋转对底盘位姿估计的影响，使 Nav2 侧 `base_footprint` 位姿与云台解耦。
- `*_mid360_imu` 是 Layer B IMU factory anchor（Mid360 出厂 IMU 相对 LiDAR 的 `-0.011 -0.02329 0.04412 0 0 0`）；`front_mid360_imu` 的位姿是 Point-LIO `extrinsic_T/R` 的 codegen 派生源，`back_mid360_imu` 仅作诊断，不进 LIO 状态估计。
- 双 Mid360 外参只能改 xmacro（唯一真相源）：`wheeled_biped_real.sdf.xmacro` 的 `front_lidar_pose / back_lidar_pose` 和 `sentry_dual_mid360/urdf/mid360_imu_tf.sdf.xmacro` 的 IMU factory pose；改完重跑 `colcon build --packages-select sentry_dual_mid360`，codegen 会同步刷新 `pointlio_dual_overrides.yaml`。

---

## 第3章: 地形分析与代价地图

### 3.1 Terrain Analysis
- **功能**: 将 3D 点云转化为可用于导航的 2.5D 地形信息。它能够识别地面、台阶、斜坡以及障碍物。
- **输出话题**: `terrain_map` (sensor_msgs/msg/PointCloud2)
- **关键参数**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | vehicleHeight | 0.70 | 机器人自身高度，用于过滤上方遮挡 |
    | minRelZ | -0.1 | 相对地面的最小高度过滤 |
    | maxRelZ | 0.5 | 相对地面的最大高度过滤 |
    | terrainVoxelSize | 0.2 | 地形体素化分辨率 |
    | useSorting | true | 启用排序算法以支持斜坡识别 |
    | clearDyObs | false | 不主动清除动态障碍物，保留在 costmap 中用于避障 |

### 3.2 Terrain Analysis Ext
- **功能**: 扩展版地形分析，具有更大的感知半径（通常为 20m+），专门为全局路径规划提供远距离的障碍物信息。
- **输出话题**: `terrain_map_ext`

### 3.3 Pointcloud to Laserscan
- **功能**: 将 3D 点云投影为 2D LaserScan。这主要用于兼容 SLAM Toolbox 进行建图，或者作为某些 2D 避障算法的输入。
- **关键参数**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | min_height | 0.1 | 投影切片的下边界 |
    | max_height | 0.5 | 投影切片的上边界 |
    | angle_increment | 0.0043 | 扫描角增量 |
    | range_max | 10.0 | 最大有效探测距离 |

### 3.4 代价地图配置 (Costmap2D)
系统采用 Nav2 的标准代价地图结构，并引入了自定义插件。

- **Local Costmap (局部地图)**:
    - **尺寸**: 5.0m x 5.0m 滚动窗口。
    - **插件层**:
        1. `static_layer`: 订阅全局地图。
        2. `IntensityVoxelLayer`: 自定义插件（位于 `nav2_plugins` 包），直接处理 `terrain_map` 点云，利用强度信息标记 3D 障碍物。
        3. `inflation_layer`: 膨胀层，防止机器人边缘碰撞。
- **Global Costmap (全局地图)**:
    - **尺寸**: 覆盖整个比赛场地。
    - **插件层**:
        1. `static_layer`: 基础静态地图。
        2. `IntensityVoxelLayer`: 处理 `terrain_map_ext`，提供全局避障能力。
        3. `inflation_layer`: 全局膨胀。
- **通用参数**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | inflation_radius | 仿真 0.90 / 实车 0.61 | 膨胀半径（米），按环境配置保留安全裕量 |
    | cost_scaling_factor | 8.0 | 代价随距离衰减的指数因子 |

---

## 第4章: 路径规划与运动控制

### 4.1 全局规划器 - SmacPlannerHybrid
仿真和实车 `planner_plugins: ["GridBased"]` 均使用 `nav2_smac_planner::SmacPlannerHybrid`，并且都采用 `motion_model_for_search: "DUBIN"` 只规划前进弧线。仿真和实车 MPPI 的 `vx_min=0.0` 都是前向优先，DUBIN 禁止生成 cusp/倒车段路径，与 MPPI 采样空间保持语义一致；REEDS_SHEPP 会生成反向原语，当前 MPPI 链路无法跟随。Hybrid A* 在朝向维度量化的栅格上搜索弧线路径，规划输出为带朝向的平滑路径，仿真与实车均交给 MPPI DiffDrive 跟随。

- **配置参数（仿真与实车大部分对齐，仅 `reverse_penalty` 细节按 YAML 历史值保留）**:
    | 参数名 | 仿真 | 实车 | 说明 |
    |--------|--------|--------|------|
    | motion_model_for_search | DUBIN | DUBIN | 两端都前向优先；MPPI `vx_min=0` 禁止倒车采样，DUBIN 禁止生成后退/cusp 路径 |
    | reverse_penalty | 2.1 | 2.1 | DUBIN 不会产生倒车原语，此参数只作保留占位 |
    | angle_quantization_bins | 72 | 72 | 航向量化 5° 一档 |
    | minimum_turning_radius | 0.25 | 0.25 | 最小转弯半径 (m)，差速可取较小值 |
    | tolerance | 0.5 | 0.5 | 目标附近搜索容差 (m) |
    | allow_unknown | true | true | 未知区域是否可通行（SLAM 早期很有用） |
    | cost_penalty | 2.0 | 2.0 | 靠近高代价区域的惩罚，用于居中 |
    | max_iterations | 1000000 | 1000000 | 最大搜索迭代次数 |
    | max_planning_time | 1.5 | 1.5 | 单次规划最长耗时 (s) |
    | smooth_path | true | true | 启用后置平滑 |
    | smoother.max_iterations | 1000 | 1000 | 平滑迭代次数 |

> 备选方案：若 SmacPlannerHybrid 在窄过道规划失败率过高，可退到 SmacPlanner2D（无运动学约束、速度更快）作为临时措施，但需要同步验证下游控制器的跟随表现。C/D 阶段计划引入语义 Route Graph 与 Smac Lattice 作为全局规划层的能力升级，两者都未实装。

### 4.2 局部控制器（仿真 + 实车首版均为 MPPI DiffDrive）

2026 赛季 A 阶段把仿真的局部控制器从 Nav2 官方差速默认组合迁移到单一 MPPI 控制器；本次 Phase R-A 把实车 YAML 也切到同一控制器语义下的"首版前向优先"版本。两套配置文件分别位于：

- 仿真：`src/sentry_nav_bringup/config/simulation/nav2_params.yaml`（Phase A，当前稳定版本）
- 实车：`src/sentry_nav_bringup/config/reality/nav2_params.yaml`（Phase R-A 首版，YAML 已落地，上场/全场回归依赖 Task 6 台架和实测 smoke）

行为树中 `FollowPath` 这个 `controller_id` 没有改名，仿真和实车都直接指向 MPPI 插件。运行时没有 MPPI → RPP 的 fallback；回退只能通过 git 恢复旧配置，属于配置/部署层面的动作。若实车 CPU 顶不住 30Hz，fallback 策略是**降 MPPI `batch_size`**（`2000 → 1500 → 1000`），不允许动 `controller_frequency / smoothing_frequency`，更不允许动 `robot_radius=0.318`。

#### 4.2.1 仿真局部控制器 - MPPI DiffDrive (Phase A 已落地)

`controller_plugins: ["FollowPath"]`，`FollowPath.plugin: "nav2_mppi_controller::MPPIController"`，`motion_model: "DiffDrive"`。MPPI 基于模型预测采样，每次在当前速度附近采样一组候选轨迹，用 critic 集合打分后按指数权重加权得到最终指令。

- **关键配置**:
    | 参数 | 当前值 | 说明 |
    |---|---|---|
    | `motion_model` | `DiffDrive` | 固定差速模型 |
    | `time_steps` | 32 | 预测步数 |
    | `model_dt` | 0.05 | 单步时长 (s) |
    | `vx_max` | 1.5 | 前向速度上限 (m/s)，与 velocity_smoother 对齐 |
    | `vx_min` | 0.0 | 今日禁止倒车 |
    | `vy_max` | 0.0 | 差速底盘锁侧向 |
    | `vy_std` | 0.0 | 采样噪声的侧向标准差锁 0，防止 vy 漂出 |
    | `wz_max` | 6.3 | 角速度上限 (rad/s) |
    | `ax_max` | 1.5 | 线加速度上限 (m/s²) |
    | `az_max` | 8.0 | 角加速度上限 (rad/s²) |
    | `batch_size` | 2000 | 采样批量，CPU 保守值 |
    | `iteration_count` | 1 | 单次迭代 |
    | `visualize` | false | 关闭轨迹可视化，仿真 RTF 优先 |
    | `regenerate_noises` | false | 复用噪声，减少算力 |
- **Critic 列表**: `ConstraintCritic`, `CostCritic`, `GoalCritic`, `GoalAngleCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`, `PreferForwardCritic`（Nav2 Jazzy 官方差速推荐集）。
- **Horizon 安全约束（硬规则）**: `time_steps × model_dt × vx_max < local_costmap_half_width`。当前 `32 × 0.05 × 1.5 = 2.4m`，local costmap 半径 2.5m，留 0.1m 裕量。调整任意一个参数都必须重新核对此不等式，否则 MPPI 轨迹末端会落到 costmap 之外无代价评估。
- **频率约束**: `controller_frequency` 与 `velocity_smoother.smoothing_frequency` 必须相等（当前仿真均为 20Hz）。

#### 4.2.2 实车局部控制器 - MPPI DiffDrive 首版（Phase R-A，YAML 已落地，上场验证依赖 Task 6）

`controller_plugins: ["FollowPath"]`，`FollowPath.plugin: "nav2_mppi_controller::MPPIController"`，`motion_model: "DiffDrive"`，与仿真共用控制器语义。首版定位是**前向优先、最小可上场的安全版本**：禁止倒车采样，角速度/角加速度上限被 velocity_smoother 收紧到物理可执行范围，全场/上场 Nav Goal smoke 依赖后续 Task 6 台架测试。

- **关键配置（实车当前值，与仿真差异项已标出）**:
    | 参数 | 实车值 | 与仿真差异 | 说明 |
    |---|---|---|---|
    | `motion_model` | `DiffDrive` | 一致 | 固定差速模型 |
    | `time_steps` | 32 | 一致 | 预测步数 |
    | `model_dt` | 0.05 | 一致 | 单步时长 (s) |
    | `vx_max` | 1.5 | 一致 | 前向速度上限，与实车 smoother `max_velocity[0]=1.5` 对齐 |
    | `vx_min` | **0.0** | 一致 | **首版前向优先**，禁止倒车采样，必须与 Planner DUBIN 配对 |
    | `vy_max` | 0.0 | 一致 | 差速底盘锁侧向 |
    | `vy_std` | 0.0 | 一致 | 采样噪声侧向标准差锁 0 |
    | `wz_max` | **3.0** | 仿真 6.3 | **受实车 smoother `max_velocity[2]=3.0` 约束**，不能直接套仿真值 |
    | `ax_max` | 1.5 | 一致 | 线加速度上限 (m/s²) |
    | `az_max` | **5.0** | 仿真 8.0 | **受实车 smoother `max_accel[2]=5.0` 约束**，不能直接套仿真值 |
    | `batch_size` | 2000 | 一致 | CPU 降频 fallback 起点，不够时顺序降到 1500 → 1000 |
    | `iteration_count` | 1 | 一致 | 单次迭代 |
    | `visualize` | false | 一致 | 关闭轨迹可视化，省 CPU |
    | `regenerate_noises` | false | 一致 | 复用噪声，减少算力 |
- **Horizon 安全约束（硬规则）**: 与仿真相同，`32 × 0.05 × 1.5 = 2.4m < 2.5m`，裕量 0.1m。任何参数变大都必须手算一次。
- **频率约束**: `controller_frequency = 30Hz`，`velocity_smoother.smoothing_frequency = 30Hz`，**两者必须保持相等**。首版不允许提频（避免 CPU 波动把频率打散），不允许降频（避免控制带宽不足）。只能降 `batch_size`。
- **Planner 语义对齐**: `planner_server.GridBased.motion_model_for_search: "DUBIN"`（与 `vx_min=0` 配对），不能回到 `REEDS_SHEPP`，否则 Planner 会生成 MPPI 无法跟随的倒车/cusp 路径。
- **`robot_radius=0.318` 固定不变**: 首版上场前的台架验证必须用此值完成物理外形确认，修改需要重新确认整套 inflation/cost_scaling 的安全裕度。
- **无 runtime RPP 回退**: Phase R-A 与 Phase A 同策略，回滚只能从 git 恢复旧 `["RotateShim", "FollowPath"]` + `REEDS_SHEPP` 的 YAML。

### 4.3 Velocity Smoother
- **功能**: 位于控制器和驱动器之间，负责对速度指令进行二次平滑，防止加速度过大导致底盘打滑或机械受损。
- **差速约束**: 仿真 `max_velocity: [1.5, 0.0, 6.3]`、`min_velocity: [-1.5, 0.0, -6.3]`；实车 `max_velocity: [1.5, 0.0, 3.0]`、`min_velocity: [-1.5, 0.0, -3.0]`。两端 **vy 锁 0**，保证即使上游输出异常 vy 也不会下发到差速执行器。
- **加速度限制**:
    - 仿真 `max_accel: [1.5, 0.0, 8.0]`、`max_decel: [-1.5, 0.0, -8.0]`
    - 实车 `max_accel: [2.5, 0.0, 5.0]`、`max_decel: [-2.5, 0.0, -5.0]`（实车 28kg 双电机物理上限约 2.86 m/s²，取 2.5 留余量）
- **频率**：必须与 `controller_frequency` 一致（仿真 20Hz，实车 30Hz）。
- **输出**：`cmd_vel_smoothed` remap 到 `cmd_vel_nav`，作为 `sentry_motion_manager` 的 navigation 输入。

### 4.4 sentry_motion_manager 速度仲裁
差速底盘 `chassis_yaw ≡ base_footprint_yaw`，velocity_smoother 输出的 TwistStamped 在 `base_footprint` 系即等同于 `chassis` 本体系，先进入 `cmd_vel_nav`，再由 `sentry_motion_manager` 发布最终 `/cmd_vel` 给 Gazebo DiffDrive 插件 / rm_serial_driver 消费。**无需坐标旋转节点，无自旋叠加；下游底盘消费者话题保持不变。**

### 4.5 分阶段导航路线图

2026 赛季把导航栈升级拆成四个阶段，避免规划器、控制器、地形语义同时变更导致失败不可追踪。以下为当前状态：

| 阶段 | 主题 | 状态 | 今日配置 |
|---|---|---|---|
| A | MPPI-only 局部控制器 | **仿真落地（Phase A） + 实车首版 YAML 落地（Phase R-A，上场回归待 Task 6）** | 仿真/实车均 `FollowPath = MPPIController (DiffDrive)`；实车首版 `vx_min=0.0 / wz_max=3.0 / az_max=5.0 / DUBIN`，30Hz，CPU 降频仅靠降 `batch_size` |
| B | 现有地形 / 代价地图语义增强 | **本轮审查完成，无 YAML 数值变更** | 审查结论：保持现有 `terrain_analysis` / `terrain_analysis_ext` / `IntensityVoxelLayer` 参数（`clearDyObs=False`、`robot_radius=0.46 / inflation_radius=0.90`、低矮障碍链路 `preprocess.blind=0.35 / min_obstacle_intensity=0.05 / minBlockPointNum=5`），Point-LIO `gravity/blind/range` 不动；不引入新插件 |
| C | 语义 Route Graph | **未实装** | 需 A+B 通过仿真 rmuc_2026 窄道/高低路线 smoke 和 MPPI 频率稳定后再启动 |
| D | Smac Lattice / ConstrainedSmoother / MINCO（MINCO 为可选工具，非必需） | **未实装** | 必须先完成 C 阶段语义 Route Graph 评估，确认需要 Lattice / 新 smoother / MINCO 才能解决的具体场景后再引入 |

- **长期规划器今日不动**：全局规划器继续使用 `nav2_smac_planner::SmacPlannerHybrid`（仿真和实车都用 `DUBIN`，两端 `minimum_turning_radius: 0.25`）。C/D 的 Route Graph / Lattice / MINCO / ConstrainedSmoother 都只是未来门控目标，本次迁移不做任何长期规划器替换；任何把它们写成"已实装"或"默认链路"的文档都算违规。
- **回退策略**：A / R-A 阶段没有 MPPI → RPP 的 runtime fallback；回滚通过 git/config 恢复旧 RPP 配置完成。这是刻意选择，避免双套控制器共存时参数和行为树耦合复杂化。实车 CPU 降频时只允许降 MPPI `batch_size`（`2000 → 1500 → 1000`），不允许动 `controller_frequency / smoothing_frequency`，也不允许动 `robot_radius=0.318`。
- **诊断优先**：阶段推进的门控条件是"当前阶段能复现稳定 QA 证据"，不是"跑通一次 demo"。C/D 文档工作量较大且不可回退，一旦启动再回到 A/B 成本很高，必须先积累窄道/坡道/碰撞恢复等场景的量化数据。
- **质量标准**：阶段推进看的是可观测性、可重复 QA、可安全回滚、路径语义是否稳定，不是算法复杂度。C/D 不是"换更大模型"，是"在 A/B 已经可复现稳定的基础上补语义"。

#### 4.5.1 C 阶段启动门控（必须同时满足）

- [ ] 仿真 rmuc_2026 窄道（两侧障碍）路线 smoke 连续 3 次 `navigate_to_pose` 返回 `SUCCEEDED`，无 `Costmap timed out waiting for update` 或 planner abort。
- [ ] 仿真 rmuc_2026 高低路线（坡道 / 台阶 / 资源岛）路线 smoke 连续 3 次 `navigate_to_pose` 返回 `SUCCEEDED`，地形代价语义不将可通行坡面标为致死。
- [ ] 控制器输出频率稳定在 `controller_frequency` 目标值（仿真 20Hz）附近，`ros2 topic hz /red_standard_robot1/cmd_vel_nav` 无秒级空档；Nav2 实测跑得到的频率不能低于设定 70%。
- [ ] `slam:=False` 导航所需先验资源齐备：`install/sentry_nav_bringup/share/sentry_nav_bringup/map/simulation/rmuc_2026.yaml` 能被 `map_server` 成功 load，`install/sentry_nav_bringup/share/sentry_nav_bringup/pcd/simulation/rmuc_2026.pcd` 能被 `small_gicp_relocalization` 成功 load。**当前 Task 5 smoke 的 `slam:=False` 仍被这两项资源缺失 + `Costmap timed out waiting for update` 阻塞，因此 C 阶段尚未具备启动条件。**
- [ ] `vy_max=vy_std=0`、`vx_min=0`、`enable_stamped_cmd_vel: true` 三处硬锁在上述所有 smoke 中全程保持。

#### 4.5.2 D 阶段启动门控（必须同时满足）

- [ ] C 阶段语义 Route Graph 在仿真 rmuc_2026 完整一圈巡逻里至少 1 小时运行，路径语义稳定，不出现跨区段跳边或死锁。
- [ ] 在 C 阶段稳定后，能够用具体数据证明"MPPI + 现有 costmap + 语义 Route Graph"依然无法解决某一类场景（例如特定窄弯的跟随误差、低速倒车贴障、无法保证的最小转弯半径下规划失败率），且该场景必须通过 Smac Lattice、ConstrainedSmoother 或 MINCO 之一才能收敛。
- [ ] D 阶段候选算法有明确落地路径：Smac Lattice 为 Nav2 官方全局规划器插件，ConstrainedSmoother 为 Nav2 官方 smoother 插件，MINCO 作为可选轨迹优化/smoother 工具（不是必需）。任何一项都必须先在独立分支做单独可回滚的替换，不允许三者同时上。
- [ ] D 阶段不允许为了"显得企业级"而引入：必须有 C 阶段之后遗留问题作为启动依据。

---

## 第5章: 行为树决策系统

### 5.1 框架概述
系统采用 `BehaviorTree.CPP` 4.x 版本，通过 XML 文件定义复杂的战术逻辑。决策系统通过订阅裁判系统、视觉系统和导航系统的状态，实时切换机器人的行为模式。

### 5.2 自定义插件详细列表

#### 条件节点 (Condition Nodes)
| 节点名 | 功能描述 | 关键输入端口 |
|--------|----------|--------------|
| IsGameStatus | 检查当前比赛阶段（如：准备、开始、结算） | expected_game_progress, min_remain_time |
| IsStatusOK | 检查机器人自身状态（血量、热量、弹药） | hp_min, heat_max(350-400), hp_min(300-400) |
| IsRfidDetected | 检测是否处于特定的 RFID 增益区域 | rfid_type |
| IsAttacked | 判断是否受到敌方攻击及受击方向 | - |
| IsDetectEnemy | 视觉系统是否锁定敌方目标 | armor_id_list, max_distance |
| IsHPAdd | 是否正在补血 | key_port(RobotStatus) |
| IsOutpostOk | 检查己方前哨站是否存活 | key_port(GameRobotHP) |

#### 动作节点 (Action Nodes)
| 节点名 | 功能描述 | 关键端口 |
|--------|----------|--------------|
| PubGoal | 向导航栈发布一个新的目标点 | goal_pose_x, goal_pose_y, topic_name |
| BattlefieldInformation | 分析全场血量对比，输出进攻/防守权重 | key_port(GameRobotHP) -> weight |
| Pursuit | 追踪行为，使云台指向目标并配合底盘移动 | key_port(tracker_target), topic_name |

#### 控制与装饰节点 (Control & Decorator Nodes)
| 节点名 | 功能描述 | 关键端口 |
|--------|----------|--------------|
| RecoveryNode | 失败恢复控制，指定重试次数 | num_attempts (默认 999) |
| RateController | 控制子树的运行频率 | hz (默认 10) |
| TickAfterTimeout | 超时后触发 tick | timeout (秒) |

### 5.3 核心策略逻辑

#### RMUL (3v3 策略) - rmul.xml
- **基础巡逻 (`rmul`)**: 
    - **逻辑流**: 
        1. 初始状态：机器人启动后，首先进入 `RateController` 装饰的子树，以 10Hz 频率运行。
        2. 目标发布：通过 `PubGoal` 节点发布中心点巡逻目标。
        3. 状态检测：实时调用 `IsStatusOK` 检查自身血量。若血量低于 200，立即触发回撤逻辑。
        4. 回撤补给：前往补给点 (0, -0.5)，并利用 `TickAfterTimeout` 节点在补给区停留 10 秒以确保补血完成。
        5. 进攻逻辑：若血量充足且 `IsDetectEnemy` 返回成功，切换至进攻点 (4.6, -3.3) 进行火力压制。
- **双点巡逻 (`rmul1`)**: 
    - **逻辑流**: 在两个预设的进攻点之间循环移动。使用 `Sequence` 节点连接两个 `PubGoal` 动作，配合 `RecoveryNode` 确保在导航失败时能够自动重试。
- **激进策略 (`rmul2`)**: 
    - **逻辑流**: 专门针对高血量状态设计的策略。当 `IsStatusOK` 检测到血量高于 300 时，机器人会越过中线，深入敌方半场进行骚扰，依靠高速机动吸引敌方火力。

#### RMUC (对抗赛策略) - RMUC.xml
- **多阶段切换**:
    - **阶段 1 (开局 - down_1)**: 重点在于资源争夺。机器人会快速占领附近的 RFID 增益点，并进行多点区域巡逻。同时持续监测 `IsRfidDetected`，一旦进入补给区且血量不满，则停止移动进行补给。
    - **阶段 2 (中期 - down_2)**: 侧重于阵地防御。若 `IsOutpostOk` 返回成功，机器人会驻守在前哨站附近的战略要点，利用地形优势进行防守。
- **战术分支**: 
    - **rmuc_yes**: 当己方前哨站存活时的详细战术。结合 `BattlefieldInformation` 节点输出的权重，如果己方血量占优，则主动出击；否则保持龟缩防守。
    - **rmuc_no**: 前哨站被毁后的紧急战术。机器人转入全场游走模式，优先寻找掩体，并尝试偷袭敌方落单目标。

### 5.4 Nav2 导航行为树
系统重写了 Nav2 的默认行为树 `navigate_to_pose_w_replanning_and_recovery.xml`：
- **频率控制**: 路径重规划频率限制在 5Hz，平衡计算开销与实时性。
- **恢复策略**: 旧 `<Spin/>` + `<BackUp/>` 主恢复路径已从默认 BT 中移除；当前只保留清图+等待的 `MotionManagerRecoveryPlaceholder`，确保 BT 可解析，后续由 `sentry_motion_manager` 的 Nav2 recovery 适配器接管贴墙脱困触发。
- **重试机制**: `RecoveryNode` 设置为最多重试 10 次，防止陷入死循环。

---

## 第6章: 通信接口

### 6.1 裁判系统消息 (rm_interfaces)
这是机器人获取外界信息的最高优先级接口，所有战术决策都依赖于此。
- `GameStatus`: 
    - `game_progress`: 1 (准备阶段), 2 (15s准备), 3 (5s倒计时), 4 (比赛中), 5 (结算)。
    - `stage_remain_time`: 当前阶段剩余秒数，用于 `IsGameStatus` 节点判断是否需要回撤。
- `RobotStatus`: 
    - `remain_hp`: 实时剩余血量。
    - `max_hp`: 机器人最大血量。
    - `muzzle_heat`: 枪口实时热量，用于控制射击频率。
    - `ammo_count`: 剩余弹药量，若低于 `ammo_min` 则触发补给逻辑。
- `RfidStatus`: 
    - 包含 `friendly_supply_zone_exchange`, `center_gain_point`, `friendly_fortress_gain_point` 等布尔值，实时反映机器人是否处于增益区。
- `GameRobotHP`: 
    - 实时同步红蓝双方所有 14 台机器人的血量。`BattlefieldInformation` 节点通过计算双方总血量差值来评估战场态势。

### 6.2 视觉接口 (rm_interfaces/msg/vision/)
- `Armors`: 
    - 视觉算法识别出的所有装甲板在相机坐标系下的位姿。
    - 包含 `armor_id` 和 `distance`，用于 `IsDetectEnemy` 节点过滤目标。
- `Target`: 
    - 经过卡尔曼滤波后的目标预测位姿。
    - 包含 `position`, `velocity` 和 `acceleration`。`Pursuit` 节点利用这些数据计算云台的提前量补偿，并引导底盘向目标靠近。

### 6.3 串口通信 (rm_serial_driver)
- **下行指令 (ROS2 -> MCU)**: 
    - 将 `/cmd_vel` (TwistStamped, base_footprint 系) 打包为 Nav 包（`vel_x`, `vel_w`）发送至 STM32。差速底盘无 vy，不下发。
    - 云台控制由视觉/自动瞄准节点直接写入下位机，不经过 Nav2。
- **上行数据 (MCU -> ROS2)**: 
    - 接收 IMU 原始数据、裁判系统透传字节流，解析为 ROS2 消息。
    - 接收下位机上报的 `chassis_yaw/pitch`、`gimbal_yaw/pitch` 四路姿态。

---

## 第7章: 仿真系统

### 7.1 仿真环境构建
系统基于 Gazebo Harmonic 构建了 1:1 的比赛场地仿真，为算法验证提供了安全、高效的环境。
- **世界文件 (`.sdf`)**: 
    - `rmul_2026_world.sdf`: 包含完整的 2026 赛季场地模型。
    - 物理引擎参数经过调优，模拟了真实的地面摩擦力和碰撞特性。
- **传感器模拟**: 
    - **Livox Mid360**: 模拟了其独特的非重复扫描特性，生成的点云具有与实车一致的稀疏度和分布。
    - **IMU**: 模拟了高频噪声和零偏漂移，用于测试 Point-LIO 的鲁棒性。
- **桥接器 (`ros_gz_bridge`)**: 
    - 负责处理 Gazebo 内部话题与 ROS2 话题的映射。例如，将仿真中的 `/model/sentry/odometry` 映射为 `/odom`。

### 7.2 仿真与实车切换
系统通过 launch 文件中的 `use_sim_time` 参数进行无缝切换，确保同一套代码和参数可以在两个环境运行。
- **仿真启动**: `ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py`
- **关键差异处理**: 
    - 仿真中点云较为稀疏，因此 `Small GICP` 的 `max_dist_sq` 参数会自动调大。
    - 仿真中不涉及真实的串口通信，而是通过 `ros_gz_bridge` 直接控制仿真模型。

### 7.3 调试与可视化工具
- **RViz2**: 
    - 预设了完整的配置文件，可直观查看代价地图、全局/局部路径、实时点云、TF 树以及重定位状态。
- **Groot2**: 
    - 通过 ZMQ 协议连接到运行中的行为树服务器。
    - 实时显示行为树的执行路径、节点状态（Success/Failure/Running）以及黑板变量的动态变化。
- **Wayland 适配**: 
    - 在 Ubuntu 24.04 的 Wayland 桌面环境下，Gazebo GUI 可能会出现无响应。需在启动前设置环境变量 `export QT_QPA_PLATFORM=xcb`。

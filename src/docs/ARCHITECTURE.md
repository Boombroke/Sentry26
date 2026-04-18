# 哨兵导航系统架构详解

> **Maintainer**: Boombroke <boombroke@icloud.com>
> **基于**: [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) by Lihan Chen，2026 差速轮足架构迁移由 Boombroke 主导
> 相关文档：[快速部署](QUICKSTART.md) · [运行模式](RUNNING_MODES.md) · [参数调优](TUNING_GUIDE.md)

## 第1章: 系统总览

### 1.1 项目背景与功能描述
本项目是为 RoboMaster 哨兵机器人量身定制的自主导航系统。哨兵机器人在比赛中承担着全自动防御和进攻的核心任务，要求系统具备极高的定位精度、环境感知能力以及复杂的战术决策逻辑。本系统集成了从底层传感器驱动到高层行为树决策的全栈功能，确保机器人在动态变化的赛场环境中能够稳定运行。

系统核心目标包括：
- **高精度定位**: 在剧烈运动和碰撞中保持厘米级的定位精度。
- **差速轮足控制**: 基于差速轮足底盘（仅 vx/wz 自由度，无侧向 vy），结合 Nav2 RPP + RotationShim 控制器实现高动态路径跟随。
- **智能战术决策**: 根据裁判系统实时数据，自动切换进攻、防守、补给等状态。
- **环境自适应**: 能够识别坡道、台阶等复杂地形，并生成准确的代价地图。

### 1.2 系统数据流图
以下展示了从原始传感器数据输入到最终底盘执行器输出的完整数据通路。系统采用模块化设计，各组件通过 ROS2 话题和服务进行解耦。

```text
+-----------------------+      +-----------------------+      +-----------------------+
|       感知层 (Perception)      |       定位层 (Localization)    |       规划层 (Planning)       |
+-----------+-----------+      +-----------+-----------+      +-----------+-----------+
            |                              |                              |
[Livox Mid360] ----+------> [Point-LIO 里程计] ----+------> [Nav2 全局规划器]
                   |        (lidar_odom -> lidar)  |        (SmacPlanner2D)
                   |               |               |               |
[IMU (6-Axis)] ----+               v               |               v
                                   |               |        [Nav2 局部控制器]
[先验 PCD 地图] ----------> [Small GICP 重定位] ----+        (RotationShim → RPP)
                           (map -> odom 修正)      |               |
                                   |               |               v
[视觉识别 (Armors)] ------> [BehaviorTree.CPP] <---+        [Velocity Smoother]
                           (高层战术决策)          |        (vy 锁 0, TwistStamped)
                                   ^               |               |
[裁判系统 (Referee)] ------+       |               |               v
                           |       |               |        [底盘执行器接口]
[底盘反馈 (Odom)] ---------+-------+               |        (/cmd_vel, base_footprint 系)
                                                   |               |
                                                   |               v
                                                   |        Gazebo DiffDrive / rm_serial_driver
                                                   +------> [云台独立接口]
                                                            (下位机上报 yaw/pitch)
```

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
Point-LIO 是系统的核心里程计方案，通过紧耦合 IMU 和激光雷达数据，提供高频、低延迟的位姿估计。

- **功能**: 实现激光惯性里程计，输出机器人相对于起始点的位姿。
- **输入话题**:
    - `livox/lidar` (sensor_msgs/msg/PointCloud2): 原始点云数据。
    - `livox/imu` (sensor_msgs/msg/Imu): 原始惯性数据。
- **输出话题**:
    - `aft_mapped_to_init` (nav_msgs/msg/Odometry): 优化后的里程计位姿。
    - `cloud_registered` (sensor_msgs/msg/PointCloud2): 投影到世界坐标系下的实时点云。
- **坐标变换 (TF)**: 发布 `lidar_odom -> front_mid360`。
- **仿真适配**: 在仿真环境中，通过 `ign_sim_pointcloud_tool` 将 Gazebo 的 PointCloudPacked 格式转换为 `velodyne_points` 格式，以便 Point-LIO 处理。
- **关键参数配置**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | lidar_type | 2 | 适配 Velodyne 格式的点云输入 |
    | filter_size_surf | 0.5 | 表面特征点云降采样步长 |
    | filter_size_map | 0.5 | 地图点云降采样步长 |
    | gravity | [4.905, 0, -8.496] (仿真) | 重力向量，需根据雷达安装倾角微调 |

- 仿真 wheeled_biped 的 Mid360 相对 `gimbal_pitch` 额外固定下俯约 30°，用于覆盖近场低矮底座；因此 Point-LIO 的仿真 `gravity` 必须同步为前向分量非零。

### 2.2 Odom Bridge (里程计桥接)

`odom_bridge` 合并了原 `loam_interface` 与 `sensor_scan_generation`，在单次同步回调中完成完整链路：

**输入**：Point-LIO 的 `aft_mapped_to_init`（Odometry）+ `cloud_registered`（PointCloud2），均在 lidar_odom 坐标系。

**处理流程**：
1. 一次性标定 `base_frame → lidar_frame` 静态变换，得到 `odom ↔ lidar_odom` 的旋转偏移
2. 将 Point-LIO 输出从 lidar_odom 系变换到 odom 系
3. 对 `odom → chassis` 施加 2D 约束（z=0, roll=0, pitch=0），防止 Z 漂移
4. 通过位置差分计算线速度和角速度

**输出**：
- TF: `odom → base_footprint`
- `odometry`（odom → gimbal_yaw，含差分速度）
- `sensor_scan`（lidar 系点云）
- `registered_scan`（odom 系点云，供 terrain_analysis/ext）
- `lidar_odometry`（odom 系里程计，供 terrain_analysis/ext）

### 2.3 Small GICP 重定位
- **功能**: 解决里程计累积漂移问题。通过将实时点云与预先构建的 PCD 地图进行配准，计算 `map -> odom` 的修正量。
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
系统维护的完整 TF 树（6 层，差速轮足架构）如下：

```text
map (全局地图坐标系)
 └─ odom (里程计坐标系，由 small_gicp 修正)
     └─ base_footprint (Nav2 规划坐标系，水平 2D 投影)
         └─ chassis (底盘刚体原点，yaw ≡ base_footprint)
             └─ gimbal_yaw (云台偏航角，下位机上报)
                 └─ gimbal_pitch (云台俯仰角，下位机上报)
                     └─ front_mid360 (Livox 激光雷达中心，随云台旋转)
```

- `base_footprint` 与 `chassis` 为同一刚体在不同投影层级，yaw 一致，仅 z 抬升。
- 云台链独立于底盘运动，由 `joint_state_publisher` 按下位机上报的 `gimbal_yaw/pitch` 角度持续发布。
- LiDAR 挂在 `gimbal_pitch` 上随云台旋转，`odom_bridge` 通过每帧 `lookupTransform(lidar_frame → base_frame)` 消化云台旋转对底盘位姿估计的影响。

---

## 第3章: 地形分析与代价地图

### 3.1 Terrain Analysis
- **功能**: 将 3D 点云转化为可用于导航的 2.5D 地形信息。它能够识别地面、台阶、斜坡以及障碍物。
- **输出话题**: `terrain_map` (sensor_msgs/msg/PointCloud2)
- **关键参数**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | vehicleHeight | 0.6 | 机器人自身高度，用于过滤上方遮挡 |
    | minRelZ | -0.1 | 相对地面的最小高度过滤 |
    | maxRelZ | 0.5 | 相对地面的最大高度过滤 |
    | terrainVoxelSize | 0.2 | 地形体素化分辨率 |
    | useSorting | true | 启用排序算法以支持斜坡识别 |
    | clearDyObs | true | 尝试清除动态障碍物（如其他机器人） |

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
    | inflation_radius | 0.7 | 膨胀半径（米） |
    | cost_scaling_factor | 4.0 | 代价随距离衰减的指数因子 |

---

## 第4章: 路径规划与运动控制

### 4.1 全局规划器 - SmacPlanner2D
差速轮足方案保留 SmacPlanner2D（8 方向无运动学约束栅格搜索）。原因：差速底盘可原地旋转 + RotationShim 处理大转角，2D 规划器生成的折线路径足以驱动。

- **配置参数**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | tolerance | 0.5 | 目标附近搜索容差 (m) |
    | allow_unknown | true | 未知区域是否可通行 |
    | cost_travel_multiplier | 3.0 | 路径代价权重（鼓励走通道中央） |
    | smoother.max_iterations | 1000 | 路径平滑迭代次数 |

> 备选方案：若 SmacPlanner2D 在窄过道急转时不够理想，可切换到 SmacPlannerHybrid（Dubin/Reeds-Shepp 运动模型）；但差速 + RotationShim 已能应对绝大多数场景。

### 4.2 局部控制器 - RotationShimController + RegulatedPurePursuitController
Nav2 官方差速默认组合。两者通过 `controller_plugins: ["RotateShim", "FollowPath"]` 级联：

1. **RotationShim**（大转角原地旋转）：检测当前朝向与路径起始切线方向的夹角，大于 `angular_dist_threshold`（如 45°, 0.785 rad）时先原地旋转至大致对齐，再把控制权交给主控制器。
2. **RegulatedPurePursuitController (RPP)**（路径跟随）：沿路径选取 lookahead 点，根据曲率与接近目标距离自动调整线速度，支持 `use_rotate_to_heading` 做终端对齐。

- **核心逻辑（RPP）**:
    1. 沿全局路径按 `lookahead_dist` 选前瞻点；`use_velocity_scaled_lookahead_dist: true` 时 lookahead 随当前线速度缩放，被 `[min_lookahead_dist, max_lookahead_dist]` 限幅。
    2. 根据前瞻点相对底盘的角度计算弧线曲率，折算为 `wz = vx * curvature`。
    3. `use_regulated_linear_velocity_scaling: true` 会在高曲率段自动降速（`regulated_linear_scaling_min_radius`、`regulated_linear_scaling_min_speed`）。
    4. 接近目标时按 `approach_velocity_scaling_dist` 平滑减速；`use_rotate_to_heading: true` 时在目标附近先对齐朝向再停车。
- **关键参数（RPP）**:
    | 参数名 | 仿真默认 | 实车默认 | 说明 |
    |--------|--------|--------|------|
    | desired_linear_vel | 2.0 | 1.0 | 期望线速度 (m/s) |
    | lookahead_dist | 1.2 | 1.2 | 基础前瞻距离 (m) |
    | min_lookahead_dist | 0.6 | 0.6 | 最小前瞻距离 (m) |
    | max_lookahead_dist | 1.5 | 1.5 | 最大前瞻距离 (m) |
    | lookahead_time | 1.0 | 1.0 | velocity-scaled lookahead 的时间常数 |
    | regulated_linear_scaling_min_radius | 0.5 | 0.5 | 高曲率降速触发半径 |
    | regulated_linear_scaling_min_speed | 0.3 | 0.3 | 高曲率降速的速度下限 |
    | use_rotate_to_heading | true | true | 是否在终端对齐目标朝向 |
    | rotate_to_heading_min_angle | 0.5 | 0.5 | 触发 rotate-to-heading 的最小角度 (rad) |
    | allow_reversing | false | false | 差速不倒车 |
- **关键参数（RotationShim）**:
    | 参数名 | 默认值 | 说明 |
    |--------|--------|------|
    | angular_dist_threshold | 0.785 | 45° 以上先原地旋转 |
    | forward_sampling_distance | 0.5 | 采样距离，用于判定路径方向 |
    | rotate_to_heading_angular_vel | 1.5 | 原地旋转角速度 (rad/s) |

### 4.3 Velocity Smoother
- **功能**: 位于控制器和驱动器之间，负责对速度指令进行二次平滑，防止加速度过大导致底盘打滑或机械受损。
- **差速约束**: `max_velocity: [2.5, 0.0, 3.0]`、`min_velocity: [-2.5, 0.0, -3.0]` —— **vy 锁 0**，保证即使上游输出异常 vy 也不会下发到差速执行器。
- **加速度限制**:
    - `max_accel_x`: 4.5 m/s²
    - `max_accel_yaw`: 5.0 rad/s²
- **频率**：必须与 `controller_frequency` 一致（仿真 20Hz，实车 30Hz）。

### 4.4 cmd_vel 直通（无坐标旋转）
差速底盘 `chassis_yaw ≡ base_footprint_yaw`，velocity_smoother 输出的 TwistStamped 在 `base_footprint` 系即等同于 `chassis` 本体系，直接 remap 到 `/cmd_vel` 被 Gazebo DiffDrive 插件 / rm_serial_driver 消费。**无需坐标旋转节点，无自旋叠加。**

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
- **频率控制**: 路径重规划频率限制在 3Hz，平衡计算开销与实时性。
- **恢复策略**: 当路径被阻挡时，依次尝试：
    1. `ClearLocalCostmap`: 清除局部代价地图。
    2. `ClearGlobalCostmap`: 清除全局代价地图。
    3. `BackUp`: 后退一段距离。
    4. `Spin`: 原地旋转以重新观测环境。
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

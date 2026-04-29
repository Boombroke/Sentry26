# Point-LIO 与导航参数调优指南

> **Maintainer**: Boombroke <boombroke@icloud.com>
> 相关文档：[快速部署](QUICKSTART.md) · [架构详解](ARCHITECTURE.md) · [运行模式](RUNNING_MODES.md)

本文档列出了需要在实车上根据实际情况微调的参数，以及对应的调优方法。

配置文件位置：
- 仿真：`sentry_nav_bringup/config/simulation/nav2_params.yaml`（Phase A 已迁移到 MPPI DiffDrive）
- 实车：`sentry_nav_bringup/config/reality/nav2_params.yaml`（仍使用 RotationShim + RPP，未迁移）

> 今日范围：Phase A 仿真 MPPI 已落地；Phase B 地形 / 代价地图语义审查本轮已完成，结论是保持现有 YAML 参数（`clearDyObs=False`、`robot_radius=0.46 / inflation_radius=0.90`、低矮障碍链路 `preprocess.blind=0.35 / min_obstacle_intensity=0.05 / minBlockPointNum=5`、Point-LIO `gravity/blind/range` 不动），无数值变更。C 阶段语义 Route Graph 和 D 阶段 Smac Lattice/ConstrainedSmoother/MINCO 均为未来门控目标，尚未实装；MINCO 属于可选轨迹优化 / smoother 工具，不是必需。
>
> **C 阶段启动门控**（必须同时满足）：① 仿真 rmuc_2026 窄道与高低（坡道/台阶/资源岛）smoke 连续 3 次 `navigate_to_pose` SUCCEEDED；② 控制器频率稳定在 `controller_frequency` 设定值附近（仿真 20Hz，实测不低于 70%）；③ `slam:=False` 先验资源齐备（`install/sentry_nav_bringup/share/sentry_nav_bringup/map/simulation/rmuc_2026.yaml` 可加载，`install/sentry_nav_bringup/share/sentry_nav_bringup/pcd/simulation/rmuc_2026.pcd` 可加载）。Task 5 smoke 的 `slam:=False` 仍被这两项资源缺失 + `Costmap timed out waiting for update` 阻塞，**C 阶段尚未具备启动条件**。
>
> **D 阶段启动门控**（必须同时满足）：① C 阶段语义 Route Graph 在仿真 rmuc_2026 完整巡逻 1 小时路径语义稳定；② 有具体数据证明"MPPI + 现有 costmap + 语义 Route Graph"仍无法解决某类场景（窄弯跟随误差、倒车贴障、最小转弯半径规划失败等），需 Smac Lattice / ConstrainedSmoother / MINCO 之一才能收敛；③ 每项候选分支单独可回滚，不允许三者同时上。
>
> 推进阶段看的是可观测性、可重复 QA、可安全回滚、路径语义稳定，**不是算法复杂度**。"企业级"体现在故障可追溯，不是堆模型。详见 [架构详解 §4.5 分阶段导航路线图](ARCHITECTURE.md#45-分阶段导航路线图)。

---

## 一、Point-LIO 里程计参数

### 1. point_filter_num（点云抽稀比例）

**当前值**：4（每 4 个点取 1 个，25% 参与配准）

**调优范围**：2 ~ 8

**影响**：
- 值越小 → 参与配准的点越多 → 精度越高，CPU 占用越高
- 值越大 → 点越少 → 速度越快，但配准可能漂移

**调优方法**：
```bash
# 1. 让机器人以目标速度（如 3m/s）跑直线并返回起点
# 2. 观察回到起点时的位移误差
ros2 topic echo /red_standard_robot1/aft_mapped_to_init --field pose.pose.position --once

# 3. 同时监控 CPU 占用
top -p $(pgrep -f point_lio)
```

**判断标准**：
- 闭环位移误差 < 10cm → 当前值可接受
- 闭环位移误差 > 20cm → 减小到 3 或 2
- CPU 单核占用 > 40% → 增大到 6 或 8

---

### 2. filter_size_surf / filter_size_map（体素降采样分辨率）

**当前值**：0.2m / 0.2m

**调优范围**：0.1 ~ 0.5

**影响**：
- filter_size_surf：当前帧配准的点云密度，影响实时性
- filter_size_map：增量地图的点云密度，影响配准精度和内存占用

**调优方法**：
```bash
# 监控 point_lio 处理延迟（应 < 50ms/帧）
ros2 topic delay /red_standard_robot1/aft_mapped_to_init

# 监控里程计频率（应接近 LiDAR 频率）
ros2 topic hz /red_standard_robot1/aft_mapped_to_init
```

**判断标准**：
- 处理延迟 > 50ms → 增大 filter_size（0.3 或 0.5）
- 里程计频率低于 LiDAR 频率的 80% → 增大 filter_size
- 直线漂移明显 → 减小 filter_size（0.15 或 0.1）

---

### 3. ivox_grid_resolution（增量体素地图分辨率）

**当前值**：0.5m

**调优范围**：0.2 ~ 1.0

**影响**：
- 更小的分辨率 → 地图更精细，配准更精确，内存占用更大
- 更大的分辨率 → 地图更粗糙，内存更小，配准可能不精确

**调优方法**：
```bash
# 监控内存占用（长时间运行后）
ps aux | grep point_lio | awk '{print $6/1024 "MB"}'

# 同时观察配准精度（闭环误差）
```

**判断标准**：
- 内存持续增长超过 500MB → 增大到 0.8 或 1.0
- 配准精度不足 → 减小到 0.3 或 0.2
- 一般保持与 filter_size_map 相同或略大

---

### 4. ivox_nearby_type（增量体素邻居搜索类型）

**当前值**：18

**可选值**：0（1邻居）、6（6邻居）、18（18邻居）、26（26邻居）

**影响**：
- 值越大 → 搜索更多邻居 → 配准更鲁棒，CPU 更高
- 值越小 → 搜索更少 → 速度更快，可能漏匹配

**判断标准**：
- CPU 余量充足 → 保持 18
- CPU 紧张 → 降到 6（精度损失较小）
- 不建议用 26（收益递减，CPU 开销大）

---

### 5. lidar_meas_cov（LiDAR 测量协方差）

**当前值**：仿真 0.001，实车 0.01

**调优范围**：0.001 ~ 0.1

**影响**：
- 值越小 → EKF 更信任 LiDAR → 配准权重更大，对 LiDAR 噪声敏感
- 值越大 → EKF 更信任 IMU → 对 LiDAR 退化场景更鲁棒，但整体精度可能下降

**调优方法**：
```bash
# 在 LiDAR 退化场景（长走廊、空旷区域）测试
# 观察里程计是否剧烈抖动

# 在纹理丰富场景测试
# 观察闭环误差
```

**判断标准**：
- 空旷/走廊场景里程计抖动 → 增大到 0.05 或 0.1
- 纹理丰富场景精度不足 → 减小到 0.005 或 0.001
- 实车一般用 0.01（mid360 噪声比仿真大）

---

### 6. plane_thr（平面判定阈值）

**当前值**：0.1

**调优范围**：0.01 ~ 0.2

**影响**：
- 值越小 → 更严格的平面判定 → 只有非常平的面才参与配准
- 值越大 → 更宽松 → 更多点参与配准，但可能引入噪声

**判断标准**：
- 结构化环境（室内、场地围栏多）→ 0.05~0.1
- 非结构化环境（户外、不规则地形）→ 0.1~0.2

---

### 7. match_s（配准搜索半径²）

**当前值**：81（即 9m 搜索半径）

**调优范围**：25 ~ 225

**影响**：搜索半径 = sqrt(match_s)。更大的搜索范围在大位移时更鲁棒，但计算量更大。

**判断标准**：
- 最大帧间位移 < 搜索半径 → 正常
- 3m/s ÷ 20Hz = 0.15m/帧，远小于 9m → 当前值足够
- 一般不需要调整，除非 LiDAR 频率极低或速度极快

---

## 二、近距离过滤与自身点云

### 8. blind（近距离过滤半径）

**当前值**：仿真 0.35m，实车 0.2m

**调优范围**：0.2 ~ 1.0

**影响**：
- 过小 → 自身点云泄漏，轨迹上出现虚假障碍物
- 过大 → 丢失近距离真实障碍物

> 先确认 `gimbal_pitch → front_mid360` 的传感器安装外参与实际一致，再调 `blind`。`blind` 只能压自身点，不能补传感器 FOV 盲区；也不要再用 `mapping.gravity` 去表达安装角。

**调优方法**：
```bash
# 让机器人静止，在 RViz 中可视化 cloud_registered
# 检查是否有机器人本体的点

# 如果 costmap 在机器人位置附近显示障碍物，说明 blind 不够大
```

**判断标准**：
- RViz 中 cloud_registered 能看到机器人部件的点 → 增大 blind
- 近距离（< 0.5m）真实障碍物检测不到 → 减小 blind

---

## 三、地形分析与障碍物检测

### 9. min_obstacle_intensity（障碍物最小高度阈值）

**当前值**：仿真 / 实车均为 0.05（即 5cm 以上的高度差即可进入 `IntensityVoxelLayer`）

**调优范围**：0.05 ~ 0.3

**配置位置**：`local_costmap` 和 `global_costmap` 下的 `intensity_voxel_layer`

**影响**：
- 过小 → 地面噪声被识别为障碍物
- 过大 → 矮小的真实障碍物被忽略
- rmuc_2026 的低矮底座依赖该阈值保持较低；没有实测噪声证据时，不要为了让 costmap “看起来更干净”而调高。

**调优方法**：
```bash
# 在 RViz 中订阅 terrain_map 查看点云
# intensity = 该点相对于地面的高度差 (m)

# 平坦地面上有大量障碍物标记 → 增大阈值
# 已知的矮障碍物没有被检测到 → 减小阈值
```

---

### 10. vehicleHeight（障碍物最大检测高度）

**当前值**：terrain_analysis 0.70m，terrain_analysis_ext 0.8m

**调优范围**：0.3 ~ 1.5

**影响**：高于此值的点不会被标记为障碍物（如天花板、高处结构）

**判断标准**：
- 设为机器人能通过的最大高度即可
- 过大会把高处结构误判为障碍物

### 10-B. terrain / costmap 安全边界（rmuc_2026）

- `terrain_analysis.clearDyObs` 保持 `False`：动态障碍需要留在 costmap 中参与避障，不能被地形层主动清掉。
- Point-LIO `preprocess.blind`、`mapping.gravity` / `gravity_init` 和仿真 LiDAR 最小量程不属于 costmap 调参入口；近场低矮障碍漏检时应先核对 TF 外参、传感器量程和点云，而不是把安装角写进 gravity。
- 仿真 `robot_radius: 0.46`、`inflation_radius: 0.90`、`cost_scaling_factor: 8.0` 是当前保守碰撞裕量；没有窄通道实测失败证据时不要放松，否则会降低防擦角安全余量。

---

## 四、LiDAR 频率与时间同步

### 11. publish_freq（实车 LiDAR 发布频率）

**当前值**：30Hz

**调优范围**：10 ~ 50

**影响**：
- 更高频率 → 里程计更新更快，高速运动跟踪更好，CPU 增加
- 更低频率 → CPU 更低，但高速运动时配准难度增加

**判断标准**：
- 最大运动速度 / LiDAR 频率 < 0.15m → 频率足够
- 3m/s：20Hz → 每帧 15cm ✓，10Hz → 每帧 30cm ✗

**注意**：修改 `publish_freq` 后必须同步修改 `lidar_time_inte = 1 / publish_freq`。

---

## 五、GICP 重定位

### 12. accumulated_count_threshold（初始定位累积帧数）

**当前值**：20

**调优范围**：10 ~ 50

**调优方法**：
```bash
# 查看 GICP 初始定位日志
grep "GICP" /tmp/nav_log.txt

# converged=1 且 inlier_ratio > 0.5 → 当前值足够
# converged=0 或 inlier_ratio < 0.3 → 增大到 30-50
```

---

### 13. registered_leaf_size / global_leaf_size（GICP 降采样分辨率）

**当前值**：0.1 / 0.2

**调优范围**：0.05 ~ 0.3

**影响**：
- registered_leaf_size：source 点云密度，影响配准精度
- global_leaf_size：target（先验地图）密度，影响加载时间和内存

**判断标准**：
- GICP source 点数 < 200 → 减小 registered_leaf_size
- GICP source 点数 > 5000 → 可适当增大
- 查看日志中 `GICP input: source=N points`

---

## 六、速度平滑器 (velocity_smoother)

velocity_smoother 是 Nav2 官方节点，位于 controller_server 与 `sentry_motion_manager` 之间，负责限制加速度和最大速度，防止指令突变。集成 bringup 中它输出 `cmd_vel_nav`，最终 `/cmd_vel` 由 motion manager 发布给底盘执行器。

### 14. smoothing_frequency（平滑器频率）

**当前值**：仿真 20Hz，实车 30Hz

**核心原则（硬约束）**：必须与 `controller_frequency` 一致，否则高频端指令被丢弃。

| 环境 | controller_frequency | smoothing_frequency | 状态 |
|---|---|---|---|
| 仿真 | 20 | 20 | ✅ 匹配 |
| 实车 | 30 | 30 | ✅ 匹配 |

**判断标准**：
- `ros2 topic hz /cmd_vel_nav` 应接近 `smoothing_frequency`
- `ros2 topic hz /cmd_vel` 应接近 motion manager 的 `output_frequency_hz`
- 如果明显低于设定值 → CPU 不足，降低频率

### 15. feedback 模式

**当前值**：`OPEN_LOOP`

| 模式 | 行为 | 适用场景 |
|---|---|---|
| `OPEN_LOOP` | 以上一次输出的指令作为"当前速度"计算加速度 | 底盘响应好、不需要精确跟踪 |
| `CLOSED_LOOP` | 读 odometry 实际速度作为"当前速度" | 底盘响应延迟大、打滑严重 |

**调优方法**：
```bash
# 运行 serial_visualizer 观察 cmd vs actual 曲线
# 仿真:
python3 src/sentry_tools/serial_visualizer.py --ros-args -r __ns:=/red_standard_robot1
# 实车:
python3 src/sentry_tools/serial_visualizer.py
```

**判断标准**：
- 实际速度能跟上命令，误差 < 0.2 m/s → `OPEN_LOOP` 够用
- 实际速度明显滞后或超调 → 切 `CLOSED_LOOP`
- `CLOSED_LOOP` 依赖 odometry 质量，如果里程计有噪声反而可能引入抖动

### 16. max_accel / max_decel（加速度限制）

**当前值**：`[4.5, 0.0, 5.0]` / `[-4.5, 0.0, -5.0]`

**调优范围**：1.0 ~ 6.0 m/s²（vy 始终锁 0）

**影响**：
- 过大 → 底盘实际跟不上（轮子打滑），smoother 形同虚设
- 过小 → 机器人加速慢，响应迟钝

**调优方法**：
```bash
# 1. 用 serial_visualizer 给导航目标，观察加速段：
#    - cmd 曲线是平滑斜坡（smoother 在限制）
#    - actual 曲线紧跟 cmd → 当前加速度合适
#    - actual 曲线明显低于 cmd → 加速度超过底盘能力，减小

# 2. 估算底盘最大加速度：
#    观察 actual 曲线从 0 到峰值的斜率 = 实际最大加速度
#    max_accel 应 <= 实际最大加速度的 80%
```

**判断标准**：
- 仿真摩擦力限制 ≈ 0.2g ≈ 2.0 m/s²，当前 4.5 超过物理极限（仿真中 smoother 无实际意义）
- 实车需实测：全速加速时观察轮子是否打滑

### 17. max_velocity（最大速度限制）

**当前值**：仿真 `[1.5, 0.0, 6.3]` / 实车 `[1.5, 0.0, 3.0]` —— **vy 必须锁 0**（差速约束）

应与 controller 的 `desired_linear_vel` / `max_angular_accel * simulate_ahead_time` 一致或略大。如果 smoother 限速比 controller 小，controller 的指令会被截断。

### 18. deadband_velocity（死区）

**当前值**：`[0.0, 0.0, 0.0]`

**调优范围**：0.0 ~ 0.05

**影响**：低于死区的速度被归零，消除低速抖动。

**判断标准**：
- serial_visualizer 中静止时 cmd 在 ±0.02 范围内抖动 → 设 `[0.02, 0.0, 0.05]`（vy 永为 0）
- 没有抖动 → 保持 0

---

## 六-B、仿真差速控制器 (MPPI DiffDrive，Phase A 已落地)

仿真 `config/simulation/nav2_params.yaml` 的 `controller_plugins` 现在只留 `FollowPath`，插件为 `nav2_mppi_controller::MPPIController`，`motion_model: "DiffDrive"`。实车 `config/reality/nav2_params.yaml` 仍使用 RotationShim + RPP，未进入本阶段迁移。

### 18-B. MPPI Horizon 安全约束（硬规则）

**不等式**：`time_steps × model_dt × vx_max < local_costmap_half_width`

**当前值**：`32 × 0.05 × 1.5 = 2.4m`，local costmap `width/height=5.0m`，半径 2.5m，留 0.1m 裕量。

**为什么重要**：MPPI 末端轨迹点如果落到 local costmap 之外，该段轨迹无代价可评估，critic 会返回默认值导致采样失真，等价于在盲区决策。任意一个参数变大都会吃掉裕量，必须手算一次再改。

### 18-C. 差速语义硬锁

| 参数 | 必须取值 | 原因 |
|---|---|---|
| `motion_model` | `DiffDrive` | 差速模型严格禁止横移采样 |
| `vy_max` | 0.0 | 即使 critic 允许，下游 velocity_smoother / motion_manager 也会锁 0，提前锁可减少无效采样 |
| `vy_std` | 0.0 | 采样噪声在 vy 方向的标准差也锁 0，避免噪声让 MPPI 学到偏侧向的策略 |
| `vx_min` | 0.0 | 今日不允许倒车；倒车需用户单独开闸并重新验证贴障风险 |

### 18-D. CPU / RTF 相关参数

| 参数 | 当前值 | 说明 |
|---|---|---|
| `batch_size` | 2000 | 采样批量，保守值；拉高会线性增加 CPU |
| `iteration_count` | 1 | 单次迭代；增大会多轮 refine，每帧耗时翻倍 |
| `visualize` | false | 轨迹可视化走 ROS 话题，打开后会明显拖低仿真 RTF |
| `regenerate_noises` | false | 复用上帧噪声，关掉随机重生以省 CPU |

### 18-E. 频率一致性

- `controller_frequency` 与 `velocity_smoother.smoothing_frequency` 必须相等（仿真 20Hz）。Smoother 频率低于 controller 会丢指令，高于又吃不到数据。
- 不要拉高到 CPU 跑不动的值。实测若 20Hz 设定跑不到 15Hz，就应降频而不是堆参数。

### 18-F. Critics 列表

Nav2 Jazzy 官方差速推荐集，不要随意删 critic：

```
ConstraintCritic, CostCritic, GoalCritic, GoalAngleCritic,
PathAlignCritic, PathFollowCritic, PathAngleCritic, PreferForwardCritic
```

调权重时优先只改 `cost_weight`，不要直接改 `cost_power`，也不要新增自研 critic（Phase A 明确排除）。

---

## 六-C、实车差速控制器 (RotationShim + RPP，未迁移)

仿真已迁移到 MPPI（见 六-B）。以下参数只对 `config/reality/nav2_params.yaml` 生效；如果后续要把实车也迁到 MPPI，整段会被替换。

### 19. RotationShim 关键参数

| 参数 | 实车默认 | 说明 |
|---|---|---|
| `angular_dist_threshold` | 3.14 | 接近 180°：几乎总让 RPP 先接管，只有近似掉头才会先原地转。调小（如 0.785）会让机器人起步更多场合"先原地对齐再走"，起步响应变慢但贴墙更稳；调大（接近 pi）几乎禁用 RotationShim，交由 RPP 自身曲率控制 |
| `forward_sampling_distance` | 0.5 | 路径前方采样距离，用于判定目标方向 |
| `rotate_to_heading_angular_vel` | 1.2 | 原地旋转角速度上限 (rad/s) |
| `max_angular_accel` | 2.5 | 原地旋转角加速度 (rad/s²) |
| `simulate_ahead_time` | 1.0 | 碰撞检查的预测时间窗 |

**调优方法**：
- 机器人进入路径时反复小幅振荡 → 增大 `angular_dist_threshold`（如 0.785 → 1.571 或接近 pi）减少 RotationShim 接管频次
- 出了 RotationShim 后 RPP 急转 → 减小 `angular_dist_threshold`（如 3.14 → 1.0 → 0.785）让 RotationShim 对齐得更精确

### 20. RPP 关键参数

| 参数 | 实车默认 | 说明 |
|---|---|---|
| `desired_linear_vel` | 0.8 | 期望线速度 (m/s)，略降巡航以给后向感知更多反应裕度 |
| `lookahead_dist` | 1.0 | 基础前瞻距离 (m) |
| `min_lookahead_dist` | 0.5 | velocity-scaled lookahead 下限 |
| `max_lookahead_dist` | 1.2 | velocity-scaled lookahead 上限 |
| `lookahead_time` | 1.0 | `lookahead = max(min, vx * lookahead_time)` 被限幅 |
| `use_velocity_scaled_lookahead_dist` | true | 开启速度相关的 lookahead 自适应 |
| `use_regulated_linear_velocity_scaling` | true | 开启曲率与接近减速 |
| `regulated_linear_scaling_min_radius` | 0.5 | 曲率半径小于此值开始降速 |
| `regulated_linear_scaling_min_speed` | 0.3 | 曲率降速的速度下限 |
| `approach_velocity_scaling_dist` | 0.8 | 接近目标开始减速的距离 |
| `min_approach_linear_velocity` | 0.3 | 接近段最小线速度 |
| `use_rotate_to_heading` | false | 哨兵由云台瞄准，底盘不要求终端对齐目标朝向 |
| `rotate_to_heading_min_angle` | 0.785 | 触发终端旋转的最小角度差 (rad，仅在 `use_rotate_to_heading: true` 时生效) |
| `max_angular_accel` | 2.5 | 角加速度上限 (rad/s²) |
| `rotate_to_heading_angular_vel` | 1.2 | 原地/终端旋转角速度 (rad/s) |
| `allow_reversing` | true | 允许倒车跟踪路径，后置雷达场景可优先使用后向感知 |
| `use_collision_detection` | YAML `false`，`slam:=False` 时 launch 改 `true` | SLAM 建图保持 false 避免 unknown 误报；纯导航模式自动开启前向碰撞预测 |
| `max_allowed_time_to_collision_up_to_carrot` | 1.5 | 碰撞预测时间窗 (s) |

**调优流程**（实车场景）：
1. **线速度**：先用 `desired_linear_vel` 把期望速度定在硬件能吃到的值（实车当前 0.8，留后向感知反应裕度；若窄道没问题再逐步上调到 1.0/1.2 验证）。
2. **路径跟随**：若直线段频繁震荡，增大 `lookahead_dist`；若弯道切内拐角严重，减小 `lookahead_dist` 或增大 `regulated_linear_scaling_min_radius`。
3. **高曲率降速**：观察 U 弯是否擦墙，若需更激进减速减小 `regulated_linear_scaling_min_radius`。
4. **终端朝向**：哨兵默认不做底盘终端对齐；如果某个任务确实要求车头朝向目标，再单独打开 `use_rotate_to_heading` 并重新验证贴障风险。
5. **RotationShim 与 RPP 交接**：观察日志行 `Rotating shim active` 频率；如果起步贴墙/侧扫障碍，优先减小 `angular_dist_threshold`，让机器人先原地对齐再给前进。
6. **碰撞预测**：导航模式应确认 launch 已把 `use_collision_detection` 打开；SLAM 模式若 unknown 太多导致误报，再保留关闭。

### 20-B. 差速恢复策略

- 差速底盘的 recovery 不能依赖 `vy` 逃逸，所有兜底动作都必须落在 `vx + wz` 语义内。
- 默认 BT 已移除旧 `<Spin/>` + `<BackUp/>` 主恢复路径，避免贴墙场景继续依赖会从 LIO 抖动中误判成功的反向动作。
- `BackUpFreeSpace` 已于 2026 赛季重构时从 `nav2_plugins` 中删除；贴墙脱困由 `sentry_motion_manager` recovery 状态机通过 `cmd_vel_recovery` 接管，使用投影位移/进展判断避免抖动误判。`nav2_plugins` 包仍保留用于提供 `IntensityVoxelLayer`，不要删除整个包。
- 当前 BT 只保留清图+等待占位；真正贴墙脱困应由 `sentry_motion_manager` recovery 状态机接管。

### 21. 终端参数（general_goal_checker）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `xy_goal_tolerance` | 0.15 | 到点距离容差 (m) |
| `yaw_goal_tolerance` | 6.28 | 哨兵默认不关心底盘终端 yaw，保持 2π 避免额外朝向约束 |
| `stateful` | True | 到点后保持状态，避免振荡 |

---

## 七、快速诊断命令汇总

```bash
# Point-LIO 里程计频率（应接近 LiDAR 频率）
ros2 topic hz /red_standard_robot1/aft_mapped_to_init

# Point-LIO 处理延迟（应 < 50ms）
ros2 topic delay /red_standard_robot1/aft_mapped_to_init

# 注册点云频率和点数
ros2 topic hz /red_standard_robot1/registered_scan
ros2 topic echo /red_standard_robot1/registered_scan --field width --once

# GICP 重定位状态
grep "GICP" /tmp/nav_log.txt

# CPU 占用
top -p $(pgrep -f "point_lio|terrain|nav2_container")

# 内存占用
ps aux | grep point_lio | awk '{print $6/1024 "MB"}'

# TF 延迟（应 < 100ms）
ros2 run tf2_ros tf2_monitor map odom --ros-args -r __ns:=/red_standard_robot1
```

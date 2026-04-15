# Point-LIO 与导航参数调优指南

本文档列出了需要在实车上根据实际情况微调的参数，以及对应的调优方法。

配置文件位置：
- 仿真：`sentry_nav_bringup/config/simulation/nav2_params.yaml`
- 实车：`sentry_nav_bringup/config/reality/nav2_params.yaml`

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

**当前值**：仿真 0.5m，实车 0.2m

**调优范围**：0.2 ~ 1.0

**影响**：
- 过小 → 自身点云泄漏，轨迹上出现虚假障碍物
- 过大 → 丢失近距离真实障碍物

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

**当前值**：0.2（即 20cm 以上的高度差才算障碍物）

**调优范围**：0.1 ~ 0.3

**配置位置**：`local_costmap` 和 `global_costmap` 下的 `intensity_voxel_layer`

**影响**：
- 过小 → 地面噪声被识别为障碍物
- 过大 → 矮小的真实障碍物被忽略

**调优方法**：
```bash
# 在 RViz 中订阅 terrain_map 查看点云
# intensity = 该点相对于地面的高度差 (m)

# 平坦地面上有大量障碍物标记 → 增大阈值
# 已知的矮障碍物没有被检测到 → 减小阈值
```

---

### 10. vehicleHeight（障碍物最大检测高度）

**当前值**：terrain_analysis 0.5m，terrain_analysis_ext 1.0m

**调优范围**：0.3 ~ 1.5

**影响**：高于此值的点不会被标记为障碍物（如天花板、高处结构）

**判断标准**：
- 设为机器人能通过的最大高度即可
- 过大会把高处结构误判为障碍物

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

velocity_smoother 是 Nav2 官方节点，位于 controller_server 和 fake_vel_transform 之间，负责限制加速度和最大速度，防止指令突变。

### 14. smoothing_frequency（平滑器频率）

**当前值**：仿真 20Hz，实车 40Hz

**核心原则**：必须与 `controller_frequency` 一致，否则高频端指令被丢弃。

| 环境 | controller_frequency | smoothing_frequency | 状态 |
|---|---|---|---|
| 仿真 | 20 | 20 | ✅ 匹配 |
| 实车 | 40 | 40 | ✅ 匹配 |

**判断标准**：
- `ros2 topic hz cmd_vel_nav2_result` 应接近 `smoothing_frequency`
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

**当前值**：`[4.5, 4.5, 5.0]` / `[-4.5, -4.5, -5.0]`

**调优范围**：1.0 ~ 6.0 m/s²

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

**当前值**：`[2.5, 2.5, 3.0]`

应与 controller 的 `v_linear_max` / `v_angular_max` 一致或略大。如果 smoother 限速比 controller 小，controller 的指令会被截断。

### 18. deadband_velocity（死区）

**当前值**：`[0.0, 0.0, 0.0]`

**调优范围**：0.0 ~ 0.05

**影响**：低于死区的速度被归零，消除低速抖动。

**判断标准**：
- serial_visualizer 中静止时 cmd 在 ±0.02 范围内抖动 → 设 `[0.02, 0.02, 0.05]`
- 没有抖动 → 保持 0

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

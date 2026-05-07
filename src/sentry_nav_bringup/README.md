# sentry_nav_bringup

> **Maintainer**: Boombroke <boombroke@icloud.com>

## 简介

sentry_nav_bringup 是哨兵机器人导航系统的核心启动配置包。它整合了所有 launch 脚本、Nav2 参数配置、地图数据、点云文件、RViz 配置文件以及行为树 XML 定义。

## 局部控制器阶段化迁移

- **仿真 `config/simulation/nav2_params.yaml`（Phase A 已落地）**：`controller_plugins: ["FollowPath"]`，`FollowPath` 为 `nav2_mppi_controller::MPPIController`，`motion_model: "DiffDrive"`，锁 `vy_max=0 / vy_std=0`，今日 `vx_min=0`（不倒车）。Horizon 约束 `time_steps × model_dt × vx_max < local_costmap_half_width`，当前 `32 × 0.05 × 1.5 = 2.4m < 2.5m`。控制器频率 `controller_frequency` 与 `velocity_smoother.smoothing_frequency` 必须保持相等（当前仿真 20Hz）。
- **实车 `config/reality/nav2_params.yaml`（Phase R-A 首版已落地，全场验证待 Task 6）**：`controller_plugins: ["FollowPath"]`，`FollowPath` 为 `nav2_mppi_controller::MPPIController`，`motion_model: "DiffDrive"`，`vy_max=0 / vy_std=0 / vx_min=0`（首版前向优先），`wz_max=3.0 / az_max=5.0`（受 `velocity_smoother.max_velocity[2]=3.0 / max_accel[2]=5.0` 约束，不能直接套用仿真 6.3/8.0）。Planner `motion_model_for_search: "DUBIN"` 与 `vx_min=0` 语义对齐，避免生成无法跟随的倒车/cusp 路径。Horizon 同上：`32 × 0.05 × 1.5 = 2.4m < 2.5m`。控制器频率 `controller_frequency` 与 `velocity_smoother.smoothing_frequency` 必须保持相等（当前实车 30Hz）。全场/上场 Nav Goal smoke 依赖 Task 6 台架和实测落地。
- **运行时无 RPP 回退**：仿真和实车都没有 MPPI → RPP 的 runtime fallback，回退只能通过 git/config 重新部署旧的 RPP/RotationShim 配置；实车回滚时记得同步把 Planner 换回 REEDS_SHEPP。若实车 CPU 顶不住 30Hz，fallback 策略是**降 MPPI `batch_size`**（`2000 → 1500 → 1000`），不允许动 `controller_frequency / smoothing_frequency`，更不允许动 `robot_radius=0.318`。
- **C/D 阶段未实装**：语义 Route Graph（Phase C）、Smac Lattice / ConstrainedSmoother / MINCO（Phase D，MINCO 为可选工具，不是必需）均挂在未来路线图上，**今日不修改长期规划器**（全局规划器继续使用 `nav2_smac_planner::SmacPlannerHybrid`）。C 启动门控：rmuc_2026 窄道/高低 smoke 连续 3 次 `navigate_to_pose` SUCCEEDED + 控制器频率稳定 + `slam:=False` 所需 `map/simulation/rmuc_2026.yaml` 与 `pcd/simulation/rmuc_2026.pcd` 均可加载（当前 Task 5 smoke 还被这两项资源缺失加 `Costmap timed out waiting for update` 阻塞）。D 启动门控：C 语义 Route Graph 稳定 + 有证据证明当前链路解决不了某类场景。完整表格与门控细则见 [`src/docs/ARCHITECTURE.md` §4.5](../docs/ARCHITECTURE.md#45-分阶段导航路线图)。

## 主要 Launch 文件

### 仿真模式（两终端启动，时序敏感）

仿真需分两步启动。直接合并成一个 launch 会导致 Gazebo spawn/unpause/传感器流稳定未完成时 Nav2 已拉起，Point-LIO 收不到 IMU 初始化失败。

| Launch | 作用 |
|---|---|
| 外部: `rmu_gazebo_simulator/bringup_sim.launch.py` | 终端 1：启动 Gazebo 仿真器（需手动 unpause） |
| `rm_navigation_simulation_launch.py` | 终端 2：Gazebo 就绪后启动导航栈（SLAM 或 定位 模式） |

### 实车模式

| Launch | 作用 |
|---|---|
| `rm_sentry_launch.py` | 实车一键启动（导航 + 串口驱动 + LiDAR，整合 `rm_navigation_reality_launch.py` 和 `rm_serial_driver`） |
| `rm_navigation_reality_launch.py` | 实车导航主入口（单独启动，不含串口驱动） |

### 内部模块化 Launch（被上面主 launch 调用，通常不直接调用）

| Launch | 作用 |
|---|---|
| `bringup_launch.py` | 集成 slam / localization / navigation 三者的路由 launch |
| `slam_launch.py` | 启动 slam_toolbox 建图 |
| `localization_launch.py` | 启动 map_server + small_gicp 重定位 |
| `navigation_launch.py` | 启动 Nav2 核心节点（controller / planner / bt_navigator 等），velocity_smoother 输出 `cmd_vel_nav` |
| `robot_state_publisher_launch.py` | 发布机器人 TF 和模型状态 |
| `rviz_launch.py` | 启动预配置的可视化界面 |

`bringup_launch.py` 会同时拉起 `sentry_motion_manager`。集成 bringup 参数中启用 `command_output_enabled: true`，因此最终 `/cmd_vel` 由 motion manager 发布；`rm_serial_driver` 和 Gazebo DiffDrive 仍订阅原来的 `/cmd_vel`，无需改下游消费者。

### 多机器人（实验性）

| Launch | 作用 |
|---|---|
| `rm_multi_navigation_simulation_launch.py` | 多机器人同场景仿真启动 |

## 启动示例

### 仿真建图（首次跑）

```bash
# 终端 1：Gazebo
QT_QPA_PLATFORM=xcb ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# 等 Gazebo spawn 完成后，手动注册机器人为 level performer（每个机器人执行一次）：
gz service -s /world/default/level/set_performer \
  --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'data: "red_standard_robot1"'

# 然后 unpause Gazebo：
gz service -s /world/default/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'pause: false'

# 再等 ~10s 让传感器流稳定。验证：
ros2 topic hz /red_standard_robot1/livox/imu   # 应见 ~150Hz

# 终端 2：导航 + SLAM 实时建图
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=True
```

### 仿真导航（有先验地图后）

```bash
# 同样终端 1 起 Gazebo，set_performer 后 unpause（步骤同上）
# 终端 2：
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False
```

### 实车一键

```bash
# 首次建图
ros2 launch sentry_nav_bringup rm_sentry_launch.py slam:=True

# 有图后导航
ros2 launch sentry_nav_bringup rm_sentry_launch.py world:=<map_name> slam:=False
```

### 发 Nav Goal 测试

```bash
ros2 action send_goal /red_standard_robot1/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

## Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | `red_standard_robot1`（仿真） / `""`（实车） | 机器人命名空间 |
| `slam` | `False` | 是否启用 slam_toolbox 实时建图 |
| `world` | `""` | 地图名称（对应 `map/{simulation,reality}/<world>.yaml`） |
| `params_file` | `config/{simulation,reality}/nav2_params.yaml` | Nav2 参数文件 |
| `use_rviz` | `True` | 启动 RViz 可视化 |
| `use_robot_state_pub` | `True`（`rm_sentry_launch.py`） / `False`（`rm_navigation_reality_launch.py`） | 实车一键启动时默认发布机器人 TF；单独启动导航时按需打开 |

## 常见启动问题

### `package 'sentry_motion_manager' not found`

当前速度链路要求 `velocity_smoother` 输出 `/cmd_vel_nav`，再由 `sentry_motion_manager` 仲裁发布最终 `/cmd_vel`。如果实车 workspace 是旧版本创建的，可能已经有 `sentry_nav_bringup` 链接，但缺少后来新增的 `sentry_motion_manager` 包。

现场恢复：

```bash
cd ~/sentry_ws
colcon build --symlink-install --packages-select sentry_motion_manager sentry_nav_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg prefix sentry_motion_manager
```

如果 `--packages-select` 提示找不到 `sentry_motion_manager`，先同步最新源码或重新运行 `bash src/scripts/setup_env.sh`；脚本会逐项补齐旧 workspace 中缺失的新增源码包链接。

## 订阅话题

- `/initialpose`：接收初始位姿估计
- `/goal_pose`：接收 RViz 发出的导航目标点

## 发布话题

- `/tf`, `/tf_static`：坐标变换树
- `/map`：当前使用的 2D 占据栅格地图
- `/cmd_vel`（`TwistStamped`）：给底盘消费者（Gazebo DiffDrive 插件 或 rm_serial_driver）
- `/cmd_vel_nav`（`TwistStamped`）：Nav2 velocity_smoother 输出，作为 motion manager 的 navigation 输入
- `/motion_manager/state`（`String`）：底盘运动管理器当前模式与选中速度源摘要
- `/odometry`（`Odometry`）：pose 为 `odom -> base_footprint`，twist 为 `base_footprint` 机体系 `vx + wz`

## 实车调试录包建议

排查导航失败或速度尖峰时，建议至少录制：

- `/tf`, `/tf_static`, `/joint_states`
- `/livox/lidar`, `/livox/imu`（`use_dual_mid360:=True` 时 `/livox/lidar` 是 merger 输出的 CustomMsg；实车还可加录 `/livox/lidar_front`, `/livox/lidar_back`, `/livox/imu_back` 做前融合回溯）
- `/aft_mapped_to_init`, `/cloud_registered`
- `/odometry`, `/cmd_vel`, `/cmd_vel_nav`, `/cmd_vel_controller`
- `/local_costmap/costmap_raw`, `/global_costmap/costmap_raw`
- `/terrain_map`, `/terrain_map_ext`
- `/rosout`

`/rosout` 对定位 BT 失败原因很关键；只有行为树状态统计时，只能看到 `ComputePathToPose` / `FollowPath` 等节点失败次数，无法知道具体是 TF 超时、碰撞检查、progress checker 还是 planner 不可达。当前默认 BT 已移除旧 `<Spin/>/<BackUp/>` 主恢复路径，暂以清图+等待占位，后续由 motion manager recovery 适配器接管贴墙脱困触发。

## 配置与资源目录

- **`config/`**：`simulation/` 和 `reality/` 两套 `nav2_params.yaml`
- **`map/`**：2D 占据栅格地图（大文件未入仓）
- **`pcd/`**：Point-LIO 先验点云（大文件未入仓）
- **`rviz/`**：RViz 配置
- **`behavior_trees/`**：Nav2 行为树定义（`navigate_to_pose_w_replanning_and_recovery.xml` 等）

## Debug

```bash
# 跟随最新 launch 日志
bash src/scripts/debug/tail_log.sh

# 跟随指定节点
bash src/scripts/debug/tail_log.sh controller_server
bash src/scripts/debug/tail_log.sh pointlio

# 搜关键字
bash src/scripts/debug/grep_log.sh error
bash src/scripts/debug/grep_log.sh "TF lookup"
```

详见 [logs/README.md](../../logs/README.md)。

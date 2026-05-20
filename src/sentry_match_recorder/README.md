# sentry_match_recorder

## 1. 概述

`sentry_match_recorder` 是哨兵机器人**比赛期间自动 rosbag 录制**功能包。订阅 `/referee/game_status`，在比赛进入 `game_progress=4`（比赛中）的上升沿自动启动 `ros2 bag record`，下降沿（含 `5/0/1` 等其他状态）自动停止。利用 rosbag2 内建 `--max-bag-duration` 在**单个进程内**滚动切片，单场比赛全部切片输出到一个 `sortie_YYYYMMDD_HHMMSS/` 目录，便于按场次归档和回放调试。

设计目标：

- **零人工干预**：上电后接入裁判帧即可，比赛开始自动录、结束自动停。
- **稳定低占用**：单 subprocess + MCAP 不压缩，CPU 几乎只是 IO；切片间无缝隙，不丢数据。
- **回放友好**：默认话题白名单覆盖 TF + 双 LiDAR/IMU + 里程计 + Nav2 控制全链 + Costmap + 行为树 + 裁判全量，足以离线复现 Point-LIO、costmap、控制器决策。

## 2. 目录结构

```
sentry_match_recorder/
├── package.xml
├── setup.py / setup.cfg
├── resource/sentry_match_recorder              # ament 资源标记
├── sentry_match_recorder/
│   ├── __init__.py
│   ├── match_recorder_node.py                  # 状态机 + subprocess 管理
│   └── topics.yaml                             # 默认录制话题白名单（可被覆盖）
├── launch/match_recorder_launch.py             # 独立启动入口
├── config/recorder.yaml                        # 节点参数默认值
└── README.md
```

## 3. 工作原理

### 3.1 状态机

```
                game_progress != 4              game_progress == 4
                 ┌────────────────────┐          ┌──────────────────┐
                 ▼                    │          ▼                  │
       ┌─────────────────┐    去抖 N 帧 │  ┌──────────────────┐      │
       │      IDLE       │ ────────────┼─▶│    RECORDING     │ ─────┘
       │ 不录、轮询裁判帧 │              │  │ 一个 ros2 bag    │
       └─────────────────┘    去抖 N 帧  │  │ record 子进程    │
                ▲                       └──│ --max-bag-       │
                │ SIGINT 优雅停 / 异常自愈    │ duration 60      │
                └──────────────────────────│                  │
                                           └──────────────────┘
```

- 每收到一帧 `game_progress`，匹配 `target_progress`（默认 4）则 `consec_active++`，否则 `consec_inactive++`，互斥清零。
- 仅当连续 `debounce_count` 帧（默认 3）状态一致才发生跨态转换；防止裁判系统瞬时丢包/抖动导致误启停。
- 上升沿：`mkdir -p logs/match-bags/sortie_YMDHMS/` → spawn `ros2 bag record -o ... --max-bag-duration 60 <topics>`。
- 下降沿：对子进程组 `SIGINT`，等待 10s；超时则升级 `SIGTERM` 再等 5s；保证最后一个切片 flush 完整。

### 3.2 切片机制

走 **rosbag2 内建** `--max-bag-duration 60`：单 `ros2 bag record` 进程从头跑到比赛结束，rosbag2 自身按时长滚动 `*_0.mcap`、`*_1.mcap` …，全部落到同一个 `sortie_*/` 目录，**切片间零缝隙**，不会因为进程重启丢数据。

### 3.3 异常处理（watchdog 0.5s 周期）

| 场景 | 行为 |
|---|---|
| `ros2 bag record` 异常退出（rc != 0） | 记录 ERROR 日志，状态强制回 IDLE，下次上升沿重新创建新 sortie 目录 |
| 单场录制时长超过 `max_session_seconds`（默认 480s） | 强制 SIGINT 停录，防止裁判帧异常卡 4 漏停 |
| 节点退出（`Ctrl-C` 或 launch 关闭） | 在 `destroy_node()` / `main` 的 finally 里发 SIGINT 优雅停 |

## 4. ROS 接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/referee/game_status` | `rm_interfaces/msg/GameStatus` | Subscription | 状态机唯一输入；QoS = `qos_profile_sensor_data`（best-effort）匹配 `rm_serial_driver` |

录制行为通过 `ros2 bag record` 子进程实现，节点本身不发布任何业务话题。

## 5. 参数配置

参数定义于 `config/recorder.yaml`：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `record_dir` | `logs/match-bags` | 录制根目录（相对仓库根，可改绝对路径） |
| `sortie_prefix` | `sortie` | 单场目录前缀，最终目录名为 `<prefix>_YYYYMMDD_HHMMSS` |
| `slice_seconds` | `60` | 每个 `.mcap` 切片时长（秒） |
| `storage_id` | `mcap` | rosbag2 存储后端，`mcap` / `sqlite3` |
| `debounce_count` | `3` | 状态转换需要的连续帧数（裁判帧 ~10Hz → 0.3s） |
| `max_session_seconds` | `480` | 单场最长录制时长（8 分钟），超时强制停录 |
| `target_progress` | `4` | 触发录制的 `game_progress` 值（4 = 比赛中） |
| `topics_file` | `""` | 话题白名单 YAML 路径；空字符串走包内 share 中的 `topics.yaml` |
| `extra_topics` | `[]` | 在白名单基础上**追加**录制的话题 |
| `exclude_topics` | `[]` | 从合并后的列表中**剔除**的话题 |

最终录制话题 = `(topics_file 加载) + extra_topics − exclude_topics`，按出现顺序去重。

## 6. 默认录制话题（`topics.yaml`）

| 类别 | 话题 |
|---|---|
| TF + 云台关节 | `/tf`、`/tf_static`、`/serial/gimbal_joint_state` |
| 传感器原始 | `/livox/lidar_primary`、`/livox/lidar_secondary`、`/livox/imu_primary`、`/livox/imu_secondary` |
| 里程计 / 配准点云 | `/odometry`、`/lidar_odometry`、`/cloud_registered`、`/registered_scan` |
| Nav2 控制链 | `/plan`、`/local_plan`、`/received_global_plan`、`/cmd_vel_nav`、`/cmd_vel_chassis`、`/cmd_vel` |
| Costmap + 行为树 | `/global_costmap/costmap`、`/global_costmap/costmap_updates`、`/local_costmap/costmap`、`/local_costmap/costmap_updates`、`/behavior_tree_log` |
| 裁判系统全量 | `/referee/game_status`、`/referee/robot_status`、`/referee/all_robot_hp`、`/referee/rfidStatus` |

需要新增/裁剪话题时**优先**使用 `extra_topics` / `exclude_topics` 参数覆盖，而非直接修改 `topics.yaml`，便于不同场景共用一个仓库。

## 7. 编译与启动

### 7.1 编译

```bash
colcon build --packages-select sentry_match_recorder --symlink-install
source install/setup.bash
```

依赖：`rclpy`、`rm_interfaces`、`ros2bag`、`rosbag2_storage_mcap`。

### 7.2 启动方式

#### 方式 A：实车一键启动（推荐）

`rm_sentry_launch.py` 已内置 `enable_recorder` 开关，默认 `True`，并通过 `TimerAction(period=5.0)` 延迟启动以避开 nav 栈/串口刚启动的话题未就绪窗口：

```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py                      # 默认带录包
ros2 launch sentry_nav_bringup rm_sentry_launch.py enable_recorder:=False  # 临时禁用
```

#### 方式 B：独立启动（调试期）

```bash
ros2 launch sentry_match_recorder match_recorder_launch.py
# 覆盖参数：
ros2 launch sentry_match_recorder match_recorder_launch.py record_dir:=/data/match_bags
```

### 7.3 节点直接运行

```bash
ros2 run sentry_match_recorder match_recorder_node \
  --ros-args --params-file install/sentry_match_recorder/share/sentry_match_recorder/config/recorder.yaml
```

## 8. 输出目录结构

```
logs/match-bags/
├── sortie_20260521_143012/             # 一场比赛一个目录
│   ├── metadata.yaml                   # rosbag2 自动生成
│   ├── sortie_20260521_143012_0.mcap   # 0~60s
│   ├── sortie_20260521_143012_1.mcap   # 60~120s
│   ├── ...
│   └── sortie_20260521_143012_6.mcap   # 360~420s（一整场 7 个切片）
└── sortie_20260521_153504/             # 下一场
    └── ...
```

`logs/` 已加入 `.gitignore`，目录是本地产物。

## 9. 离线回放

### 9.1 单切片回放

```bash
ros2 bag play logs/match-bags/sortie_20260521_143012/sortie_20260521_143012_0.mcap
```

### 9.2 整场连续回放

`ros2 bag play` 直接传目录即可（rosbag2 ≥ Jazzy）：

```bash
ros2 bag play logs/match-bags/sortie_20260521_143012
```

切片之间无缝衔接，时间戳由 rosbag2 处理。

### 9.3 信息查看

```bash
ros2 bag info logs/match-bags/sortie_20260521_143012
```

## 10. 验证用例

| 场景 | 期望行为 |
|---|---|
| 节点启动后 `game_progress` 长期为 0 | 持续 IDLE，不创建任何 sortie 目录 |
| `game_progress: 0 → 4` | 0.3s 后（3 帧去抖）创建 `sortie_*/`，开始落 mcap |
| 录制中每过 `slice_seconds` | sortie 目录新增一个 `*_N.mcap`，无缝衔接 |
| `game_progress: 4 → 5`（正常结算） | SIGINT 优雅停，最后一个切片 flush 完整 |
| 中途 `ros2 bag record` 进程崩溃 | 节点状态回 IDLE，日志记录 ERROR；下次上升沿重新创建新 sortie |
| 一场超过 `max_session_seconds` 仍未停 | 强制 SIGINT 停录（防漏停） |

## 11. 注意事项

- **磁盘空间**：默认 26 个话题 + 双 Mid360 ~10Hz CustomMsg + LIO ~1kHz IMU，单场 7 分钟约占用 1–3 GB。比赛日前确认实车 IPC 剩余空间。
- **不要**用 `SIGKILL` 强杀 `ros2 bag record`，会导致最后一个 mcap 索引损坏；本节点已统一走 `SIGINT → SIGTERM` 升级路径。
- **多实例**：当前实现假设单节点单进程独占同一目录前缀；若同时跑两个实例（例如同时本地 + 录回放），需通过 `record_dir` / `sortie_prefix` 区分，否则 `mkdir` 不会冲突但会写到同一根目录显得混乱。
- **QoS 匹配**：默认录制 best-effort 话题（LiDAR、IMU 等）由 rosbag2 自动匹配 publisher 的 QoS，无需额外配置。如需精确控制，可加 `--qos-profile-overrides-path` 通过 `extra_topics` 派生扩展。
- **白名单未生效**：节点启动若打印 `failed to load topics file ...`，检查 `topics_file` 参数是否指向已编译安装的 share 路径；空字符串走包内默认值时不会有此告警。

# 运行模式与参数详解

> 本文档详细说明所有运行模式的启动方式、完整参数列表及调优建议。
> 快速上手请参阅 [快速部署指南](QUICKSTART.md)，系统架构请参阅 [架构详解](ARCHITECTURE.md)。

---

## 目录

- [1. 运行模式总览](#1-运行模式总览)
- [2. 仿真导航模式](#2-仿真导航模式)
- [3. 实车导航模式](#3-实车导航模式)
- [4. SLAM 建图模式](#4-slam-建图模式)
- [5. 行为树决策系统](#5-行为树决策系统)
- [6. 多机器人仿真](#6-多机器人仿真)
- [7. 辅助工具](#7-辅助工具)
- [8. Nav2 导航参数详解](#8-nav2-导航参数详解)
- [9. 定位模块参数详解](#9-定位模块参数详解)
- [10. 地形分析参数详解](#10-地形分析参数详解)
- [11. 串口通信参数](#11-串口通信参数)
- [12. 仿真与实车关键参数差异对照](#12-仿真与实车关键参数差异对照)
- [13. 常见调参场景](#13-常见调参场景)

---

## 1. 运行模式总览

系统支持以下运行模式，通过不同的 launch 文件和参数组合实现切换：

| 模式 | 启动文件 | 核心参数 | 用途 |
|:---|:---|:---|:---|
| 仿真导航 | `rm_navigation_simulation_launch.py` | `slam:=False` | Gazebo 仿真环境中的自主导航 |
| 仿真建图 | `rm_navigation_simulation_launch.py` | `slam:=True` | 在仿真中构建 2D 栅格地图和保存 PCD |
| 实车导航 | `rm_navigation_reality_launch.py` | `slam:=False` | 物理机器人的自主导航 |
| 实车建图 | `rm_navigation_reality_launch.py` | `slam:=True` | 物理机器人构建地图 |
| 行为树决策 | `sentry_behavior_launch.py` | `target_tree` | 比赛战术逻辑（独立启动） |
| 多机器人仿真 | `rm_multi_navigation_simulation_launch.py` | `robots` | 多机同场景仿真（实验性） |
| 手柄遥控 | 内嵌于导航 launch | 默认开启 | PS4 手柄控制 |

### Launch 层级关系

```
仿真模式:
  bringup_sim.launch.py (Gazebo 仿真器)
  └── rm_navigation_simulation_launch.py
      ├── ign_sim_pointcloud_tool (仿真专用点云格式转换)
      ├── bringup_launch.py
      │   ├── [slam=True]  → slam_launch.py
      │   ├── [slam=False] → localization_launch.py
      │   └── navigation_launch.py (始终启动)
      ├── joy_teleop_launch.py (手柄控制)
      └── rviz_launch.py (可视化，可选)

实车模式:
  rm_navigation_reality_launch.py
  ├── livox_ros_driver2 (实车激光雷达驱动)
  ├── robot_state_publisher_launch.py (可选)
  ├── bringup_launch.py
  │   ├── [slam=True]  → slam_launch.py
  │   ├── [slam=False] → localization_launch.py
  │   └── navigation_launch.py (始终启动)
  ├── joy_teleop_launch.py
  └── rviz_launch.py (可选)

行为树决策 (独立):
  sentry_behavior_launch.py
  ├── sentry_behavior_server (BT 执行服务)
  └── sentry_behavior_client (BT 目标选择客户端)
```

### 两层行为树架构

系统包含两个独立的行为树子系统，协作完成自主决策与导航：

```
sentry_behavior (游戏策略 BT)
    │  决定 "去哪里"
    │  PubGoal 发布目标到 /goal_pose
    v
Nav2 bt_navigator (导航 BT)
    │  决定 "怎么去"
    │  ComputePathToPose → FollowPath
    v
omni_pid_pursuit_controller (局部控制)
```

- **游戏策略层** (`sentry_behavior/behavior_trees/`): BTCPP_format="4"，根据裁判系统、视觉识别等信息决定巡逻路线和战术切换。
- **导航执行层** (`sentry_nav_bringup/behavior_trees/`): BTCPP_format="3"，Nav2 标准导航行为树，处理路径规划、跟踪和恢复。

---

## 2. 仿真导航模式

### 前置条件
- Gazebo Ignition Fortress 已安装
- 项目已编译且 `source install/setup.bash`
- 先验点云文件 (PCD) 已放置在 `sentry_nav_bringup/pcd/simulation/` 目录

### 启动步骤

**第 1 步：启动 Gazebo 仿真器（必须先启动）**

```bash
# Wayland 桌面环境必须设置 QT_QPA_PLATFORM=xcb
QT_QPA_PLATFORM=xcb ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

等待 Gazebo 窗口出现后，点击 Play 按钮启动仿真。若 Play 按钮无响应（Wayland 已知问题），使用命令行 unpause：
```bash
ign service -s /world/default/control \
  --reqtype ignition.msgs.WorldControl \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'pause: false'
```

> **重要**：必须确认仿真时钟已开始运行后，再启动导航栈。否则 Point-LIO 会因时间戳异常报错 `lidar loop back, clear buffer`。

**第 2 步：启动导航**

```bash
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False
```

### 完整参数列表

| 参数 | 类型 | 默认值 | 说明 |
|:---|:---|:---|:---|
| `namespace` | string | `red_standard_robot1` | 机器人命名空间，多机模式下区分不同机器人 |
| `slam` | bool | `False` | `True`=SLAM 建图模式，`False`=导航模式（使用先验地图定位） |
| `world` | string | `rmuc_2025` | 仿真世界名。可选：`rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025` |
| `map` | string | 自动推导 | 2D 栅格地图路径，默认 `map/simulation/{world}.yaml` |
| `prior_pcd_file` | string | 自动推导 | 先验 PCD 文件路径，默认 `pcd/simulation/{world}.pcd` |
| `use_sim_time` | bool | `True` | 使用 Gazebo 仿真时钟 |
| `params_file` | string | `config/simulation/nav2_params.yaml` | Nav2 参数配置文件 |
| `autostart` | bool | `true` | 自动启动 Nav2 生命周期节点 |
| `use_composition` | bool | `True` | 使用组合节点（共享进程，降低通信开销） |
| `use_respawn` | bool | `False` | 节点崩溃后是否自动重启 |
| `use_rviz` | bool | `True` | 是否启动 RViz 可视化 |
| `rviz_config_file` | string | `rviz/nav2_default_view.rviz` | RViz 配置文件路径 |

### 仿真专有节点
- **ign_sim_pointcloud_tool**: 将 Gazebo 的 `PointCloudPacked` 格式转换为标准 `PointCloud2`（`velodyne_points` 话题），供 Point-LIO 处理。仅仿真模式启动。

---

## 3. 实车导航模式

### 前置条件
- Livox MID360 激光雷达已连接并配置网络（默认 LiDAR IP: `192.168.1.150`，主机 IP: `192.168.1.50`）
- 串口设备已连接（默认 `/dev/ttyACM0`）
- 先验点云文件 (PCD) 已放置在 `sentry_nav_bringup/pcd/reality/` 目录
- 2D 栅格地图已放置在 `sentry_nav_bringup/map/reality/` 目录

### 启动命令

**导航模式（使用先验地图）：**
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  world:=rmul_2024 \
  slam:=False \
  use_robot_state_pub:=True
```

**建图模式：**
```bash
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True
```

### 完整参数列表

| 参数 | 类型 | 默认值 | 与仿真的区别 |
|:---|:---|:---|:---|
| `namespace` | string | `""` (空) | 仿真默认 `red_standard_robot1` |
| `slam` | bool | `False` | 同仿真 |
| `world` | string | `rmul_2024` | 仿真默认 `rmuc_2025` |
| `map` | string | `map/reality/{world}.yaml` | 使用 reality 子目录 |
| `prior_pcd_file` | string | `pcd/reality/{world}.pcd` | 使用 reality 子目录 |
| `use_sim_time` | bool | `False` | 使用系统真实时钟 |
| `params_file` | string | `config/reality/nav2_params.yaml` | 实车专用参数 |
| `use_robot_state_pub` | bool | `False` | 实车专有参数：是否启动 robot_state_publisher（仿真由 Gazebo 提供） |
| `use_rviz` | bool | `True` | 同仿真 |

### 实车专有节点
- **livox_ros_driver2**: Livox MID360 激光雷达驱动，仅实车模式启动。
- **robot_state_publisher**: 发布机器人 URDF/TF（仿真中由 Gazebo 提供，实车中需手动启用）。

### 网络配置

Livox MID360 网络配置文件位于 `sentry_nav_bringup/config/reality/mid360_user_config.json`：

| 配置项 | 默认值 | 说明 |
|:---|:---|:---|
| `host_net_info.cmd_data_ip` | `192.168.1.50` | 主机 PC 的 IP 地址 |
| `lidar_configs[0].ip` | `192.168.1.150` | LiDAR 的 IP 地址 |
| `lidar_configs[0].pcl_data_type` | `1` | 1=笛卡尔32位, 2=笛卡尔16位, 3=球坐标 |
| `lidar_configs[0].pattern_mode` | `0` | 0=非重复扫描, 1=重复扫描, 2=重复低频扫描 |

> **部署提醒**：IP 地址必须根据实际网络环境修改。使用网线直连 LiDAR 时，确保主机网卡在 `192.168.1.x` 网段。

---

## 4. SLAM 建图模式

### 工作原理

SLAM 模式通过 `slam:=True` 参数触发。与导航模式的主要区别：

| 组件 | 导航模式 (slam=False) | 建图模式 (slam=True) |
|:---|:---|:---|
| Point-LIO `prior_pcd.enable` | 取决于配置 | `False`（不加载先验地图） |
| Point-LIO `pcd_save.pcd_save_en` | `False` | `True`（退出时保存 PCD） |
| small_gicp_relocalization | 启动 | **不启动** |
| slam_toolbox | 不启动 | **启动** |
| map_saver_server | 不启动 | **启动** |
| map→odom TF | 由 small_gicp 动态发布 | 静态恒等变换 |
| pointcloud_to_laserscan | 不启动 | **启动**（为 slam_toolbox 提供 2D 扫描） |

### 启动命令

```bash
# 仿真建图
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py slam:=True

# 实车建图
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True
```

### 建图后保存地图

```bash
# 保存 2D 栅格地图（指定命名空间，仿真模式下需要）
ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r __ns:=/red_standard_robot1

# 实车模式（无命名空间）
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

保存后会生成两个文件：
- `my_map.pgm`: 栅格图像
- `my_map.yaml`: 地图元数据（分辨率、原点等）

> **提示**：Point-LIO 在 SLAM 模式下会自动保存 PCD 文件。该 PCD 可作为后续导航模式的先验点云使用。PCD 文件保存路径取决于 Point-LIO 的配置。

### SLAM Toolbox 关键参数

在 `nav2_params.yaml` 的 `slam_toolbox:` 段配置：

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `odom_frame` | `odom` | 里程计坐标系 |
| `map_frame` | `map` | 地图坐标系 |
| `base_frame` | `chassis` | 机器人基座坐标系 |
| `scan_topic` | `obstacle_scan` | 2D 扫描话题（由 pointcloud_to_laserscan 提供） |
| `resolution` | `0.05` | 地图分辨率 (m/像素) |
| `max_laser_range` | `10.0` | 最大激光有效距离 (m) |
| `minimum_travel_distance` | `0.5` | 触发地图更新的最小移动距离 (m) |
| `minimum_travel_heading` | `0.5` | 触发地图更新的最小旋转角度 (rad) |

---

## 5. 行为树决策系统

### 概述

行为树决策系统独立于导航栈运行，通过向 `/goal_pose` 话题发布目标点来驱动 Nav2 导航。

### 启动命令

```bash
# 仿真环境
ros2 launch sentry_behavior sentry_behavior_launch.py \
  namespace:=red_standard_robot1 \
  use_sim_time:=True

# 实车环境
ros2 launch sentry_behavior sentry_behavior_launch.py \
  use_sim_time:=False
```

### 服务端参数 (sentry_behavior_server)

配置文件：`sentry_behavior/params/sentry_behavior.yaml`

| 参数 | 类型 | 默认值 | 说明 |
|:---|:---|:---|:---|
| `action_name` | string | `sentry_behavior` | ROS2 Action 服务名 |
| `tick_frequency` | int | `2` | 行为树主循环频率 (Hz)。控制决策刷新速率 |
| `groot2_port` | int | `1667` | Groot2 可视化调试端口。设为 0 禁用 |
| `ros_plugins_timeout` | int | `1000` | ROS 插件初始化超时 (ms) |
| `use_cout_logger` | bool | `false` | 是否在终端打印节点状态切换日志 |
| `plugins` | list | `[sentry_behavior/bt_plugins]` | 插件库搜索路径 |
| `behavior_trees` | list | `[sentry_behavior/behavior_trees]` | 行为树 XML 搜索路径 |

### 客户端参数 (sentry_behavior_client)

| 参数 | 类型 | 默认值 | 说明 |
|:---|:---|:---|:---|
| `target_tree` | string | `red` | 要执行的行为树名（XML 中的 `<BehaviorTree ID="xxx">` 属性） |

### 可用行为树

| 文件 | 树名 (target_tree) | 适用场景 |
|:---|:---|:---|
| `RMUC.xml` | `red`, `blue` | RMUC 全场对抗赛，红/蓝方完整策略 |
| `RMUC.xml` | `red_down_1`, `red_down_2` | RMUC 不同阶段子策略 |
| `RMUC.xml` | `rmuc_yes`, `rmuc_no` | 前哨站存活/阵亡时的战术分支 |
| `rmul.xml` | `rmul` | RMUL 3v3 基础巡逻策略 |
| `rmul.xml` | `rmul1`, `rmul2` | RMUL 双点巡逻/激进策略 |
| `test.xml` | `test_uphill` | 单目标点测试（调试用） |

### 策略切换示例

运行时动态切换行为树（通过修改参数或重启客户端）：
```bash
# 方法 1: 修改 sentry_behavior.yaml 中的 target_tree 后重启
# 方法 2: 直接在 launch 参数中指定
ros2 launch sentry_behavior sentry_behavior_launch.py target_tree:=rmul
```

### Groot2 可视化调试

1. 启动行为树服务后，Groot2 ZMQ 服务会自动在 `groot2_port` (默认 1667) 监听
2. 打开 Groot2 应用，选择 "Connect to running tree"
3. 输入 `localhost:1667`（或远程 IP）
4. 可实时查看：
   - 行为树执行路径和节点状态（Success/Failure/Running）
   - 全局黑板变量的实时值
   - 各节点的端口参数

### 自定义行为树节点一览

#### 条件节点 (Condition Nodes)

| 节点名 | 黑板键 | 说明 | 关键端口 |
|:---|:---|:---|:---|
| `IsGameStatus` | `@referee_gameStatus` | 检查比赛阶段和剩余时间 | `expected_game_progress` (int), `min_remain_time` (int) |
| `IsStatusOK` | `@referee_robotStatus` | 检查弹药、热量、血量是否满足阈值 | `ammo_min` (int), `heat_max` (int), `hp_min` (int) |
| `IsRfidDetected` | `@referee_rfidStatus` | 检查是否在指定 RFID 区域 | `rfid_type` (string) |
| `IsOutpostOk` | `@referee_allRobotHP` | 检查己方前哨站是否存活 | `key_port` (GameRobotHP) |
| `IsAttacked` | `@referee_robotStatus` | 检测受击，输出云台方向 | 输出: `gimbal_yaw`, `gimbal_pitch` |
| `IsDetectEnemy` | `@detector_armors` | 检查视觉系统是否检测到敌方 | `armor_id_list` (string), `max_distance` (float) |
| `IsHPAdd` | `@referee_robotStatus` | 检测血量是否正在增加（正在补血） | `key_port` (RobotStatus) |

#### 动作节点 (Action Nodes)

| 节点名 | 说明 | 关键端口 |
|:---|:---|:---|
| `PubGoal` | 向 Nav2 发布目标位姿 | `goal_pose_x` (float), `goal_pose_y` (float), `topic_name` (string, 默认 `/goal_pose`) |
| `Pursuit` | 利用 TF2 追踪敌方目标 | `key_port` (tracker_target), `topic_name` (string) |
| `BattlefieldInformation` | 分析全场血量，输出战术权重 | `key_port` (GameRobotHP) → 输出 `weight` (0/1/2，用于 Switch3 路由) |

#### 控制与装饰节点

| 节点名 | 说明 | 关键端口 |
|:---|:---|:---|
| `RecoveryNode` | 双子节点恢复控制：子节点1失败则运行子节点2，然后重试 | `num_attempts` (int, 默认 999) |
| `RateController` | 限制子节点 tick 频率 | `hz` (float, 默认 10) |
| `TickAfterTimeout` | 上次成功后延迟指定秒数再 tick | `timeout` (float, 秒) |

### 黑板数据来源

行为树服务端 (`sentry_behavior_server`) 订阅以下话题，并将数据写入全局黑板：

| 黑板键 (使用 `@` 前缀访问) | 订阅话题 | 消息类型 |
|:---|:---|:---|
| `referee_gameStatus` | `referee/game_status` | `rm_interfaces/GameStatus` |
| `referee_robotStatus` | `referee/robot_status` | `rm_interfaces/RobotStatus` |
| `referee_rfidStatus` | `referee/rfid_status` | `rm_interfaces/RfidStatus` |
| `referee_allRobotHP` | `referee/all_robot_hp` | `rm_interfaces/GameRobotHP` |
| `detector_armors` | `detector/armors` | `auto_aim_interfaces/Armors` |
| `tracker_target` | `tracker/target` | `auto_aim_interfaces/Target` |
| `nav_globalCostmap` | `global_costmap/costmap` | `nav2_msgs/Costmap` |

---

## 6. 多机器人仿真

> **实验性功能**：多机模式仍在开发中，可能存在 TF 冲突或资源竞争问题。

### 启动命令

```bash
ros2 launch sentry_nav_bringup rm_multi_navigation_simulation_launch.py \
  world:=rmuc_2025 \
  robots:="robot1={x: 1.0, y: 1.0, yaw: 1.5707}; robot2={x: -1.0, y: -1.0, yaw: 0.0}"
```

每个机器人会获得独立的：
- 命名空间 (如 `/robot1`, `/robot2`)
- Nav2 导航栈实例
- RViz 可视化窗口

### 参数

| 参数 | 类型 | 说明 |
|:---|:---|:---|
| `robots` | string | 机器人初始位姿。格式: `"name={x: X, y: Y, yaw: YAW}; ..."` |
| `world` | string | 仿真世界名 |

---

## 7. 辅助工具

### 7.1 手柄遥控

导航 launch 文件默认内嵌启动手柄控制节点。也可独立启动：

```bash
ros2 launch sentry_nav_bringup joy_teleop_launch.py
```

PS4 手柄键位映射在 `nav2_params.yaml` 的 `teleop_twist_joy_node:` 段配置。

### 7.2 地图坐标拾取工具

使用 matplotlib 可视化地图并交互式拾取坐标（用于设定行为树中的目标点）：

```bash
ros2 launch location map_visualizer_launch.py
```

### 7.3 独立模块启动

以下模块可独立启动，用于调试或单元测试：

| 模块 | 启动命令 |
|:---|:---|
| Point-LIO 里程计 | `ros2 launch point_lio point_lio.launch.py` |
| 里程计桥接 | 已集成到 navigation_launch.py 中 (odom_bridge composable node) |
| GICP 重定位 | `ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py` |
| Livox 驱动 | `ros2 launch livox_ros_driver2 msg_MID360_launch.py` |
| 速度变换 | `ros2 launch fake_vel_transform fake_vel_transform_launch.py` |
| 里程计插值 | `ros2 launch odom_interpolator odom_interpolator_launch.py` |
| 机器人描述 | `ros2 launch sentry_robot_description robot_description_launch.py` |
| 串口驱动 | `ros2 launch serial_driver serial_driver.launch.py` |

---

## 8. Nav2 导航参数详解

参数文件位于：
- 仿真：`sentry_nav_bringup/config/simulation/nav2_params.yaml`
- 实车：`sentry_nav_bringup/config/reality/nav2_params.yaml`

### 8.1 全局规划器

**仿真使用 SmacPlannerHybrid，实车使用 ThetaStarPlanner**（实车配置中 SmacPlannerHybrid 已注释保留）。

#### SmacPlannerHybrid (仿真)

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `motion_model_for_search` | `DUBIN` | 运动模型。虽然是全向底盘，Dubin 可生成更自然的路径 |
| `minimum_turning_radius` | `0.05` | 最小转弯半径 (m)，设为极小值模拟全向移动 |
| `angle_quantization_bins` | `64` | 角度搜索离散化精度 |
| `cost_travel_multiplier` | `2.0` | 路径长度代价权重 |
| `max_iterations` | `1000000` | 最大搜索迭代次数 |
| `lookup_table_size` | `20.0` | 查找表大小 |
| `smoother.max_iterations` | `1000` | 路径平滑迭代次数 |
| `smoother.w_smooth` | `0.3` | 平滑权重 |
| `smoother.w_data` | `0.2` | 数据保真权重 |

#### ThetaStarPlanner (实车)

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `how_many_corners` | `8` | 搜索方向数量 (4 或 8) |
| `w_euc_cost` | `1.0` | 欧几里得距离代价权重 |
| `w_traversal_cost` | `2.0` | 代价地图穿越代价权重 |
| `terminal_checking_interval` | `5000` | 终端检查间隔 |

### 8.2 局部控制器 - OmniPidPursuitController

仿真和实车均使用此控制器：

| 参数 | 仿真 | 实车 | 说明 |
|:---|:---|:---|:---|
| `lookahead_dist` | `2.0` | `2.0` | 前瞻距离 (m) |
| `v_linear_max` | `2.5` | `2.5` | 最大线速度 (m/s) |
| `v_angular_max` | `3.0` | `3.0` | 最大角速度 (rad/s) |
| `curvature_max` | `1.0` | `1.0` | 触发减速的最大曲率阈值 |
| `reduction_ratio` | `0.5` | `0.5` | 高曲率时速度缩减比例 |
| `enable_rotation` | `false` | `false` | 是否旋转到目标朝向（全向底盘不需要） |
| `target_frame` | `gimbal_yaw_fake` | `gimbal_yaw_fake` | 控制器参考坐标系 |

### 8.3 代价地图 (Costmap2D)

#### 局部代价地图 (Local Costmap)

| 参数 | 仿真 | 实车 | 说明 |
|:---|:---|:---|:---|
| `update_frequency` | `10.0` | `30.0` | 更新频率 (Hz) |
| `publish_frequency` | `5.0` | `30.0` | 发布频率 (Hz) |
| `width` | `5.0` | `5.0` | 滚动窗口宽度 (m) |
| `height` | `5.0` | `5.0` | 滚动窗口高度 (m) |
| `resolution` | `0.05` | `0.05` | 分辨率 (m/像素) |
| `robot_radius` | `0.2` | `0.24` | 机器人外接圆半径 (m) |
| `inflation_radius` | `0.7` | `0.5` | 膨胀半径 (m) |
| `cost_scaling_factor` | `4.0` | `4.0` | 代价随距离衰减的指数因子 |

**插件层顺序**：`static_layer` → `IntensityVoxelLayer` → `inflation_layer`

- **IntensityVoxelLayer**: 自定义插件 (位于 `nav2_plugins` 包)，订阅 `terrain_map` 话题，利用点云强度信息标记 3D 障碍物。

#### 全局代价地图 (Global Costmap)

与局部代价地图参数结构相同，但：
- 覆盖整个场地（非滚动窗口）
- IntensityVoxelLayer 订阅 `terrain_map_ext` 话题（更大感知范围）

### 8.4 速度平滑器 (Velocity Smoother)

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `smoothing_frequency` | `20.0` | 平滑处理频率 (Hz) |
| `max_velocity` | `[2.5, 2.5, 3.0]` | 最大速度 [vx, vy, vθ] (m/s, m/s, rad/s) |
| `min_velocity` | `[-2.5, -2.5, -3.0]` | 最小速度（反向） |
| `max_accel` | `[4.5, 4.5, 5.0]` | 最大加速度 [ax, ay, aθ] |
| `max_decel` | `[-4.5, -4.5, -5.0]` | 最大减速度 |
| `feedback` | `OPEN_LOOP` | 反馈模式。`OPEN_LOOP` 不依赖里程计反馈 |

### 8.5 恢复行为插件

| 插件名 | 说明 |
|:---|:---|
| `Spin` | 原地旋转以重新观测环境 |
| `BackUpFreeSpace` | 自定义插件：向自由空间方向后退 |
| `DriveOnHeading` | 沿当前朝向直行一段距离 |
| `Wait` | 原地等待指定时间 |
| `AssistedTeleop` | 辅助遥控模式 |

### 8.6 Nav2 导航行为树

| 文件 | 用途 |
|:---|:---|
| `navigate_to_pose_w_replanning_and_recovery.xml` | 单目标导航：3Hz 重规划 + 代价地图清除 + 后退恢复 |
| `navigate_through_poses_w_replanning_and_recovery.xml` | 多航点导航：同上 + 已通过航点自动移除 (radius=0.7m) |

---

## 9. 定位模块参数详解

### 9.1 Point-LIO 里程计

| 参数 | 仿真 | 实车 | 说明 |
|:---|:---|:---|:---|
| `common.lid_topic` | `velodyne_points` | `livox/lidar` | LiDAR 输入话题 |
| `common.imu_topic` | `livox/imu` | `livox/imu` | IMU 输入话题 |
| `preprocess.lidar_type` | `2` (Velodyne) | `1` (Livox) | LiDAR 类型: 1=Livox, 2=Velodyne, 3=Ouster |
| `preprocess.scan_line` | `32` | `4` | 扫描线数（MID360 实际为 4 线） |
| `preprocess.timestamp_unit` | `2` (微秒) | `3` (纳秒) | 时间戳单位: 0=秒, 1=毫秒, 2=微秒, 3=纳秒 |
| `preprocess.blind` | `0.5` | `0.2` | 盲区半径 (m)，过滤近距离噪声 |
| `mapping.acc_norm` | `9.81` | `1.0` | 加速度单位：1.0=g, 9.81=m/s² |
| `mapping.satu_acc` | `30.0` | `4.0` | IMU 加速度计饱和值 |
| `mapping.gravity` | `[0, -4.9, -8.49]` | `[2.64, 0, -9.68]` | **重力向量，必须匹配 LiDAR 安装倾角** |
| `mapping.gravity_init` | `[0, -4.9, -8.49]` | `[2.64, 0, -9.68]` | 初始重力估计 |
| `mapping.extrinsic_T` | `[0, 0, 0]` | `[-0.011, -0.023, 0.044]` | LiDAR-IMU 外参平移 (m) |
| `mapping.lidar_meas_cov` | `0.001` | `0.01` | LiDAR 测量协方差 |
| `filter_size_surf` | `0.2` | `0.2` | 表面特征降采样步长 (m) |
| `filter_size_map` | `0.2` | `0.2` | 地图降采样步长 (m) |
| `publish.tf_send_en` | `False` | `False` | TF 发布（禁用，由 odom_bridge 处理） |
| `pcd_save.pcd_save_en` | `False` | `False` | 退出时保存 PCD（SLAM 模式自动设为 True） |

> **调参警告**：`gravity` 向量必须精确匹配 LiDAR 的物理安装角度。仿真中 LiDAR 倾斜约 30° (`rpy=[0, pi/6, 0]`)，故重力分量为 `[0, -4.9, -8.49]`。实车需通过实际标定获得。错误的重力向量会导致里程计快速发散。

### 9.2 Small GICP 重定位

| 参数 | 仿真 | 实车 | 说明 |
|:---|:---|:---|:---|
| `num_threads` | `4` | `8` | OpenMP 并行线程数 |
| `num_neighbors` | `20` | `20` | 协方差估计邻域点数 |
| `global_leaf_size` | `0.25` | `0.2` | 先验地图体素降采样步长 (m) |
| `registered_leaf_size` | `0.25` | `0.1` | 累积扫描体素降采样步长 (m) |
| `max_dist_sq` | **`9.0`** | **`3.0`** | GICP 对应点最大距离平方 (m²)。**仿真点云稀疏，需更大值** |
| `max_iterations` | **`50`** | **`20`** | GICP 优化器最大迭代次数 |
| `accumulated_count_threshold` | **`50`** | **`20`** | 累积多少帧点云后触发一次配准 |
| `min_range` | **`0.3`** | **`0.5`** | 最小点云距离过滤 (m) |
| `min_inlier_ratio` | **`0.1`** | **`0.3`** | 最小内点比率。**仿真需更低阈值** |
| `max_fitness_error` | **`5.0`** | **`1.0`** | 最大每内点适配误差。**仿真需更高容忍度** |
| `enable_periodic_relocalization` | `false` | `false` | 是否启用周期性重定位 |
| `relocalization_interval` | `30.0` | `30.0` | 周期性重定位间隔 (秒) |
| `map_frame` | `map` | `map` | 地图坐标系 |
| `odom_frame` | `odom` | `odom` | 里程计坐标系 |
| `prior_pcd_file` | (由 launch 传入) | (由 launch 传入) | 先验 PCD 地图文件路径 |

**质量门控机制**：
1. GICP 必须报告收敛
2. 内点比率 ≥ `min_inlier_ratio`
3. 平均适配误差 ≤ `max_fitness_error`
4. 周期性修正幅度 < 2m（硬编码上限）
5. **2D 约束**：输出的 map→odom 仅包含 (x, y, yaw)，z/roll/pitch 强制置零

> **关键设计**：仿真环境点云远比实车稀疏，因此 GICP 相关阈值需大幅放宽。修改这些参数时，**必须同时更新** `config/simulation/` 和 `config/reality/` 两套配置。

### 9.3 Livox MID360 驱动参数

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `xfer_format` | `4` | 数据格式: 0=PointCloud2, 1=CustomMsg, 4=AllMsg(推荐) |
| `publish_freq` | `30.0` | 发布频率 (Hz)。可选: 5, 10, 20, 30, 50 |
| `frame_id` | `front_mid360` | TF 坐标系名称 |
| `multi_topic` | `0` | 0=共享话题, 1=每个 LiDAR 独立话题 |

---

## 10. 地形分析参数详解

### 10.1 Terrain Analysis (局部地形)

订阅 `sensor_scan` + `odometry`，发布 `terrain_map` → 供**局部代价地图** IntensityVoxelLayer 使用。

| 参数 | 仿真 | 实车 | 说明 |
|:---|:---|:---|:---|
| `scanVoxelSize` | `0.05` | `0.05` | 点云降采样步长 (m) |
| `decayTime` | `0.5` | `0.5` | 点云衰减时间 (秒)，超过此时间的点被丢弃 |
| `noDecayDis` | `0.0` | `0.0` | 此距离内的点不衰减 (m) |
| `clearingDis` | `0.0` | `0.0` | 超出此距离的点被清除 (m)，0=禁用 |
| `useSorting` | `true` | `true` | 使用分位数地面估计（支持坡道识别）。**不可与 considerDrop 同时启用** |
| `quantileZ` | `0.2` | `0.2` | Z 方向分位数（仅 useSorting=true 时生效） |
| `considerDrop` | `false` | `false` | 考虑凹地形（绝对高度差） |
| `clearDyObs` | `true` | `true` | 清除动态障碍物 |
| `minDyObsDis` | `0.3` | `0.5` | 动态障碍物最小检测距离 (m) |
| `vehicleHeight` | `0.5` | `0.55` | 机器人高度 (m)，仅处理低于此高度的点 |
| `minRelZ` | `-1.5` | `-1.5` | 有效点最低相对高度 (m) |
| `maxRelZ` | `0.5` | `0.5` | 有效点最高相对高度 (m) |
| `disRatioZ` | `0.2` | `0.2` | Z 范围随距离缩放因子（坡道补偿） |
| `minBlockPointNum` | `10` | `10` | 每个体素块最少点数 |
| `noDataObstacle` | `false` | `false` | 无数据区域视为障碍物 |

### 10.2 Terrain Analysis Ext (全局地形)

订阅 `terrain_map`，发布 `terrain_map_ext` → 供**全局代价地图** IntensityVoxelLayer 使用。

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `scanVoxelSize` | `0.1` | 更粗的降采样（全局范围） |
| `decayTime` | `0.2` | 更快衰减以保持全局地图新鲜度 |
| `clearingDis` | `20.0` | 扩展清除距离 (m) |
| `vehicleHeight` | `1.0` | 更高的过滤阈值 |
| `localTerrainMapRadius` | `4.0` | 局部地形地图半径 (m) |
| `ceilingFilteringThre` | `2.0` | 天花板过滤阈值 (m) |
| `checkTerrainConn` | `false` | 检查地形连通性 |

---

## 11. 串口通信参数

### 配置文件

`serial/serial_driver/config/serial_driver.yaml`

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `device_name` | `/dev/ttyACM0` | 串口设备路径 |
| `baud_rate` | `115200` | 波特率 |
| `flow_control` | `none` | 流控: `none`, `hardware`, `software` |
| `parity` | `none` | 校验: `none`, `odd`, `even` |
| `stop_bits` | `"1"` | 停止位: `1`, `1.5`, `2` |

### 通信协议

**上行 (MCU → ROS2)**：`ReceiveNavPacket`，帧头 `0x5B`，CRC16 校验

| 字段 | 类型 | 说明 |
|:---|:---|:---|
| `pitch`, `yaw` | float | 云台关节角度 |
| `game_progress` | uint8 | 比赛阶段 (0-5) |
| `stage_remain_time` | uint16 | 当前阶段剩余时间 (秒) |
| `current_hp` | uint16 | 机器人当前血量 |
| `projectile_allowance_17mm` | uint16 | 剩余弹药量 |
| `red_*/blue_*_robot_hp` | uint16 | 全场 14 台机器人血量 |
| `team_colour` | uint16 | 队伍颜色 (1=红方) |
| `tracker_x`, `tracker_y` | float | 追踪目标位置 |
| `tracking` | bool | 是否正在追踪 |

**下行 (ROS2 → MCU)**：`SendNavPacket`，帧头 `0xB5`，CRC16 校验

| 字段 | 类型 | 说明 |
|:---|:---|:---|
| `vel_x`, `vel_y` | float | 线速度指令 (m/s) |
| `vel_v` | float | 角速度指令（映射自 `cmd_vel.angular.z`） |

### 发布话题

| 话题 | 消息类型 | 说明 |
|:---|:---|:---|
| `serial/gimbal_joint_state` | `JointState` | 云台 pitch/yaw 关节状态 |
| `referee/game_status` | `GameStatus` | 比赛阶段 + 剩余时间 |
| `referee/robot_status` | `RobotStatus` | 血量 + 弹药 |
| `referee/all_robot_hp` | `GameRobotHP` | 全场机器人血量 + 队伍颜色 |
| `referee/rfid_status` | `RfidStatus` | RFID 区域检测 |
| `tracker/target` | `Target` | 追踪目标位姿 |

---

## 12. 仿真与实车关键参数差异对照

以下汇总了仿真和实车之间最关键的参数差异，部署时务必确认使用正确的配置文件：

| 模块 | 参数 | 仿真 | 实车 | 差异原因 |
|:---|:---|:---|:---|:---|
| **Point-LIO** | `lidar_type` | `2` (Velodyne) | `1` (Livox) | 仿真使用转换后的 Velodyne 格式 |
| | `lid_topic` | `velodyne_points` | `livox/lidar` | 话题名不同 |
| | `scan_line` | `32` | `4` | 仿真模拟的扫描线数不同 |
| | `timestamp_unit` | `2` (微秒) | `3` (纳秒) | 时间戳格式不同 |
| | `gravity` | `[0, -4.9, -8.49]` | `[2.64, 0, -9.68]` | LiDAR 安装角度不同 |
| | `acc_norm` | `9.81` | `1.0` | 加速度单位不同 |
| **GICP** | `max_dist_sq` | `9.0` | `3.0` | 仿真点云稀疏 |
| | `min_inlier_ratio` | `0.1` | `0.3` | 仿真匹配率低 |
| | `max_fitness_error` | `5.0` | `1.0` | 仿真误差大 |
| | `max_iterations` | `50` | `20` | 仿真需更多迭代 |
| | `num_threads` | `4` | `8` | 实车算力更强 |
| **Nav2** | `controller_frequency` | `20.0` | `60.0` | 实车需要更高控制频率 |
| | `costmap update_frequency` | `10.0` | `30.0` | 实车实时性要求更高 |
| | `planner plugin` | `SmacPlannerHybrid` | `ThetaStarPlanner` | 不同规划策略 |
| | `robot_radius` | `0.2` | `0.24` | 实车尺寸略大 |
| | `inflation_radius` | `0.7` | `0.5` | 实车场地较小 |
| **地形** | `vehicleHeight` | `0.5` | `0.55` | 实车高度不同 |
| | `minDyObsDis` | `0.3` | `0.5` | 实车动态障碍检测距离更大 |

---

## 13. 常见调参场景

### 场景 1: 机器人频繁撞墙

- 增大 `inflation_radius`（如 0.5 → 0.8）
- 增大 `robot_radius`（确保覆盖实际外形）
- 降低 `v_linear_max`（减速增加反应时间）
- 检查 `costmap update_frequency` 是否足够高

### 场景 2: 路径规划失败 / 找不到路径

- 减小 `inflation_radius`（给规划器留出更多通行空间）
- 检查静态地图是否准确反映实际环境
- 增大 `SmacPlannerHybrid.max_iterations` 或降低 `cost_travel_multiplier`
- 确认 `robot_radius` 不大于最窄通道宽度的一半

### 场景 3: 重定位不收敛

- 增大 `max_dist_sq`（放宽对应点搜索范围）
- 降低 `min_inlier_ratio`（降低匹配质量要求）
- 增大 `max_fitness_error`（提高容忍度）
- 增大 `accumulated_count_threshold`（累积更多帧再配准）
- 确认先验 PCD 地图与当前环境一致

### 场景 4: Point-LIO 里程计发散

- **首先检查 `gravity` 向量**是否匹配 LiDAR 实际安装角度
- 确认 `lidar_type` 和 `timestamp_unit` 设置正确
- 检查 IMU 数据质量，调整 `satu_acc` / `satu_gyro`
- 增大 `lidar_meas_cov`（降低 LiDAR 测量信任度）

### 场景 5: 行为树不执行 / 无动作

- 确认裁判系统话题正在发布数据：`ros2 topic echo /referee/game_status`
- 检查 `target_tree` 参数是否指向正确的树名
- 启用 Groot2 调试：连接到 `localhost:1667` 查看节点执行状态
- 设置 `use_cout_logger: true` 查看终端日志
- 确认 Nav2 导航栈已正常启动（行为树通过 `/goal_pose` 发布目标）

### 场景 6: 仿真启动后 Point-LIO 报错 "lidar loop back"

- 这是启动时序问题：**必须先启动 Gazebo 并 unpause，等仿真时钟稳定后再启动导航栈**
- 通常 IMU 初始化完成后会自行恢复，可等待 10-15 秒

### 场景 7: 需要切换比赛策略

1. 修改 `sentry_behavior.yaml` 中的 `target_tree` 值
2. 或在 launch 命令中指定：`ros2 launch sentry_behavior sentry_behavior_launch.py target_tree:=rmul1`
3. 可选树名参见[第 5 节 - 可用行为树](#可用行为树)

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

RoboMaster 哨兵机器人 ROS2 自主导航系统，基于 Ubuntu 22.04 + ROS2 Humble。
**完整的包职责、架构决策、编码规范、踩坑记录见 `AGENTS.md`（必读）。历次 AI 会话的决策记录见 `MEMORY.md`（必读）。**

## Build & Run

```bash
# 全量编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 单包编译
colcon build --packages-select <pkg> --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 测试
colcon test && colcon test-result --verbose

# 一键环境配置（首次）
bash src/scripts/setup_env.sh
```

- 禁止使用 catkin；始终带 `--symlink-install`（参数文件改动免重编）。
- 修改 C++ 源码后如增量编译未触发，执行 `rm -rf build/<pkg>` 强制重编。
- 调试时切换 `-DCMAKE_BUILD_TYPE=Debug`；默认 Release。

## Quick Start

```bash
# 仿真（Wayland 需加 QT_QPA_PLATFORM=xcb）
QT_QPA_PLATFORM=xcb ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
# 确保 Gazebo unpause 后：
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmul_2026 slam:=False

# 实车建图
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True

# 实车导航
ros2 launch sentry_nav_bringup rm_navigation_reality_launch.py world:=<WORLD_NAME> slam:=False use_robot_state_pub:=True

# 实车一键启动
ros2 launch sentry_nav_bringup rm_sentry_launch.py

# 无 GUI 仿真（更稳定）
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py headless:=true

# Gazebo 命令行 unpause（Wayland Play 按钮无响应时）
ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 5000 --req 'pause: false'
```

## Key Architecture

**TF 链**: `map → odom → chassis → gimbal_yaw → gimbal_pitch → front_mid360`

**数据流**:
```
Point-LIO → odom_bridge → TF/Odometry/registered_scan/lidar_odometry
                        → small_gicp_relocalization → map→odom TF
                        → terrain_analysis/ext → costmap
Nav2 controller_server → cmd_vel_controller → velocity_smoother
  → cmd_vel_nav2_result → fake_vel_transform (+spin 3.14 rad/s) → /cmd_vel → 底盘/串口
```

**速度话题对比必须用** `cmd_vel_nav2_result`（world 系）vs `/odometry` twist（world 系）。  
**绝对不能用** `/cmd_vel`（已旋转到 body 系 + 叠加 spin_speed）。

**串口协议单源**: `serial/serial_driver/protocol/protocol.yaml` → `python3 generate.py` → 生成 C++/C/Python 代码。修改协议后必须重新生成并复制到对应位置。

## Common Modification Points

| 任务 | 文件位置 |
|---|---|
| 导航参数 | `sentry_nav_bringup/config/simulation/nav2_params.yaml` 或 `config/reality/nav2_params.yaml` |
| 行为树逻辑 | `sentry_behavior/plugins/` + 对应 `CMakeLists.txt`（用 `BT_PLUGIN_EXPORT` 宏） |
| 串口协议 | `serial/serial_driver/protocol/protocol.yaml` → 执行 `generate.py` |
| 机器人模型 | `sentry_robot_description/resource/xmacro/` |
| 仿真世界 | `simulator/` 下的 SDF 文件（`<cast_shadows>` 必须为 `false`） |

## Critical Rules

- **`controller_frequency` 和 `smoothing_frequency` 必须一致**（实车 30Hz，仿真 20Hz）。频率不能超过 CPU 实际能跑到的值。
- **`enable_periodic_relocalization` 必须为 `true`**，否则初始定位后没有持续纠偏机制。
- **`min_lookahead_dist` 必须 ≥ 1.0m**，否则低速时 velocity-scaled lookahead 形成正反馈死循环。
- **仿真启动顺序**：先启动 Gazebo 并 unpause，再启动导航栈。
- **PCD 和 2D 地图必须在同一坐标系（odom 系）**，从同一出生点建图。旧 PCD（lidar_odom 系）不可直接用于导航。
- **仿真渲染**：`gui.config` 用 `ogre`，SDF `<render_engine>` 用 `ogre2`（gpu_lidar 只在 ogre2 下工作）。

## Coding Conventions

- C++17，ament_cmake，各子包自带 `.clang-format`（无根目录统一配置）。
- 包名 snake_case；C++ 命名空间与包名一致；头文件保护 `PACKAGE_NAME__FILE_NAME_HPP_`。
- 话题名 snake_case（如 `referee/game_status`）；launch 文件 Python 编写，文件名 `_launch.py`。
- 推荐 Composable Node + rclcpp_components；日志用 `RCLCPP_ERROR/WARN/INFO`。

## Output Language

**与用户交互、注释讨论、commit message 优先使用中文。代码标识符和代码注释使用英文。**

## Documentation Sync (Mandatory)

任何代码或功能变更必须在**同一次修改中**同步更新相关文档。`AGENTS.md` 中的包职责/文件索引必须与实际代码一致；详细同步规则见 `AGENTS.md` 第 10 节。

## Key Files Index

- 仿真启动: `src/sentry_nav_bringup/launch/rm_navigation_simulation_launch.py`
- 实车启动: `src/sentry_nav_bringup/launch/rm_navigation_reality_launch.py`
- 实车一键: `src/sentry_nav_bringup/launch/rm_sentry_launch.py`
- 仿真导航参数: `src/sentry_nav_bringup/config/simulation/nav2_params.yaml`
- 实车导航参数: `src/sentry_nav_bringup/config/reality/nav2_params.yaml`
- 串口协议定义: `src/serial/serial_driver/protocol/protocol.yaml`
- 协议代码生成器: `src/serial/serial_driver/protocol/generate.py`
- 调试工具箱: `src/sentry_tools/sentry_toolbox.py`
- 串口数据可视化: `src/sentry_tools/serial_visualizer.py`
- 架构详解: `src/docs/ARCHITECTURE.md`
- 快速部署: `src/docs/QUICKSTART.md`
- 参数调优: `src/docs/TUNING_GUIDE.md`
- 运行模式说明: `src/docs/RUNNING_MODES.md`
- 远程调试: `src/docs/REMOTE_DEBUG.md`

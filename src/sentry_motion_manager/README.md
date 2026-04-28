# sentry_motion_manager

`sentry_motion_manager` 是哨兵底盘运动管理器的基础包。它位于 Nav2 / recovery / terrain / evasion / manual 等上游速度源与最终 `/cmd_vel` 执行链路之间，用于统一做速度仲裁、恢复状态机、Nav2 适配和安全限幅。

节点会订阅各类计划速度源，但默认 `command_output_enabled=false`，因此输出始终为零速 `geometry_msgs/msg/TwistStamped`。后续功能分支可以在不重写包结构的前提下扩展 `MotionSource`、`MotionMode`、`RecoveryPhase`、`MotionCommand` 和 `MotionState`。

## 节点

- 可组合节点：`sentry_motion_manager::MotionManagerNode`
- 可执行入口：`sentry_motion_manager_node`
- 默认节点名：`motion_manager`

## 频率设计

| 环节 | 默认频率 | 说明 |
| --- | --- | --- |
| manager tick | 100 Hz | 内部仲裁与状态更新入口，给恢复/安全状态机预留高频检查能力 |
| output command | 50 Hz | 对下游 `/cmd_vel` 发布平滑、稳定的 TwistStamped；低于底盘闭环、高于 Nav2 控制输出 |
| state publish | 10 Hz | 发布轻量状态摘要，供调试和上层行为树观察 |
| diagnostics | 2 Hz | 发布基础诊断，避免诊断流量影响控制链路 |
| Nav2 controller | 20-30 Hz | 现有 Nav2 RPP/RotationShim 控制频率范围 |
| chassis loop | 200 Hz+ | 下位机闭环频率，运动管理器不替代底盘控制器 |

## 话题契约

### 订阅

| 参数 | 默认话题 | 类型 | 预期来源 |
| --- | --- | --- | --- |
| `nav_command_topic` | `cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | Nav2 / velocity smoother 输出 |
| `recovery_command_topic` | `cmd_vel_recovery` | `geometry_msgs/msg/TwistStamped` | 后续恢复状态机 |
| `terrain_command_topic` | `cmd_vel_terrain` | `geometry_msgs/msg/TwistStamped` | 后续地形通过策略 |
| `evasion_command_topic` | `cmd_vel_evasion` | `geometry_msgs/msg/TwistStamped` | 后续规避/扰动策略 |
| `manual_command_topic` | `cmd_vel_manual` | `geometry_msgs/msg/TwistStamped` | 手动/遥控接管 |
| `emergency_stop_topic` | `emergency_stop` | `std_msgs/msg/Bool` | 急停输入，`true` 时强制零速 |

### 发布

| 参数 | 默认话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| `output_command_topic` | `cmd_vel` | `geometry_msgs/msg/TwistStamped` | 下游 `rm_serial_driver` / Gazebo DiffDrive 消费的最终速度 |
| `state_topic` | `motion_manager/state` | `std_msgs/msg/String` | 当前模式、选中源、输出开关和急停状态摘要 |
| `diagnostics_topic` | `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 基础诊断状态 |

## 安全默认值

- `command_output_enabled=false`：默认不透传任何上游速度，只发布零速。
- `command_output_enabled=true` 时也只允许新鲜且有效的上游命令输出；无新鲜命令、急停或输出关闭时仍发布零速。
- 仲裁优先级固定为：emergency stop > manual > recovery > terrain > evasion > navigation。
- 所有输入源都有超时门控，过期命令不会被选中；输出 tick 会按 `output_frequency_hz` 重新仲裁，避免旧命令残留。
- 输出强制差速语义：`linear.y`、`linear.z`、`angular.x`、`angular.y` 置零，仅保留 `linear.x` 和 `angular.z`。
- 输出速度限幅默认 `max_linear_x=2.0 m/s`、`max_angular_z=6.3 rad/s`。
- 输出加速度限幅默认 `max_linear_accel=3.0 m/s^2`、`max_angular_accel=12.0 rad/s^2`，基于上一帧实际发布命令和本帧发布时间差计算。
- 急停优先级高于所有速度源。

## 关键参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `command_output_enabled` | `false` | 是否允许新鲜上游命令进入最终 `/cmd_vel`；关闭时发布零速 |
| `max_linear_x` | `2.0` | `linear.x` 速度绝对值上限，单位 m/s |
| `max_angular_z` | `6.3` | `angular.z` 速度绝对值上限，单位 rad/s |
| `max_linear_accel` | `3.0` | `linear.x` 加速度绝对值上限，单位 m/s^2 |
| `max_angular_accel` | `12.0` | `angular.z` 加速度绝对值上限，单位 rad/s^2 |

## 后续扩展点

1. **恢复状态机**：扩展 `RecoveryPhase`，实现先直线后退，再低曲率/弧线脱困；进展判断使用投影位移，避免累计抖动误判。
2. **Nav2 adapter**：将 Nav2 输出重映射到 `cmd_vel_nav`，由本包统一输出最终 `/cmd_vel`。
3. **安全约束**：接入碰撞预测、底盘反馈和故障降级策略。

## 使用

```bash
ros2 launch sentry_motion_manager motion_manager_launch.py
```

单包构建：

```bash
colcon build --symlink-install --packages-select sentry_motion_manager --cmake-args -DCMAKE_BUILD_TYPE=Release
```

# ROS2 Humble → Jazzy 迁移与深度优化

## TL;DR

> **Quick Summary**: 将哨兵机器人导航系统从 Ubuntu 22.04 + ROS2 Humble 完整迁移到 Ubuntu 24.04 + ROS2 Jazzy，同时进行架构瘦身和性能优化。策略：先减负（删除未用包）→ 再兼容（修复编译/API 变更）→ 最后调优（composition/频率校准）。
> 
> **Deliverables**:
> - 全部包在 Jazzy 下编译通过
> - 仿真环境（Gazebo Harmonic）完整可用
> - 实车功能不变（TF 链、导航参数、串口协议不变）
> - 速度链统一到 TwistStamped
> - 全仓统一 C++17
> - 移除 TEB/costmap_converter 死代码
> - 性能优化：composition + intra-process + 频率校准
> 
> **Estimated Effort**: XL（~100 文件变更，6 个阶段）
> **Parallel Execution**: YES - 6 waves
> **Critical Path**: Phase 0 → Phase 1 → Phase 3 → Phase 4 → Phase 5 → Final Verification

---

## Context

### Original Request
用户希望将项目从 Ubuntu 22.04 + ROS2 Humble 迁移到 Ubuntu 24.04 + ROS2 Jazzy，保证所有功能正常可用且与之前一致（尤其是配置和 TF 变换），同时进行架构优化（功能、代码性能）。

### Interview Summary
**Key Discussions**:
- 仿真环境：必须完整迁移（Gazebo Fortress → Harmonic）
- 双版本支持：不需要，完全切换到 Jazzy
- BehaviorTree.CPP：源码编译安装（4.9.0）
- 优化范围：深度优化（composition、频率校准、代码标准统一）
- TEB 规划器：不确定是否在用（经验证：未使用，可删除）

### Research Findings
- **Gazebo 迁移**（最大工作量）：~55 文件需要 `ignition::` → `gz::` 命名空间迁移。rmoss 上游无 Jazzy 分支，须自行移植。
- **Nav2 TwistStamped**：Jazzy 速度链从 Twist 升级为 TwistStamped。fake_vel_transform 是自然转换边界。
- **Point-LIO CMake**：`find_package(PythonLibs)` 在 CMake 3.28（Ubuntu 24.04 默认）上已被移除，必须第一个修复否则整个工作区无法配置。
- **BehaviorTree.CPP**：11 个插件用了废弃的 `BT_REGISTER_NODES` 宏，但 CMakeLists.txt 已经通过 `BT_PLUGIN_EXPORT` 编译定义处理了注册。
- **TEB 确认未使用**：两个 nav2_params.yaml 均使用 omni_pid_pursuit_controller，TEB 仅存在于注释中。

### Metis Review
**Identified Gaps** (addressed):
- Point-LIO `find_package(PythonLibs)` 是 Jazzy 下的编译阻断点 → 列为最优先修复
- `enable_stamped_cmd_vel: true` 参数必须在两个 yaml 中设置 → 纳入 Phase 3
- BehaviorTree.ROS2 的 `.repos` 文件 pin 到 humble 分支 → 需要更新
- serial_visualizer.py 订阅 cmd_vel_nav2_result 需改为 TwistStamped → 纳入 Phase 3
- Nav2 controller 接口签名在 Jazzy 中未变 → 降低风险

### Oracle Consultation
**核心建议**：先减负、再兼容、最后调优。不要借迁移机会替换核心算法。性能优化聚焦系统工程（composition、频率匹配、少拷贝），而非算法更换。

---

## Work Objectives

### Core Objective
将全部 ROS2 包从 Humble 迁移到 Jazzy，保持所有功能行为一致，同时完成架构瘦身和运行时性能优化。

### Concrete Deliverables
- 全量 `colcon build` 在 Ubuntu 24.04 + ROS2 Jazzy 下零错误
- 仿真可启动：`ros2 launch rmu_gazebo_simulator bringup_sim.launch.py`
- 导航可运行：`ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py`
- TF 链完整：`map → odom → chassis → gimbal_yaw → gimbal_pitch → front_mid360`
- 实车启动可用：`ros2 launch sentry_nav_bringup rm_sentry_launch.py`
- 速度链路正确：TwistStamped 从 controller 到 fake_vel_transform，Twist 从 fake_vel_transform 到底盘/串口
- 所有文档更新完毕（AGENTS.md, README.md, CLAUDE.md, docs/*）

### Definition of Done
- [ ] `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` → 0 failures
- [ ] `colcon test && colcon test-result --verbose` → 无新增失败
- [ ] 仿真导航端到端可用（headless 模式下发目标点，机器人导航到达）
- [ ] `ros2 run tf2_ros tf2_echo map chassis` → 有效变换输出
- [ ] `grep -r "ignition::" src/simulator/ --include="*.cpp" --include="*.hpp"` → 0 结果
- [ ] `grep -r "CMAKE_CXX_STANDARD 14" src/*/CMakeLists.txt` → 0 结果（活跃包）

### Must Have
- 所有 TF 变换链保持不变
- 所有 nav2_params.yaml 参数值不变（仅添加 `enable_stamped_cmd_vel: true`）
- 串口协议不变（protocol.yaml 不改）
- 导航行为一致（同样的地图、同样的目标点、机器人走类似路径）

### Must NOT Have (Guardrails)
- ❌ 替换 SmacPlanner2D 或 omni_pid_pursuit_controller
- ❌ 重构 Point-LIO 超出 CMake 修复范围
- ❌ 改变导航参数（costmap 尺寸、inflation、PID 增益等）
- ❌ 改变 TF 树结构或坐标系名称
- ❌ 改变串口协议或 protocol.yaml
- ❌ 添加新 ROS 包或新功能
- ❌ 添加 Docker Compose 或 devcontainer
- ❌ 重构 sentry_tools Python 代码（除 TwistStamped 订阅修改外）
- ❌ 升级 SDF 版本号（1.6/1.7 在 Harmonic 上兼容）

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** - ALL verification is agent-executed. No exceptions.

### Test Decision
- **Infrastructure exists**: YES (colcon test + ament_lint)
- **Automated tests**: Tests-after（迁移后验证编译和基本功能）
- **Framework**: colcon test (ament_lint_auto)
- **Primary verification**: Agent-executed QA via simulation launch + topic inspection

### QA Policy
Every task MUST include agent-executed QA scenarios.
Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

- **Build verification**: Bash — `colcon build --packages-select <pkg>`
- **TF verification**: Bash — `ros2 run tf2_ros tf2_echo`
- **Topic verification**: Bash — `ros2 topic info <topic> --verbose`
- **Simulation**: interactive_bash (tmux) — launch sim, send goal, check output

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 0 (减负 — 可在 Humble 上完成):
├── Task 1: 删除 TEB/costmap_converter + 清理文档 [quick]

Wave 1 (CMake 阻断修复 — 可在 Humble 上验证):
├── Task 2: Point-LIO CMake 现代化 [quick]
├── Task 3: 自研包 C++17 统一 [quick]
├── Task 4: 第三方 fork 包 C++17 统一 [quick]

Wave 2 (BehaviorTree + Nav2 API — 需要 Jazzy 环境):
├── Task 5: BT.CPP 宏迁移 + BT::NodeConfig [quick]
├── Task 6: BehaviorTree.ROS2 更新到 Jazzy 分支 [quick]
├── Task 7: fake_vel_transform TwistStamped 迁移 [deep]
├── Task 8: Nav2 参数启用 TwistStamped + teleop 配置 [quick]
├── Task 9: back_up_free_space Nav2 Jazzy 适配 [unspecified-high]
├── Task 10: serial_visualizer.py TwistStamped 适配 [quick]

Wave 3 (Gazebo Fortress→Harmonic — 最大工作量):
├── Task 11: rmoss_gz_plugins 迁移 (ignition→gz) [deep]
├── Task 12: rmoss_gz_base 迁移 (ignition→gz) [deep]
├── Task 13: rmoss_gz_bridge + rmoss_gz_cam 迁移 [deep]
├── Task 14: SDF 世界文件 + 模型文件迁移 [unspecified-high]
├── Task 15: env-hooks + 脚本 + launch 文件迁移 [unspecified-high]
├── Task 16: ign_sim_pointcloud_tool 适配 [quick]
├── Task 17: rmoss_core C++17 升级 [quick]

Wave 4 (基础设施 — Wave 3 之后):
├── Task 18: setup_env.sh 全面更新 [quick]
├── Task 19: Dockerfiles 更新 [quick]
├── Task 20: .repos 文件 + CI 工作流更新 [quick]
├── Task 21: 全部文档更新 (AGENTS.md, README.md, CLAUDE.md, docs/*) [writing]

Wave 5 (性能优化 — Wave 4 之后):
├── Task 22: terrain_analysis/ext 组合进 nav2_container [unspecified-high]
├── Task 23: 评估并启用 intra-process 通信 [deep]
├── Task 24: 频率校准验证与文档记录（仅验证+记录，不改 YAML 参数）[unspecified-high]

Wave FINAL (4 parallel reviews → user okay):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Code quality review (unspecified-high)
├── Task F3: Real manual QA — simulation E2E (unspecified-high + playwright for doc review)
├── Task F4: Scope fidelity check (deep)
-> Present results -> Get explicit user okay
```

### Dependency Matrix

| Task | Depends On | Blocks |
|------|-----------|--------|
| T1 | - | T2-T4 (unblocks cleaner build) |
| T2 | T1 | T5-T10 (Point-LIO must configure) |
| T3 | T1 | T7, T9 (fake_vel_transform, nav2_plugins need C++17) |
| T4 | T1 | T11-T17 (third-party forks need C++17 before Gz migration) |
| T5 | T2 | - |
| T6 | T2 | T5 (BT.ROS2 must be compatible) |
| T7 | T3, T8 | - |
| T8 | T2 | T7, T9 |
| T9 | T3, T8 | - |
| T10 | T8 | - |
| T11 | T4 | T14, T15 |
| T12 | T4 | T14, T15 |
| T13 | T4 | T14, T15 |
| T14 | T11, T12, T13 | T15 |
| T15 | T14 | T18 |
| T16 | T3 | - |
| T17 | T4 | T11, T12, T13 |
| T18 | T15 | - |
| T19 | T15 | - |
| T20 | T6, T15 | - |
| T21 | T1-T20 (all) | F1-F4 |
| T22 | T18 | - |
| T23 | T22 | T24 |
| T24 | T23 | F1-F4 |
| F1-F4 | T21, T24 | user okay |

### Agent Dispatch Summary

- **Wave 0**: 1 task → `quick`
- **Wave 1**: 3 tasks → `quick` × 3
- **Wave 2**: 6 tasks → `deep` × 1, `quick` × 4, `unspecified-high` × 1
- **Wave 3**: 7 tasks → `deep` × 3, `unspecified-high` × 2, `quick` × 2
- **Wave 4**: 4 tasks → `quick` × 3, `writing` × 1
- **Wave 5**: 3 tasks → `deep` × 1, `unspecified-high` × 2
- **FINAL**: 4 review agents

---

## TODOs

- [ ] 1. 删除 TEB/costmap_converter + 清理文档

  **What to do**:
  - 删除 `src/teb_local_planner/` 和 `src/costmap_converter/` 整个目录
  - 删除 `config/reality/nav2_params.yaml` 中被注释的 TEB 配置块（约 lines 405-540）
  - 更新 AGENTS.md §2 移除 TEB/costmap_converter 条目，§8 移除 TEB 依赖描述
  - 更新 README.md 目录树移除 teb_local_planner 和 costmap_converter

  **Must NOT do**: 不要修改任何活跃代码或未注释的配置

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 0, Blocks T2-T4, Blocked By: None

  **References**:
  - `src/sentry_nav_bringup/config/reality/nav2_params.yaml` lines 405-540 — 注释的 TEB 配置块
  - `src/sentry_nav_bringup/config/simulation/nav2_params.yaml` — 检查是否也有 TEB 注释
  - `AGENTS.md` §2 和 §8 — TEB/costmap_converter 描述
  - `README.md` — 目录树

  **QA Scenarios**:
  ```
  Scenario: TEB 目录已删除
    Tool: Bash
    Steps:
      1. ls src/teb_local_planner src/costmap_converter 2>&1
    Expected Result: "No such file or directory" for both
    Evidence: .sisyphus/evidence/task-1-teb-removed.txt

  Scenario: 无活跃 TEB 引用
    Tool: Bash
    Steps:
      1. grep -r "teb\|TEB\|costmap_converter" src/sentry_nav_bringup/config/ --include="*.yaml" | grep -v "^#" | grep -v "^--"
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-1-no-teb-refs.txt
  ```

  **Commit**: YES — `refactor: 移除未使用的 TEB/costmap_converter 包并清理文档`

- [ ] 2. Point-LIO CMake 现代化

  **What to do**:
  - 替换 `find_package(PythonLibs REQUIRED)` → `find_package(Python3 COMPONENTS Development REQUIRED)`
  - 替换 `${PYTHON_LIBRARIES}` → `Python3::Python`，`${PYTHON_INCLUDE_DIRS}` → `Python3::Python`（target 自动带 include）
  - 删除 line 8 `ADD_COMPILE_OPTIONS(-std=c++17)` 和 line 9 `set(CMAKE_CXX_FLAGS "-std=c++17 -O3")`（保留 line 14 `set(CMAKE_CXX_STANDARD 17)`）
  - 删除 line 17 中 `-std=c++17` 重复
  - 删除 `find_package(rclpy REQUIRED)` 和 `ament_target_dependencies(... rclpy ...)`
  - 删除 `ament_export_dependencies(... rclpy ...)`

  **Must NOT do**: 不要修改 Point-LIO 任何 C++ 源代码

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 1 (with T3, T4), Blocked By: T1

  **References**:
  - `src/sentry_nav/point_lio/CMakeLists.txt` — 目标文件
  - `src/sentry_nav/odom_bridge/CMakeLists.txt` — C++17 设置的正确模式参考

  **QA Scenarios**:
  ```
  Scenario: Point-LIO 编译成功
    Tool: Bash
    Steps:
      1. source /opt/ros/jazzy/setup.bash（或当前 ROS 环境）
      2. colcon build --packages-select point_lio --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    Expected Result: "1 package finished" with 0 failures
    Evidence: .sisyphus/evidence/task-2-pointlio-build.txt

  Scenario: 无 PythonLibs 引用
    Tool: Bash
    Steps:
      1. grep -i "PythonLibs\|PYTHON_LIBRARIES\|PYTHON_INCLUDE_DIRS" src/sentry_nav/point_lio/CMakeLists.txt
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-2-no-pythonlibs.txt
  ```

  **Commit**: YES — `fix(point_lio): 替换废弃 PythonLibs，清理冗余 CMake 标志`

- [ ] 3. 自研包 C++17 统一

  **What to do**:
  - `fake_vel_transform/CMakeLists.txt`: `CMAKE_CXX_STANDARD 14` → `17`
  - `ign_sim_pointcloud_tool/CMakeLists.txt`: 添加 `set(CMAKE_CXX_STANDARD 17)`（如无）
  - `serial/serial_driver/CMakeLists.txt`: `CMAKE_CXX_STANDARD 14` → `17`
  - `omni_pid_pursuit_controller/CMakeLists.txt`: 添加 `set(CMAKE_CXX_STANDARD 17)` + `set(CMAKE_CXX_STANDARD_REQUIRED ON)`
  - `pointcloud_to_laserscan/CMakeLists.txt`: 添加 `set(CMAKE_CXX_STANDARD 17)` + `set(CMAKE_CXX_STANDARD_REQUIRED ON)`

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 1 (with T2, T4), Blocked By: T1

  **References**:
  - 各包的 `CMakeLists.txt`
  - `src/sentry_nav/odom_bridge/CMakeLists.txt` lines 4-5 — 正确的 C++17 设置模式

  **QA Scenarios**:
  ```
  Scenario: 无 C++14 残留（自研包）
    Tool: Bash
    Steps:
      1. grep "CMAKE_CXX_STANDARD 14" src/sentry_nav/fake_vel_transform/CMakeLists.txt src/sentry_nav/ign_sim_pointcloud_tool/CMakeLists.txt src/serial/serial_driver/CMakeLists.txt
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-3-no-cpp14.txt
  ```

  **Commit**: YES — `build: 统一自研包 C++ 标准为 C++17`

- [ ] 4. 第三方 fork 包 C++17 统一

  **What to do**:
  - `livox_ros_driver2/CMakeLists.txt`: `CMAKE_CXX_STANDARD 14` → `17`
  - `teleop_twist_joy/CMakeLists.txt`: 确保有 `set(CMAKE_CXX_STANDARD 17)`
  - `terrain_analysis/CMakeLists.txt`: 添加或更新为 C++17
  - `terrain_analysis_ext/CMakeLists.txt`: 添加或更新为 C++17

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 1 (with T2, T3), Blocked By: T1

  **QA Scenarios**:
  ```
  Scenario: 第三方 fork 包编译通过
    Tool: Bash
    Steps:
      1. colcon build --packages-select livox_ros_driver2 teleop_twist_joy terrain_analysis terrain_analysis_ext --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    Expected Result: "4 packages finished" with 0 failures
    Evidence: .sisyphus/evidence/task-4-forks-build.txt
  ```

  **Commit**: YES — `build: 统一第三方 fork 包 C++ 标准为 C++17`

- [ ] 5. BT.CPP 类型别名迁移

  **What to do**:
  - 将所有 `BT::NodeConfiguration` → `BT::NodeConfig`（约 5 个文件：2 个头文件 + 3 个源文件）
  - **保留** 所有 `BT_REGISTER_NODES(factory) { ... }` 块不动 — 这是当前插件的运行时注册入口，`CMakeLists.txt` 的 `BT_PLUGIN_EXPORT` 编译定义只控制符号导出，不替代 `BT_REGISTER_NODES` 的注册逻辑
  - 检查 BT.CPP 4.9.0 是否弃用了 `BT_REGISTER_NODES`，如果 Jazzy 版本仍支持则保持不变

  **Must NOT do**: 不要删除 `BT_REGISTER_NODES` 宏块 — 删除会导致插件运行时无法注册

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 2 (with T6-T10), Blocked By: T2

  **References**:
  - `src/sentry_behavior/CMakeLists.txt` lines 82-84 — `BT_PLUGIN_EXPORT` 编译定义（仅控制符号导出）
  - `src/sentry_behavior/plugins/action/battlefield_information.cpp` lines 45-49 — `BT_REGISTER_NODES` 注册入口示例
  - `src/sentry_behavior/include/sentry_behavior/plugins/control/recovery_node.hpp` — 含 `BT::NodeConfiguration`
  - `src/sentry_behavior/include/sentry_behavior/plugins/decorator/rate_controller.hpp` — 含 `BT::NodeConfiguration`

  **QA Scenarios**:
  ```
  Scenario: 无 NodeConfiguration 残留
    Tool: Bash
    Steps:
      1. grep -r "NodeConfiguration" src/sentry_behavior/ --include="*.hpp" --include="*.cpp"
    Expected Result: 无输出（全部替换为 NodeConfig）
    Evidence: .sisyphus/evidence/task-5-no-nodeconfig.txt

  Scenario: BT_REGISTER_NODES 仍然存在
    Tool: Bash
    Steps:
      1. grep -c "BT_REGISTER_NODES" src/sentry_behavior/plugins/action/battlefield_information.cpp
    Expected Result: 输出 "1"（注册宏保留）
    Evidence: .sisyphus/evidence/task-5-bt-register-kept.txt

  Scenario: sentry_behavior 编译通过
    Tool: Bash
    Steps:
      1. colcon build --packages-select sentry_behavior --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    Expected Result: "1 package finished" with 0 failures
    Evidence: .sisyphus/evidence/task-5-bt-build.txt
  ```

  **Commit**: YES — `refactor(sentry_behavior): BT::NodeConfiguration → BT::NodeConfig`

- [ ] 6. BehaviorTree.ROS2 更新到 Jazzy 兼容分支

  **What to do**:
  - 更新 `src/sentry_behavior/dependencies.repos` 中 BehaviorTree.ROS2 的 `version:` 从 `humble` 到 Jazzy 兼容分支（main 或 jazzy）
  - 更新 `src/sentry_robot_description/dependencies.repos` 中相关依赖分支

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 2, Blocked By: T2

  **QA Scenarios**:
  ```
  Scenario: .repos 无 humble 分支引用
    Tool: Bash
    Steps:
      1. grep "humble" src/sentry_behavior/dependencies.repos src/sentry_robot_description/dependencies.repos
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-6-no-humble-repos.txt
  ```

  **Commit**: YES — `build: 更新 BehaviorTree.ROS2 到 Jazzy 兼容分支`

- [ ] 7. fake_vel_transform TwistStamped 迁移

  **What to do**:
  - 修改 `fake_vel_transform.hpp/cpp` 中 `cmd_vel_nav2_result` 订阅类型从 `geometry_msgs::msg::Twist` → `geometry_msgs::msg::TwistStamped`
  - 回调函数提取 `msg->twist` 获取速度分量
  - 保持输出 `/cmd_vel` 仍为 `geometry_msgs::msg::Twist`（串口驱动需要）
  - 更新 fake_vel_transform README.md 移除过时的 local_plan 时间戳 hack 描述

  **Must NOT do**: 不要改变输出 `/cmd_vel` 的消息类型

  **Recommended Agent Profile**: `deep`
  **Parallelization**: Wave 2, Blocked By: T3, T8

  **References**:
  - `src/sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp` — 主要修改目标
  - `src/sentry_nav/fake_vel_transform/include/` — 头文件
  - `src/sentry_nav/omni_pid_pursuit_controller/include/omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp` line 89 — TwistStamped 使用模式参考

  **QA Scenarios**:
  ```
  Scenario: fake_vel_transform 编译通过
    Tool: Bash
    Steps:
      1. colcon build --packages-select fake_vel_transform --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    Expected Result: "1 package finished" with 0 failures
    Evidence: .sisyphus/evidence/task-7-fvt-build.txt

  Scenario: 输入类型为 TwistStamped（代码验证）
    Tool: Bash
    Steps:
      1. grep "TwistStamped" src/sentry_nav/fake_vel_transform/src/fake_vel_transform.cpp
    Expected Result: 至少 1 行包含 TwistStamped（订阅类型）
    Evidence: .sisyphus/evidence/task-7-twiststamped.txt
  ```

  **Commit**: YES — `feat(fake_vel_transform): 升级输入为 TwistStamped 适配 Jazzy Nav2`

- [ ] 8. Nav2 参数启用 TwistStamped

  **What to do**:
  - 在 `config/simulation/nav2_params.yaml` 的 `controller_server.ros__parameters` 下添加 `enable_stamped_cmd_vel: true`
  - 在 `config/reality/nav2_params.yaml` 的 `controller_server.ros__parameters` 下添加 `enable_stamped_cmd_vel: true`
  - 在两个 yaml 的 teleop_twist_joy 配置中设置 `publish_stamped_twist: true`

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 2, Blocked By: T2

  **QA Scenarios**:
  ```
  Scenario: 两个 yaml 均含 enable_stamped_cmd_vel
    Tool: Bash
    Steps:
      1. grep "enable_stamped_cmd_vel" src/sentry_nav_bringup/config/simulation/nav2_params.yaml src/sentry_nav_bringup/config/reality/nav2_params.yaml
    Expected Result: 两个文件各输出一行含 "true"
    Evidence: .sisyphus/evidence/task-8-stamped-enabled.txt
  ```

  **Commit**: YES — `feat: 启用 Nav2 TwistStamped 参数`

- [ ] 9. back_up_free_space Nav2 Jazzy 适配

  **What to do**:
  - 检查 Jazzy `nav2_behaviors` 头文件中 `vel_pub_` 类型是否变为 TwistStamped
  - 如果变了，更新 `back_up_free_space.cpp` 中构造 velocity 消息为 TwistStamped
  - 确保 lifecycle publisher 的 `on_activate()/on_deactivate()` 调用与 Jazzy API 兼容

  **Recommended Agent Profile**: `unspecified-high`
  **Parallelization**: Wave 2, Blocked By: T3, T8

  **References**:
  - `src/sentry_nav/nav2_plugins/src/behaviors/back_up_free_space.cpp` — 修改目标
  - `src/sentry_nav/nav2_plugins/include/nav2_plugins/behaviors/back_up_free_space.hpp` — 头文件
  - Jazzy `nav2_behaviors` installed headers — 运行时参考

  **QA Scenarios**:
  ```
  Scenario: nav2_plugins 编译通过
    Tool: Bash
    Steps:
      1. colcon build --packages-select nav2_plugins --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    Expected Result: "1 package finished" with 0 failures
    Evidence: .sisyphus/evidence/task-9-nav2plugins-build.txt
  ```

  **Commit**: YES — `fix(nav2_plugins): 适配 Jazzy Nav2 TwistStamped API`

- [ ] 10. serial_visualizer.py TwistStamped 适配

  **What to do**:
  - 更新 `serial_visualizer.py` 中 `cmd_vel_nav2_result` 的订阅消息类型为 `TwistStamped`
  - 回调中改为 `msg.twist.linear.x` 等（而非 `msg.linear.x`）

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 2, Blocked By: T8

  **References**:
  - `src/sentry_tools/serial_visualizer.py` lines 123, 126 — 订阅和回调

  **QA Scenarios**:
  ```
  Scenario: 订阅类型已更新
    Tool: Bash
    Steps:
      1. grep "TwistStamped" src/sentry_tools/serial_visualizer.py
    Expected Result: 至少 1 行
    Evidence: .sisyphus/evidence/task-10-viz-stamped.txt
  ```

  **Commit**: YES — `fix(sentry_tools): 适配 TwistStamped 话题订阅`

- [ ] 11. rmoss_gz_plugins 迁移 (ignition→gz)

  **What to do**:
  - `CMakeLists.txt`: `find_package(ignition-gazebo6)` → `find_package(gz-sim8)`，对应 target 链接也更新
  - 3 个插件 .cc/.hh 文件：`ignition::gazebo` → `gz::sim`，`ignition::msgs` → `gz::msgs`
  - `IGNITION_ADD_PLUGIN` → `GZ_ADD_PLUGIN`，`IGNITION_ADD_PLUGIN_ALIAS` → `GZ_ADD_PLUGIN_ALIAS`
  - `#include <ignition/gazebo/` → `#include <gz/sim/`
  - `IGNITION_GAZEBO_VISIBLE` → `GZ_SIM_VISIBLE`

  **Recommended Agent Profile**: `deep`
  **Parallelization**: Wave 3 (with T12-T17), Blocked By: T4, T17

  **QA Scenarios**:
  ```
  Scenario: 无 ignition 引用
    Tool: Bash
    Steps:
      1. grep -r "ignition" src/simulator/rmoss_gazebo/rmoss_gz_plugins/ --include="*.cc" --include="*.hh" --include="*.cmake" -l
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-11-gz-plugins-clean.txt
  ```

  **Commit**: YES — `feat(simulator): 迁移 rmoss_gz_plugins 到 Gazebo Harmonic`

- [ ] 12. rmoss_gz_base 迁移 (ignition→gz)

  **What to do**:
  - `CMakeLists.txt`: `ignition-transport11` → `gz-transport13`，`ignition-msgs8` → `gz-msgs10`
  - `package.xml`: 同步更新依赖名
  - 所有 .hpp/.cpp：`ignition::transport` → `gz::transport`，`ignition::msgs` → `gz::msgs`，`ignition::math` → `gz::math`
  - `#include <ignition/` → `#include <gz/`

  **Recommended Agent Profile**: `deep`
  **Parallelization**: Wave 3, Blocked By: T4, T17

  **QA Scenarios**:
  ```
  Scenario: 无 ignition 引用
    Tool: Bash
    Steps:
      1. grep -r "ignition" src/simulator/rmoss_gazebo/rmoss_gz_base/ --include="*.hpp" --include="*.cpp" -l
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-12-gz-base-clean.txt
  ```

  **Commit**: YES — `feat(simulator): 迁移 rmoss_gz_base 到 Gazebo Harmonic`

- [ ] 13. rmoss_gz_bridge + rmoss_gz_cam 迁移

  **What to do**:
  - 两个包的 CMakeLists.txt: `ignition-transport11` → `gz-transport13`，`ignition-msgs8` → `gz-msgs10`
  - 所有 .hpp/.cpp: `ignition::` → `gz::` 命名空间替换
  - `#include <ignition/` → `#include <gz/`

  **Recommended Agent Profile**: `deep`
  **Parallelization**: Wave 3, Blocked By: T4, T17

  **QA Scenarios**:
  ```
  Scenario: 两个包无 ignition 引用
    Tool: Bash
    Steps:
      1. grep -r "ignition" src/simulator/rmoss_gazebo/rmoss_gz_bridge/ src/simulator/rmoss_gazebo/rmoss_gz_cam/ --include="*.hpp" --include="*.cpp" --include="*.hh" --include="*.cc" -l
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-13-gz-bridge-cam-clean.txt
  ```

  **Commit**: YES — `feat(simulator): 迁移 rmoss_gz_bridge/cam 到 Gazebo Harmonic`

- [ ] 14. SDF 世界文件 + 模型文件迁移

  **What to do**:
  - 所有 `*_world.sdf` 文件：`libignition-gazebo-*-system.so` → `gz-sim-*-system`
  - 所有 `*_world.sdf` 和 `model.sdf` 文件：`ignition::gazebo::systems::*` → `gz::sim::systems::*`
  - 检查并替换所有 `<ignition>` XML 标签 → 对应 Harmonic 格式

  **Recommended Agent Profile**: `unspecified-high`
  **Parallelization**: Wave 3, Blocked By: T11, T12, T13

  **References**:
  - `src/simulator/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/` — 6+ world SDF 文件
  - `src/simulator/rmoss_gz_resources/resource/models/` — 模型 SDF 文件

  **QA Scenarios**:
  ```
  Scenario: 无 libignition 插件引用
    Tool: Bash
    Steps:
      1. grep -r "libignition\|ignition::gazebo" src/simulator/ --include="*.sdf" -l
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-14-sdf-clean.txt
  ```

  **Commit**: YES — `feat(simulator): 更新 SDF 世界/模型文件为 Gz Harmonic 格式`

- [ ] 15. env-hooks + 脚本 + launch 文件迁移

  **What to do**:
  - `*.dsv.in` env-hooks: `IGN_GAZEBO_RESOURCE_PATH` → `GZ_SIM_RESOURCE_PATH`，`IGN_FILE_PATH` → `GZ_FILE_PATH`
  - launch 文件: `ign_config_path` → `gz_config_path`，`gz_version: "6"` → `gz_version: "8"`
  - shell 脚本: `ign service` → `gz service`，`ign topic` → `gz topic`，`ign gazebo` → `gz sim`
  - `sdformat_tools/sdf_util.py`: `IGN_GAZEBO_RESOURCE_PATH` → `GZ_SIM_RESOURCE_PATH`

  **Recommended Agent Profile**: `unspecified-high`
  **Parallelization**: Wave 3, Blocked By: T14

  **QA Scenarios**:
  ```
  Scenario: 无 IGN_ 环境变量引用
    Tool: Bash
    Steps:
      1. grep -r "IGN_GAZEBO\|IGN_FILE" src/simulator/ --include="*.dsv.in" --include="*.py" --include="*.sh" -l
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-15-no-ign-env.txt
  ```

  **Commit**: YES — `feat(simulator): 更新 env-hooks/scripts/launch 为 gz CLI`

- [ ] 16. ign_sim_pointcloud_tool 适配

  **What to do**:
  - 确保 C++17 标准已设置（T3 已完成）
  - 检查是否有 Ignition-specific API 调用需要迁移（该包主要是 PCL 处理，预计无 Gazebo API 依赖）
  - 如果用户决定重命名，执行包名、命名空间、include path 全面重命名

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 3, Blocked By: T3

  **QA Scenarios**:
  ```
  Scenario: 包编译通过
    Tool: Bash
    Steps:
      1. colcon build --packages-select ign_sim_pointcloud_tool --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    Expected Result: "1 package finished" with 0 failures
    Evidence: .sisyphus/evidence/task-16-ign-tool-build.txt
  ```

  **Commit**: YES — `feat: 适配 ign_sim_pointcloud_tool 到 Gz Harmonic`

- [ ] 17. rmoss_core C++17 升级

  **What to do**:
  - 更新 `rmoss_core` 下所有子包的 CMakeLists.txt 为 C++17
  - 包括：rmoss_util, rmoss_cam, rmoss_base, rmoss_projectile_motion

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 3, Blocked By: T4

  **QA Scenarios**:
  ```
  Scenario: rmoss_core 编译通过
    Tool: Bash
    Steps:
      1. colcon build --packages-select rmoss_util rmoss_cam rmoss_base rmoss_projectile_motion --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
    Expected Result: "4 packages finished" with 0 failures
    Evidence: .sisyphus/evidence/task-17-rmoss-core-build.txt
  ```

  **Commit**: YES — `build(simulator): 升级 rmoss_core 到 C++17`

- [ ] 18. setup_env.sh 全面更新

  **What to do**:
  - `jammy` → `noble`
  - `ros-humble-*` → `ros-jazzy-*`
  - `/opt/ros/humble/` → `/opt/ros/jazzy/`
  - `--rosdistro=humble` → `--rosdistro=jazzy`
  - `ign` → `gz` 相关命令
  - `ignition-fortress` → Gazebo Harmonic via `ros-jazzy-ros-gz`

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 4 (with T19-T21), Blocked By: T15

  **QA Scenarios**:
  ```
  Scenario: 无 humble/jammy 引用
    Tool: Bash
    Steps:
      1. grep -i "humble\|jammy" src/scripts/setup_env.sh
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-18-setup-env-clean.txt
  ```

  **Commit**: YES — `build: 更新 setup_env.sh 支持 Ubuntu 24.04 + Jazzy`

- [ ] 19. Dockerfiles 更新

  **What to do**:
  - 两个 Dockerfile: `FROM ros:humble-ros-base` → `FROM ros:jazzy-ros-base`
  - `ignition-fortress` → Gazebo Harmonic 安装
  - `--rosdistro humble` → `--rosdistro jazzy`

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 4, Blocked By: T15

  **QA Scenarios**:
  ```
  Scenario: 无 humble 引用
    Tool: Bash
    Steps:
      1. grep -i "humble\|ignition-fortress" src/sentry_nav/Dockerfile src/simulator/rmu_gazebo_simulator/Dockerfile
    Expected Result: 无输出
    Evidence: .sisyphus/evidence/task-19-docker-clean.txt
  ```

  **Commit**: YES — `build: 更新 Dockerfiles 为 Jazzy 基础镜像`

- [ ] 20. .repos 文件 + CI 工作流更新

  **What to do**:
  - 更新所有 `.repos` 文件中的 `version: humble` → Jazzy 兼容分支
  - 如有 `.github/workflows/` CI 文件，更新容器镜像和 distro 引用
  - 更新 `fix_*.sh` 脚本中的 humble/jammy 引用

  **Recommended Agent Profile**: `quick`
  **Parallelization**: Wave 4, Blocked By: T6, T15

  **QA Scenarios**:
  ```
  Scenario: 无 humble 分支引用
    Tool: Bash
    Steps:
      1. grep -r "humble" src/*/dependencies.repos src/**/*.repos src/scripts/fix_*.sh 2>/dev/null | grep -v "^Binary"
    Expected Result: 无输出（或仅有注释中的历史引用）
    Evidence: .sisyphus/evidence/task-20-repos-clean.txt
  ```

  **Commit**: YES — `build: 更新 .repos 和 CI 到 Jazzy`

- [ ] 21. 全部文档更新

  **What to do**:
  - AGENTS.md: 更新 §1（Ubuntu 24.04 + Jazzy）, §4（Gazebo Harmonic CLI）, §5（Jazzy 编译命令）, §7（如有新文件）, §8（依赖版本）, §9（Jazzy 特有注意事项）
  - README.md: 更新系统要求、编译命令、快速开始命令中的 Gazebo 引用
  - CLAUDE.md: 更新编译命令和 Gazebo 命令
  - `src/docs/QUICKSTART.md`, `RUNNING_MODES.md`, `ARCHITECTURE.md`, `REMOTE_DEBUG.md`: 更新 Jazzy/Harmonic 引用
  - `src/sentry_tools/README.md`: 更新 ROS 环境引用

  **Recommended Agent Profile**: `writing`
  **Parallelization**: Wave 4, Blocked By: T1-T20 (all)

  **QA Scenarios**:
  ```
  Scenario: AGENTS.md 不含过时 Humble 引用
    Tool: Bash
    Steps:
      1. grep -c "Humble\|humble" AGENTS.md
    Expected Result: 0 或仅在历史说明中
    Evidence: .sisyphus/evidence/task-21-docs-clean.txt
  ```

  **Commit**: YES — `docs: 全面更新文档至 Ubuntu 24.04 + Jazzy + Gz Harmonic`

- [ ] 22. terrain_analysis/ext 组合进 nav2_container

  **What to do**:
  - 首先检查 `terrain_analysis` 和 `terrain_analysis_ext` 是否支持 composable node（查 CMakeLists.txt 有无 `rclcpp_components_register_node`）
  - 如果支持：在 `src/sentry_nav_bringup/launch/navigation_launch.py`（实际启动 terrain 节点的文件，约 lines 117-137, 356-357）中将独立节点改为 composition 加载到 `nav2_container`
  - 如果不支持：需要先在 CMakeLists.txt 中注册为 composable node，再修改 launch 文件
  - 同步修改实车 launch 如果 terrain 也在其中以独立方式启动

  **Recommended Agent Profile**: `unspecified-high`
  **Parallelization**: Wave 5 (with T23, T24), Blocked By: T18

  **References**:
  - `src/sentry_nav_bringup/launch/navigation_launch.py` lines 117-137, 356-357 — **实际承载 terrain 节点启动的 launch 文件**
  - `src/sentry_nav/terrain_analysis/CMakeLists.txt` — 检查是否已有 `rclcpp_components_register_node`
  - `src/sentry_nav/terrain_analysis_ext/CMakeLists.txt` — 检查是否已有 `rclcpp_components_register_node`
  - `src/sentry_nav_bringup/launch/rm_navigation_simulation_launch.py` — 顶层 launch（include navigation_launch.py）

  **QA Scenarios**:
  ```
  Scenario: terrain 节点在 nav2_container 中加载
    Tool: Bash
    Steps:
      1. grep -A2 "terrain_analysis" src/sentry_nav_bringup/launch/navigation_launch.py | grep -i "container\|composable\|load_node\|ComposableNode"
    Expected Result: 至少 1 行显示 composition 加载模式
    Evidence: .sisyphus/evidence/task-22-terrain-composed.txt

  Scenario: terrain 包的 CMakeLists 含 composable 注册
    Tool: Bash
    Steps:
      1. grep "rclcpp_components_register_node" src/sentry_nav/terrain_analysis/CMakeLists.txt src/sentry_nav/terrain_analysis_ext/CMakeLists.txt
    Expected Result: 两个文件各至少 1 行（如果原本没有则本 task 会先添加）
    Evidence: .sisyphus/evidence/task-22-composable-register.txt
  ```

  **Commit**: YES — `perf: 将 terrain_analysis/ext 组合进 nav2_container`

- [ ] 23. 评估并启用 intra-process 通信

  **What to do**:
  - 评估将 `component_container_isolated` → `component_container` 对速度链路节点的影响
  - 如果评估安全，启用 `use_intra_process_comms=True`
  - 如果评估有风险（线程安全问题），记录原因到 `.sisyphus/evidence/intra-process-evaluation.md` 并跳过

  **Recommended Agent Profile**: `deep`
  **Parallelization**: Wave 5, Blocked By: T22

  **QA Scenarios**:
  ```
  Scenario: 发送导航目标后速度链路正常
    Tool: interactive_bash (tmux)
    Preconditions: Jazzy 环境已 source
    Steps:
      1. 启动仿真 headless: ros2 launch rmu_gazebo_simulator bringup_sim.launch.py headless:=true
      2. 等待 10s 后 unpause: gz service -s /world/default/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false'
      3. 启动导航: ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmul_2026 slam:=False
      4. 等待 15s 让 Nav2 完全启动
      5. 发送导航目标触发速度输出: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
      6. 在另一个 tmux pane 测量: ros2 topic hz /cmd_vel --window 10
      7. 同时测量: ros2 topic hz /cmd_vel_nav2_result --window 10
    Expected Result: 两个话题在机器人运动期间均有稳定频率（> 10 Hz）
    Failure Indicators: /cmd_vel 频率为 0 或极不稳定
    Evidence: .sisyphus/evidence/task-23-intra-process-hz.txt

  Scenario: 评估报告已生成（无论是否启用）
    Tool: Bash
    Steps:
      1. ls .sisyphus/evidence/intra-process-evaluation.md 2>/dev/null || ls .sisyphus/evidence/task-23-intra-process-hz.txt
    Expected Result: 至少一个文件存在
    Evidence: .sisyphus/evidence/task-23-evaluation-exists.txt
  ```

  **Commit**: YES (if enabled) — `perf: 启用 intra-process 通信优化速度链路`

- [ ] 24. 频率校准验证与文档记录

  **What to do**:
  - 在 Jazzy 环境下运行仿真和导航，测量 controller_server 实际频率
  - 记录 `controller_frequency` 和 `smoothing_frequency` 在 Jazzy 下的实际达成率
  - 将结果写入 `.sisyphus/evidence/frequency-calibration.md`
  - **不修改 nav2_params.yaml 中的频率参数**（仅记录和验证）

  **Must NOT do**: 不修改 nav2_params.yaml 中的任何频率参数值

  **Recommended Agent Profile**: `unspecified-high`
  **Parallelization**: Wave 5, Blocked By: T23

  **QA Scenarios**:
  ```
  Scenario: 频率校准文档已生成
    Tool: Bash
    Steps:
      1. cat .sisyphus/evidence/frequency-calibration.md
    Expected Result: 文件存在且包含实测频率数据
    Evidence: .sisyphus/evidence/task-24-freq-calibration.txt
  ```

  **Commit**: YES — `docs: 验证并记录 Jazzy 下频率校准结果`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE. Present consolidated results to user and get explicit "okay" before completing.

- [ ] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. For each "Must Have": verify implementation exists (read file, run command). For each "Must NOT Have": search codebase for forbidden patterns. Check evidence files exist in `.sisyphus/evidence/`. Compare deliverables against plan.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [ ] F2. **Code Quality Review** — `unspecified-high`
  Run `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`. Check all changed files for: `as any`, empty catches, console.log in prod, commented-out code. Verify zero `ignition::` references in simulator code. Verify zero `CMAKE_CXX_STANDARD 14` in active packages. Check for `find_package(PythonLibs)` anywhere.
  Output: `Build [PASS/FAIL] | Tests [N pass/N fail] | Files [N clean/N issues] | VERDICT`

- [ ] F3. **Real Manual QA** — `unspecified-high`
  Start from clean state. Launch simulation headless: `ros2 launch rmu_gazebo_simulator bringup_sim.launch.py headless:=true`. Unpause Gazebo. Launch navigation: `ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmul_2026 slam:=False`. Verify TF chain with `tf2_echo`. Send nav goal. Verify robot moves. Check velocity topics (TwistStamped on `cmd_vel_nav2_result`, Twist on `/cmd_vel`). Save screenshots to `.sisyphus/evidence/final-qa/`.
  Output: `TF Chain [PASS/FAIL] | Navigation [PASS/FAIL] | Velocity Pipeline [PASS/FAIL] | VERDICT`

- [ ] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual diff. Verify 1:1 — everything in spec was built, nothing beyond spec was built. Check "Must NOT do" compliance. Detect cross-task contamination. Flag unaccounted changes. Verify no navigation parameters were changed (diff nav2_params.yaml against pre-migration baseline, only `enable_stamped_cmd_vel` should be new).
  Output: `Tasks [N/N compliant] | Contamination [CLEAN/N issues] | Unaccounted [CLEAN/N files] | VERDICT`

---

## Commit Strategy

| Phase | Commit Message | Key Files |
|-------|---------------|-----------|
| 0 | `refactor: 移除未使用的 TEB/costmap_converter 包` | `src/teb_local_planner/`, `src/costmap_converter/` |
| 0 | `docs: 清理 TEB 注释配置，更新文档索引` | `nav2_params.yaml`, `AGENTS.md`, `README.md` |
| 1 | `fix(point_lio): 替换废弃 PythonLibs，清理冗余 CMake 标志` | `point_lio/CMakeLists.txt` |
| 1 | `build: 统一自研包 C++ 标准为 C++17` | 多个 `CMakeLists.txt` |
| 1 | `build: 统一第三方 fork 包 C++ 标准为 C++17` | 多个 `CMakeLists.txt` |
| 2 | `refactor(sentry_behavior): BT::NodeConfiguration → BT::NodeConfig` | `sentry_behavior/include/**/*.hpp`, `sentry_behavior/plugins/**/*.cpp` |
| 2 | `build: 更新 BehaviorTree.ROS2 到 Jazzy 兼容分支` | `.repos` 文件 |
| 3 | `feat(fake_vel_transform): 升级输入为 TwistStamped 适配 Jazzy Nav2` | `fake_vel_transform/src/*.cpp` |
| 3 | `feat: 启用 Nav2 TwistStamped 参数` | `nav2_params.yaml` × 2 |
| 3 | `fix(nav2_plugins): 适配 Jazzy Nav2 TwistStamped API` | `back_up_free_space.cpp` |
| 3 | `fix(sentry_tools): 适配 TwistStamped 话题订阅` | `serial_visualizer.py` |
| 4 | `feat(simulator): 迁移 rmoss_gz_plugins 到 Gazebo Harmonic` | `rmoss_gz_plugins/**` |
| 4 | `feat(simulator): 迁移 rmoss_gz_base 到 Gazebo Harmonic` | `rmoss_gz_base/**` |
| 4 | `feat(simulator): 迁移 rmoss_gz_bridge/cam 到 Gazebo Harmonic` | `rmoss_gz_bridge/**`, `rmoss_gz_cam/**` |
| 4 | `feat(simulator): 更新 SDF 世界/模型文件为 Gz Harmonic 格式` | `*.sdf` |
| 4 | `feat(simulator): 更新 env-hooks/scripts/launch 为 gz CLI` | `*.dsv.in`, `*.sh`, `*_launch.py` |
| 4 | `feat: 适配 ign_sim_pointcloud_tool 到 Gz Harmonic` | `ign_sim_pointcloud_tool/**` |
| 4 | `build(simulator): 升级 rmoss_core 到 C++17` | `rmoss_core/**/CMakeLists.txt` |
| 5 | `build: 更新 setup_env.sh 支持 Ubuntu 24.04 + Jazzy` | `setup_env.sh` |
| 5 | `build: 更新 Dockerfiles 为 Jazzy 基础镜像` | `Dockerfile` × 2 |
| 5 | `build: 更新 .repos 和 CI 到 Jazzy` | `.repos`, `.github/workflows/` |
| 5 | `docs: 全面更新文档至 Ubuntu 24.04 + Jazzy + Gz Harmonic` | `AGENTS.md`, `README.md`, `CLAUDE.md`, `docs/*` |
| 6 | `perf: 将 terrain_analysis/ext 组合进 nav2_container` | launch 文件 |
| 6 | `perf: 启用 intra-process 通信优化速度链路` | launch 文件 |
| 6 | `docs: 验证并记录 Jazzy 下频率校准结果` | `.sisyphus/evidence/frequency-calibration.md` |

---

## Success Criteria

### Verification Commands
```bash
# 全量编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# Expected: 0 failures

# 测试
colcon test && colcon test-result --verbose
# Expected: 无新增失败

# TF 链验证
ros2 run tf2_ros tf2_echo map chassis
# Expected: 有效变换输出

# 仿真启动
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py headless:=true
# Expected: Gazebo 正常启动

# 导航启动
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py world:=rmul_2026 slam:=False
# Expected: Nav2 所有节点正常启动

# 速度链类型验证
ros2 topic info /cmd_vel_nav2_result --verbose | grep Type
# Expected: geometry_msgs/msg/TwistStamped

ros2 topic info /cmd_vel --verbose | grep Type
# Expected: geometry_msgs/msg/Twist

# 无 Ignition 残留
grep -r "ignition::" src/simulator/ --include="*.cpp" --include="*.hpp" --include="*.hh" --include="*.cc" | wc -l
# Expected: 0

# 无 C++14 残留
grep -r "CMAKE_CXX_STANDARD 14" src/*/CMakeLists.txt src/sentry_nav/*/CMakeLists.txt | wc -l
# Expected: 0
```

### Final Checklist
- [ ] All "Must Have" present
- [ ] All "Must NOT Have" absent
- [ ] All tests pass
- [ ] TF chain intact
- [ ] Simulation navigates successfully
- [ ] Velocity pipeline types correct
- [ ] Zero `ignition::` references in active code
- [ ] Zero C++14 in active packages
- [ ] All documentation updated

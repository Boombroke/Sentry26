# 快速部署与上手指南

> **Maintainer**: Boombroke <boombroke@icloud.com>
> 如需了解系统各模块的详细架构设计，请参阅 [系统架构详解](ARCHITECTURE.md)。
> 参数调优见 [参数调优指南](TUNING_GUIDE.md)。

## 0. 一键配置（推荐）

全新的 Ubuntu 24.04 系统可直接运行一键配置脚本：

```bash
bash src/scripts/setup_env.sh
```

脚本会依次执行：
1. **安装 ROS2 Jazzy**: 添加官方 APT 源，安装 `ros-jazzy-desktop` 和 `ros-dev-tools`
2. **安装 Gazebo Harmonic**: 通过 `ros-jazzy-ros-gz` 安装
3. **安装系统依赖**: Eigen3、OpenMP、PCL、Nav2、SLAM Toolbox、serial-driver 等
4. **安装差速控制器 apt 包**: `ros-jazzy-nav2-regulated-pure-pursuit-controller`、`ros-jazzy-nav2-rotation-shim-controller`
5. **编译安装 small_gicp v1.0.0**: 从 GitHub 克隆并编译（需要 C++17）
6. **初始化 rosdep**
7. **同步源码包到工作空间**: 会逐项补齐后续新增包（例如 `sentry_motion_manager`）
8. **编译工作空间**: `colcon build --symlink-install`
9. **配置 bashrc（可选）**

如果希望手动逐步配置，继续阅读下面章节。

## 1. 环境要求

| 项 | 版本 |
|---|---|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8) |
| C++ | C++17 |
| Python | 3.12+ |
| 硬件（实车） | Livox Mid360 + 差速轮足底盘 + BMI088 IMU |

## 2. 源码编译部署

### 2.1 安装 small_gicp（重定位依赖）

```bash
sudo apt install -y libeigen3-dev libomp-dev
git clone https://github.com/koide3/small_gicp.git
cd small_gicp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

### 2.2 安装 xmacro（机器人模型解析）

```bash
pip3 install xmacro --break-system-packages
```

### 2.3 安装 Nav2 差速控制器

```bash
sudo apt install -y ros-jazzy-nav2-regulated-pure-pursuit-controller \
                    ros-jazzy-nav2-rotation-shim-controller
```

### 2.4 安装 ROS 依赖

```bash
cd <path_to_sentry26>
rosdep install -r --from-paths src --ignore-src --rosdistro jazzy -y
```

### 2.5 编译

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> 推荐始终带 `--symlink-install`，yaml/xmacro 修改后无需重新编译。

## 3. 仿真模式

> **Wayland 用户注意**：Ubuntu 24.04 默认 Wayland，Gazebo GUI 的 Play 按钮可能无法点击。前面加 `QT_QPA_PLATFORM=xcb` 环境变量即可解决。

### 3.1 两终端启动（唯一方式）

仿真端时序敏感（Gazebo spawn → unpause → 传感器流稳定大约需要 10s，Point-LIO 收不到 IMU 就初始化失败），因此**必须两终端手动启动**：

```bash
# === 终端 1：启动 Gazebo（可加 headless:=true 无 GUI） ===
QT_QPA_PLATFORM=xcb ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# 启动后另开一个终端 unpause Gazebo（等 5~10s 让机器人 spawn 完）：
gz service -s /world/default/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'pause: false'

# 再等 ~10 秒让 /clock 稳定，看到 ros2 topic hz /red_standard_robot1/livox/imu 能出数再起下一步

# === 终端 2：启动导航栈 ===
# 首次跑（SLAM 实时建图）：
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=True

# 有图后切换到纯定位模式：
ros2 launch sentry_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2026 slam:=False
```

### 3.3 保存建图结果

建图完成后：

```bash
# 保存 2D 占据栅格（slam_toolbox）
ros2 run nav2_map_server map_saver_cli -f <map_name> \
  --ros-args -r __ns:=/red_standard_robot1

# 保存 PCD 点云（关闭 Point-LIO 节点时自动保存到 ~/.ros 或 PCD/ 目录）
```

先验 PCD 必须与 `registered_scan` 一样处于 odom/map 系；旧流程生成的 `lidar_odom` 系 PCD 不再兼容，需要重新建图生成。

### 3.4 发一个 Nav Goal 验证

```bash
ros2 action send_goal /red_standard_robot1/navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

差速车会**先原地旋转到目标方向**（RotationShim 触发），然后直线前进（RPP 跟随）。`SUCCEEDED` 意味着到达 xy_goal_tolerance 内。

### 3.5 可用仿真世界

| 世界 | 说明 |
|---|---|
| `rmuc_2026` | 2026 对抗赛场地 |
| `rmul_2026` | 2026 联盟赛场地 |
| `rmuc_2025` | 2025 对抗赛场地（保留） |
| `empty_world` | 空地面调试 |

## 4. 实车模式

**实车端零代码改动，只需配置即可**。Stage A-H 已确保所有源码参数默认值适配差速轮足。

### 4.1 一键启动

```bash
# 首次（建图）
ros2 launch sentry_nav_bringup rm_sentry_launch.py slam:=True

# 有图后导航
ros2 launch sentry_nav_bringup rm_sentry_launch.py world:=<map_name> slam:=False
```

`rm_sentry_launch.py` 自动启动：
- rm_serial_driver 串口通信
- Point-LIO 激光惯性里程计
- small_gicp_relocalization 全局重定位
- Nav2 栈（RPP + RotationShim + velocity_smoother）
- sentry_motion_manager 底盘速度仲裁（`cmd_vel_nav` → 最终 `/cmd_vel`）
- robot_state_publisher + LiDAR 驱动
- sentry_behavior 行为树

### 4.2 跨团队协作（实车）

**下位机固件必须同步升级**：

- 串口协议 imu 包新增 4 字段（`chassis_yaw/pitch` + `gimbal_yaw/pitch`），包大小 11B → 27B
- 电控端用 `src/serial/serial_driver/example/navigation_auto.h` 重编固件
- 旧固件会因 CRC 不匹配被丢包

**实车机械参数与外参**：

- 实车 TF 入口：`src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro`
- 仿真 Gazebo 入口：`src/sentry_robot_description/resource/xmacro/wheeled_biped_sim.sdf.xmacro`
- 共享拓扑核心：`src/sentry_robot_description/resource/xmacro/wheeled_biped_core.sdf.xmacro`

LiDAR 斜装、倒装或实测平移只改 `wheeled_biped_real.sdf.xmacro` 里的 `front_lidar_pose`，不要用 Point-LIO `gravity` 表达安装角。仿真为了扫到近场低矮底座保留 30° 下俯角，这个角只存在于 `wheeled_biped_sim.sdf.xmacro`。

当前实车默认值：

- `front_lidar_pose = -0.05 0 0.05 0.0 0.2617993877991494 3.141592653589793`
- `pitch > 0` 表示下俯，因此这里表示 Mid360 相对 `gimbal_pitch` 下俯 15°
- 实车 `chassis -> gimbal_yaw -> gimbal_pitch` 现在是静态 TF，不再默认消费 `serial/gimbal_joint_state` 驱动 robot_state_publisher

外参修改前建议先看 `src/sentry_robot_description/README.md` 里的“外参与 DOF 怎么理解 / 角度为什么是 0.261799 / 常见安装场景怎么写”三节，里面已经明确了：

- `front_lidar_pose` 用的是 `x y z roll pitch yaw`
- 角度必须先转弧度
- 当前实车固定云台时只改 `gimbal_pitch -> front_mid360` 外参
- 倒装/反装也仍然只走 TF 外参，不走 Point-LIO `gravity`

## 5. 行为树决策

```bash
ros2 launch sentry_behavior sentry_behavior_launch.py
```

行为树 XML 定义在：
- `src/sentry_nav_bringup/behavior_trees/` —— Nav2 行为树（navigate_to_pose 等）
- `src/sentry_behavior/behavior_trees/` —— 战术决策（rmul.xml / RMUC.xml 等）

## 6. 调试工具

```bash
# 统一工具箱（串口 Mock / 地图拾取 / 连通性检测 / 重力标定）
python3 src/sentry_tools/sentry_toolbox.py

# 串口数据实时可视化（需 ROS 环境）
source install/setup.bash
python3 src/sentry_tools/serial_visualizer.py
```

详见 [sentry_tools/README.md](../sentry_tools/README.md)。

## 7. Debug 与日志

所有 launch 日志位于 `~/.ros/log/<timestamp>/`，每次启动一个目录。

```bash
# 实时跟随最新 launch 的合流日志
bash src/scripts/debug/tail_log.sh

# 跟随指定节点（按名字模糊匹配）
bash src/scripts/debug/tail_log.sh pointlio
bash src/scripts/debug/tail_log.sh controller_server

# 跨所有节点日志搜关键字
bash src/scripts/debug/grep_log.sh error
bash src/scripts/debug/grep_log.sh "TF lookup"

# 归档到项目 logs/ 目录便于对比调参
bash src/scripts/debug/archive_log.sh baseline
# 改参数重跑...
bash src/scripts/debug/archive_log.sh tuned
diff logs/*-baseline/launch.log logs/*-tuned/launch.log | head
```

详见 [logs/README.md](../../logs/README.md)。

## 8. 常见问题

- **编译错误**：确认 `small_gicp` v1.0.0 已安装并位于系统路径；rosdep 已安装 `ros-jazzy-nav2-regulated-pure-pursuit-controller` 和 `ros-jazzy-nav2-rotation-shim-controller`。

- **仿真 Gazebo Play 按钮无响应**：Wayland 已知问题，用 `QT_QPA_PLATFORM=xcb` 或命令行 unpause（见 3.2）。

- **Point-LIO 报 `imu loop back, clear deque`**：仿真启动时序问题。先起 Gazebo 并 unpause，等仿真时钟稳定（~10s，观察 `/red_standard_robot1/livox/imu` 有 ~150Hz 输出）再起导航栈。IMU 初始化完成后会自动恢复。

- **Nav Goal accepted 但机器人不动**：
  - 检查 RPP 的 `use_collision_detection`：SLAM 模式下若 costmap 大面积 unknown 会误报碰撞，本项目已在 `nav2_params.yaml` 中关闭（建图完成后可重开）。
  - 检查 `/<ns>/cmd_vel_nav` 是否有 Nav2 平滑输出，以及 `/<ns>/motion_manager/state` 中 `output_enabled=true`。
  - 检查 `/<ns>/cmd_vel` topic 是否单类型（`TwistStamped`，无混合）：`ros2 topic info /red_standard_robot1/cmd_vel`。

- **启动时报 `package 'sentry_motion_manager' not found`**：说明当前 workspace 没有同步或编译底盘运动管理器包。若源码中已有 `src/sentry_motion_manager/`，执行：
  ```bash
  cd ~/sentry_ws
  colcon build --symlink-install --packages-select sentry_motion_manager sentry_nav_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.bash
  ros2 pkg prefix sentry_motion_manager
  ```
  若 `--packages-select` 报找不到包，先同步最新仓库或重新运行 `bash src/scripts/setup_env.sh`，脚本会补齐旧 workspace 中缺失的新增源码包链接。

- **RViz 中无法显示**：检查 namespace 与启动参数的机器人名称一致。

- **先验 PCD/2D 地图缺失**：仓库不含大地图文件，首次运行用 `slam:=True` 自行建图，或从团队内部 Google Drive/百度网盘下载。

- **实车底盘不响应 cmd_vel**：
  - 下位机固件是否升级（imu 包 27B）
  - `ros2 topic info /cmd_vel` 的 subscriber 是否有 `rm_serial_driver_node`
  - `/cmd_vel` 类型须为 `geometry_msgs/msg/TwistStamped`，且由 `motion_manager` 发布

- **重定位后机器人在 RViz 中位置异常**：确认先验 PCD 与 2D 占据栅格在同一 odom/map 坐标系和坐标原点（从同一出生点建图）；旧 `lidar_odom` 系 PCD 必须重新生成。

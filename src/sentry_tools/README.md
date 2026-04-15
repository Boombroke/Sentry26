# sentry_tools — 哨兵机器人调试工具集

## 快速开始

```bash
# 工具箱（串口 Mock + 地图拾取 + 连通性检测 + 串口诊断 + 重力标定），无需 ROS（重力标定需 ROS）
python3 src/sentry_tools/sentry_toolbox.py

# 串口数据可视化（需 ROS 环境）
source install/setup.bash
python3 src/sentry_tools/serial_visualizer.py
```

依赖：`pip install pyserial`（PyQt5/matplotlib/numpy/pyyaml 随 ROS2 Humble 自带）

---

## 工具箱 (sentry_toolbox.py)

五个标签页，暗色主题。串口 Mock / 地图拾取与编辑 / 连通性检测 / 串口诊断无需 ROS；重力标定需要 ROS 环境。

### 标签页 1：串口 Mock

模拟电控向上位机发送协议包，用于脱离硬件调试。

**操作步骤：**

1. 顶栏选择串口和波特率，点击 Connect
2. 在 IMU/Status/HP 子页签调整字段值
3. 勾选 Enable 并设置发送周期
4. 底部「接收显示」实时显示 ROS 回传的速度指令

**发送的包：**

| 标签页 | 帧头 | 默认周期 | 可调字段 |
|---|---|---|---|
| IMU | 0xA1 | 20ms | pitch, yaw（滑块 ±3.14） |
| Status | 0xA2 | 100ms | 比赛阶段(下拉)、剩余时间、血量、弹量、红蓝队(单选)、RFID(勾选) |
| HP | 0xA3 | 500ms | 7 个己方血量（1/2/3/4/7号 + 前哨 + 基地） |

**控件是动态生成的**——修改 `protocol.yaml` 并重新生成后，新字段会自动出现。

**虚拟串口联调：**

```bash
# 终端 1：创建虚拟串口对
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# 输出 /dev/pts/3 和 /dev/pts/4

# 终端 2：工具箱连接一端
python3 src/sentry_tools/sentry_toolbox.py
# 选择 /dev/pts/3，Connect

# 终端 3：被测程序连接另一端
ros2 launch serial_driver serial_driver.launch.py device_name:=/dev/pts/4
```

### 标签页 2：地图拾取与编辑

加载地图文件，支持坐标拾取和 .pgm 地图直接编辑。通过工具栏切换拾取模式与编辑模式。

#### 坐标拾取

1. 工具栏选择「拾取」模式（默认）
2. 点击「选择地图」，选择 `.yaml` 地图文件（如 `sentry_nav_bringup/map/rmul_2026.yaml`）
3. 地图渲染后，左键点击标记坐标点（绿色 X）
4. 右侧列表显示所有已拾取坐标
5. 选中列表项，点击「复制选中」复制到剪贴板
6. 「清除标记」清空地图标记 + 列表

拾取的坐标可直接填入行为树的目标位置配置。

#### 地图编辑

工具栏提供 5 种编辑工具：

| 工具 | 功能 | 像素值 | 操作方式 |
|---|---|---|---|
| 障碍物 | 绘制墙壁/障碍 | 0（黑色） | 按住左键拖拽 |
| 自由空间 | 清除障碍 | 254（白色） | 按住左键拖拽 |
| 未知区域 | 标记为未知 | 128（灰色） | 按住左键拖拽 |
| 直线 | 画直线障碍 | 0（黑色） | 点击起点 → 点击终点 |
| 矩形 | 画矩形障碍 | 0（黑色） | 点击一角 → 点击对角 |

**笔刷大小**：滑块调节 1~20 像素半径，适用于画笔/橡皮擦/未知区域/直线工具。

**形状工具**：点击第一个点后，鼠标移动实时预览虚线轮廓（红色），点击第二个点确认绘制。右键取消。

**撤销/重做**：`Ctrl+Z` 撤销，`Ctrl+Y` 重做，最多保留 50 步历史。

**保存**：
- 「保存」覆盖写回原 .pgm 文件
- 「另存为」选择新路径保存
- 切换地图时如有未保存编辑会提示确认

**注意**：编辑模式下 matplotlib 的平移/缩放工具自动禁用，避免冲突。切回「拾取」模式即可恢复平移/缩放。

### 标签页 3：连通性检测

一键检测系统各链路是否正常。

**运行模式：**

| 模式 | 条件 | 检测项 |
|---|---|---|
| 基础模式 | 默认 | 串口设备 |
| ROS 模式 | 已 source ROS 环境 | 串口 + 节点 + Topic + TF |

**操作步骤：**

1. 点击「立即刷新」执行一次检测
2. 勾选「自动刷新」开启定时检测（默认 5 秒）
3. 勾选「详细频率检测」获取 Topic 实际频率（较慢）

**检测项：**

| 分组 | 检测内容 | 状态 |
|---|---|---|
| 串口设备 | `/dev/ttyACM*`, `/dev/ttyUSB*` | ✅ 存在 / ❌ 无设备 |
| 关键节点 | serial_driver, controller_server, planner_server, bt_navigator, behavior_server, odom_bridge 等 | ✅ 运行中 / ❌ 未检测到 |
| Topic 状态 | gimbal_joint_state, cmd_vel, odometry, obstacle_scan, terrain_map, referee/* | ✅ 有发布者 / ⚠️ 频率低 / ❌ 无发布者 |
| TF 链路 | map→odom, odom→chassis, chassis→gimbal_yaw | ✅ 正常 / ❌ 断开 |

**底部汇总**：`总览: 12/15 正常  ⚠️ 1 警告  ❌ 2 异常`

**一键修复：**

检测失败时，每个异常项旁边会显示：
- 灰色提示文字（问题原因 + 修复建议）
- 蓝色「修复」按钮（点击后在终端执行修复脚本）

顶部「一键修复」按钮执行全量环境配置（`setup_env.sh`）。

**修复脚本对照：**

| 问题 | 修复脚本 | 作用 |
|---|---|---|
| ROS2 未安装 | `bash src/scripts/fix_ros_env.sh` | 安装 ROS2 Humble |
| Nav2 节点缺失 | `bash src/scripts/fix_nav2_deps.sh` | 安装 Nav2 + 导航依赖 |
| 串口无权限 | `bash src/scripts/fix_serial_permission.sh` | 加入 dialout 组 + 设置设备权限 |
| 编译问题 | `bash src/scripts/fix_build.sh` | 清理缓存 + rosdep + 全量重编 |
| 以上全部 | `bash src/scripts/setup_env.sh` | 一键全量环境配置 |

**典型排查流程：**

| 现象 | 看哪里 | 修复 |
|---|---|---|
| 机器人不动 | `/cmd_vel` topic + controller_server | 启动导航 launch |
| 导航无路径 | planner_server + obstacle_scan + terrain_map | `fix_nav2_deps.sh` + 启动 launch |
| 定位漂移 | `odom→chassis` TF + odometry topic | 检查 point_lio 节点 |
| 裁判系统异常 | 串口设备 + `referee/*` topic | `fix_serial_permission.sh` |
| 行为树不执行 | sentry_behavior_server + referee topic | `fix_build.sh` |
| 所有都红 | 全部 | `setup_env.sh`（一键修复按钮）|

### 标签页 4：串口诊断

连接真实串口，被动监听链路质量。**独立串口连接**，不与 Mock tab 共享。

**操作步骤：**

1. 选择真实串口设备（如 `/dev/ttyACM0`）和波特率，点击 Connect
2. 面板自动开始统计，每 500ms 刷新
3. 观察智能告警面板（顶部）获取问题诊断
4. 点击「重置统计」清零重新统计

**面板说明：**

| 面板 | 内容 | 默认 |
|---|---|---|
| 智能告警 | 自动检测波特率不匹配、协议错误、CRC 异常等 | 展开 |
| 链路总览 | 运行时间、吞吐量、废字节率、总丢包率 | 展开 |
| 各包类型统计 | 0xA1/0xA2/0xA3 成功/CRC错误/丢包率/频率 | 展开 |
| 包间隔分析 | 平均/最小/最大/抖动，异常标红 | 折叠 |
| 高级诊断 | 帧同步失败、缓冲区溢出、连续CRC错误等 7 项 | 折叠 |
| 实时速率曲线 | 30s 滚动图，每类包一条线 | 展开 |

**智能告警自动检测：**

| 告警 | 触发条件 |
|---|---|
| 🔴 疑似波特率不匹配 | 运行>2s 无有效包 + 废字节率极高 |
| 🔴 疑似协议不匹配 | >1KB 数据但 0 有效包 |
| 🔴 CRC 错误率过高 | >5% |
| 🟡 连续 CRC 错误 | 峰值≥5 |
| 🔴 最长无包间隔 | >2s |
| 🟡 缓冲区溢出 | >0 次 |
| 🟡 IMU 数据跳变 | >5 次 |
| 🟢 链路正常 | 运行>3s 无异常 |

### 标签页 5：重力标定

采集 Livox mid360 内置 BMI088 IMU 的加速度数据，计算 Point-LIO 所需的 `gravity` 参数向量。**需要 ROS 环境**（通过 `ros2 topic echo` 采集数据）。

**背景：** Point-LIO 启动时通过重力对齐确定初始坐标系方向。`gravity` 参数定义了 IMU 静止时测量到的加速度向量（含重力）。如果该参数与实际不匹配，会导致建图时地图方向不一致。**每次更换雷达或调整安装角度后都需要重新标定。**

**一键标定（推荐）：**

1. 打开工具箱，切换到「重力标定」标签页
2. 点击「检测雷达连接」确认 LiDAR 可达（自动读取配置中的 LiDAR IP）
3. 点击「一键标定」：自动执行
   - 雷达连通性检测（ping）
   - 检查 Livox 驱动是否已运行
   - 如未运行则自动启动 `ros2 launch livox_ros_driver2 msg_MID360_launch.py`
   - 轮询等待 `/livox/imu` 话题就绪（15s 超时）
   - 自动开始采集并在完成后自动清理由工具箱拉起的驱动
4. 过程日志显示在上方日志框（带时间戳）

**手动采集（兼容旧流程）：**

1. 确保雷达已安装到最终位置，机器人放在水平地面上，完全静止
2. 只需启动 Livox 驱动：`ros2 launch livox_ros_driver2 msg_MID360_launch.py`
3. 打开工具箱，切换到「重力标定」标签页
4. 确认 IMU Topic（默认 `livox/imu`）和采样数（默认 1000）
5. 点击「开始采集」，等待进度条完成
6. 观察标准差（std）：任一轴 >0.05 说明机器人不够静止，需要重新采集
7. 采集完成后，点击「复制 YAML」将结果复制到剪贴板
8. 粘贴到 `config/reality/nav2_params.yaml` 的 `mapping:` 段替换 `gravity` 和 `gravity_init`

**也可直接写入配置文件：** 点击「写入配置」→ 选择 `nav2_params.yaml` → 自动替换 `mapping:` 段内的 `gravity` 和 `gravity_init` 值。

**UI 说明：**

| 区域 | 内容 |
|---|---|
| 一键标定区 | 检测雷达连接 / 一键标定 / 停止 + 状态指示 + 执行日志 |
| 顶栏 | IMU Topic 输入、采样数设置、开始/停止按钮、状态指示 |
| 实时采集 | 进度条、当前/均值/标准差实时显示、X-Y/Y-Z/X-Z 散点投影图 |
| 标定结果 | acc_norm 选择（1.0g / 9.81m/s²）、gravity 向量、norm 值（颜色编码）、复制/写入按钮 |

**acc_norm 选择：**

| 设置 | 含义 | 实车 mid360 | 仿真 |
|---|---|---|---|
| 1.0 | IMU 加速度单位为 g | ✅ 选这个 | — |
| 9.81 | IMU 加速度单位为 m/s² | — | ✅ 选这个 |

norm 值应接近所选的 acc_norm 值（绿色=正常，黄色=偏差 3-8%，红色=偏差 >8%）。

---

## 数据可视化 (serial_visualizer.py)

订阅 ROS topic 实时显示导航速度、裁判系统等数据。暗色主题，仿真和实车通用。

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash

# 仿真环境（话题在 namespace 下）
python3 src/sentry_tools/serial_visualizer.py --ros-args -r __ns:=/red_standard_robot1

# 实车环境（话题在根 namespace）
python3 src/sentry_tools/serial_visualizer.py
```

**需要先启动导航栈或 serial_driver 节点。**

| 区域 | 内容 |
|---|---|
| 左1 | 云台 pitch/yaw 滚动曲线（10s 窗口） |
| 左2 | Vx 对比：命令（实线）vs 实际（虚线） |
| 左3 | Vy 对比：命令（实线）vs 实际（虚线） |
| 左4 | Vw 对比：命令（实线）vs 实际（虚线） |
| 右上 | 比赛阶段 + 倒计时进度条 |
| 右中 | 自身 HP + 弹量 / 导航命令速度 / 实际速度 / 跟踪误差 / 最终下发速度 |
| 右下 | 全队 7 条 HP 柱状条 |
| 底部 | 各 topic 收包状态 ✓/✗ + UI 刷新率 |

**订阅的 topic（均为相对名，受 namespace 控制）：**

| Topic | 用途 |
|---|---|
| `cmd_vel_nav2_result` | 导航命令速度（world 系，fake_vel_transform 之前，无自旋） |
| `cmd_vel` | 最终下发速度（body 系 + spin_speed，fake_vel_transform 之后） |
| `odometry` | 实际速度（world 系，odom_bridge 位置差分） |
| `serial/gimbal_joint_state` | 云台关节状态 |
| `referee/game_status` | 比赛阶段 + 倒计时 |
| `referee/robot_status` | 血量 + 弹量 |
| `referee/all_robot_hp` | 全队血量 |

> ⚠️ 速度对比只能用 `cmd_vel_nav2_result`（world 系）vs `odometry`（world 系）。不能用 `cmd_vel`（body 系+自旋 3.14 rad/s），详见 AGENTS.md 第 4 节。

**完整联调 4 终端：**

```bash
# A: 虚拟串口
socat -d -d pty,raw,echo=0 pty,raw,echo=0

# B: ROS 串口驱动
ros2 launch serial_driver serial_driver.launch.py device_name:=/dev/pts/4

# C: 串口 Mock（工具箱连另一端）
python3 src/sentry_tools/sentry_toolbox.py

# D: 数据可视化（实车）
python3 src/sentry_tools/serial_visualizer.py
```

---

## 协议扩展

串口协议定义在 `src/serial/serial_driver/protocol/protocol.yaml`。

**修改协议的流程：**

```bash
# 1. 编辑唯一真相源
vim src/serial/serial_driver/protocol/protocol.yaml

# 2. 生成代码
cd src/serial/serial_driver/protocol && python3 generate.py

# 3. 部署到各位置
cp generated/packet.hpp ../include/rm_serial_driver/
cp generated/navigation_auto.h ../example/
cp generated/protocol.py ../../../sentry_tools/

# 4. 编译 ROS 端
cd /path/to/workspace && colcon build --packages-select rm_serial_driver
```

工具箱 GUI 控件自动跟随 protocol.py 变化，无需改代码。

---

## 文件结构

```text
src/sentry_tools/
├── sentry_toolbox.py          # 工具箱（串口Mock + 地图拾取 + 连通性检测 + 串口诊断）
├── serial_visualizer.py       # 串口数据实时可视化（需 ROS）
├── protocol.py                # 协议定义（由 generate.py 生成，勿手动编辑）
└── README.md

src/serial/serial_driver/protocol/
├── protocol.yaml              # 协议唯一真相源
├── generate.py                # 代码生成器
├── templates/                 # Jinja2 模板
│   ├── packet.hpp.j2
│   ├── navigation_auto.h.j2
│   └── protocol_py.j2
└── generated/                 # 生成产物
```

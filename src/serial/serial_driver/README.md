# rm_serial_driver

## 1. 概述

rm_serial_driver 是一个 ROS2 串口驱动包，作为 Nav2 导航栈与 STM32 电控端之间的通信桥梁。该包通过 USB CDC 或 UART 接口实现双向数据传输，将底盘速度指令下发至机器人底盘，并实时接收云台姿态、比赛状态及血量等关键信息。该节点采用 Composable Node 模式开发，支持高效的进程内通信。

## 2. 目录结构

```
serial/serial_driver/
├── config/serial_driver.yaml        # 串口参数配置
├── example/                         # STM32 电控端示例代码
│   ├── navigation_auto.h
│   ├── navigation_auto.c
│   └── README.md
├── include/rm_serial_driver/
│   ├── packet.hpp                   # 收发包结构体（由 generate.py 生成）
│   ├── rm_serial_driver.hpp         # 节点类声明
│   └── crc.hpp                      # CRC16 校验
├── launch/serial_driver.launch.py   # 启动文件
├── protocol/
│   ├── protocol.yaml                # 协议唯一真相源
│   ├── generate.py                  # 代码生成器
│   ├── templates/                   # Jinja2 模板
│   │   ├── packet.hpp.j2
│   │   ├── navigation_auto.h.j2
│   │   └── protocol_py.j2
│   └── generated/                   # 生成产物
├── src/
│   ├── rm_serial_driver.cpp         # 节点实现
│   └── crc.cpp                      # CRC16 实现
├── CMakeLists.txt
└── package.xml
```

## 3. 协议定义

### 帧格式
`[header 1B] [payload] [crc16 2B]`
- CRC16 校验覆盖整帧数据（包含帧头）。
- CRC16 以小端序（Little-endian） 2 字节附加在每包末尾。
- 所有结构体使用 `__attribute__((packed))` 强制字节对齐，无填充。

### 数据包列表

| 帧头 | 包名 | 方向 | 大小 | 频率 | 字段列表 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 0xA1 | IMU | stm32→ros | 11B | ~1000Hz | pitch(float,rad), yaw(float,rad) |
| 0xA2 | Status | stm32→ros | 12B | ~10Hz | game_progress(u8), stage_remain_time(u16,s), current_hp(u16), projectile_allowance_17mm(u16), team_colour(u8, 1=red 0=blue), rfid_base(u8) |
| 0xA3 | HP | stm32→ros | 17B | ~2Hz | ally_1..4_robot_hp, ally_7_robot_hp, ally_outpost_hp, ally_base_hp (均为 u16) |
| 0xB5 | Nav | ros→stm32 | 15B | 随 cmd_vel | vel_x(float,m/s), vel_y(float,m/s), vel_w(float,rad/s) |

## 4. ROS 接口

| 话题 | 类型 | 方向 | 说明 |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | geometry_msgs/msg/Twist | Subscription | 接收速度指令，封装为 Nav 包发送至 STM32 |
| `serial/gimbal_joint_state` | sensor_msgs/msg/JointState | Publication | 发布从 IMU 包解析的云台 pitch/yaw 角度 |
| `referee/game_status` | rm_interfaces/msg/GameStatus | Publication | 发布从 Status 包解析的游戏阶段及剩余时间 |
| `referee/robot_status` | rm_interfaces/msg/RobotStatus | Publication | 发布从 Status 包解析的当前血量及剩余弹量 |
| `referee/all_robot_hp` | rm_interfaces/msg/GameRobotHP | Publication | 发布从 HP 包解析的 7 个己方单位血量值 |
| `referee/rfidStatus` | rm_interfaces/msg/RfidStatus | Publication | 发布从 Status 包解析的 RFID 基地增益状态 |

## 5. 参数配置

参数定义于 `config/serial_driver.yaml`：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| device_name | /dev/ttyACM0 | 串口设备路径 |
| baud_rate | 115200 | 波特率 |
| flow_control | none | 流控模式 |
| parity | none | 校验位 |
| stop_bits | "1" | 停止位 |
| enable_vel_log | false | 启用速度下发 CSV 日志（详见第 11 节） |

## 6. 编译与使用

### 编译
```bash
colcon build --packages-select rm_serial_driver --symlink-install
```

### 启动
```bash
ros2 launch rm_serial_driver serial_driver.launch.py
```

### 带参数覆盖启动
```bash
ros2 launch rm_serial_driver serial_driver.launch.py device_name:=/dev/ttyUSB0
```
该节点已注册为 Composable Node，通常由主导航启动文件（如 `rm_navigation_reality_launch.py`）自动拉起。

## 7. 协议修改流程

1. 修改 `protocol/protocol.yaml`（协议唯一真相源）。
2. 执行生成脚本：
   ```bash
   cd protocol && python3 generate.py
   ```
3. 手动同步生成的代码文件：
   - `generated/packet.hpp` → `include/rm_serial_driver/packet.hpp`
   - `generated/navigation_auto.h` → `example/navigation_auto.h`
   - `generated/protocol.py` → `../../sentry_tools/protocol.py`
4. 重新编译 ROS 包：
   ```bash
   colcon build --packages-select rm_serial_driver
   ```
5. 使用新的 `navigation_auto.h` 和 `navigation_auto.c` 更新 STM32 电控端代码。

注意：`sentry_tools` 工具箱的 GUI 控件会根据 `protocol.py` 自动更新。

## 8. 串口调试

- **虚拟串口测试**：使用 socat 创建虚拟串口对进行闭环测试：
  ```bash
  socat -d -d pty,raw,echo=0 pty,raw,echo=0
  ```
- **模拟电控**：使用 `sentry_toolbox.py` 的 "串口 Mock" 选项卡模拟 STM32 发送数据。
- **链路诊断**：使用 `sentry_toolbox.py` 的 "串口诊断" 选项卡监控实时通信质量与丢包率。
- **常见问题**：若提示权限不足，请执行 `sudo chmod 666 /dev/ttyACM0` 或将当前用户加入 `dialout` 用户组。

## 9. 电控端集成

电控端集成代码位于 `example/` 目录，提供了基于 FreeRTOS 和 USB CDC 的实现示例。
- 详细集成说明请参考 `example/README.md`。
- 关键适配点包括：裁判系统结构体命名对接、通信接口切换（USB CDC vs UART）以及发送频率宏定义。

## 10. 注意事项

- ROS 端与电控端的结构体必须保持严格的字节对齐，协议变更时两端必须同步更新。
- CRC16 校验算法两端必须一致（采用查表法，初始值为 0xFFFF）。
- `cmd_vel` 订阅的是绝对路径话题，接收的是经过 `fake_vel_transform` 转换后的最终速度（body 系并叠加了自旋速度）。
- 驱动具备自动重连机制，串口断开后会以 1s 为间隔尝试重新打开设备。
- 本包特有的 CMake 配置将 C++ 标准设为 C++14（项目其他包通常使用 C++17）。
- `package.xml` 中保留了部分历史遗留的未使用依赖（如 `auto_nav_interfaces`, `visualization_msgs` 等）。

## 11. 速度日志

用于调试速度毛刺、突变等问题。启用后在串口发送前记录每帧 `vel_x`, `vel_y`, `vel_w` 到 CSV 文件，便于离线波形分析。

### 启用方式

在 `nav2_params.yaml` 中：

```yaml
rm_serial_driver:
  ros__parameters:
    enable_vel_log: true
```

或 launch 参数覆盖：

```bash
ros2 launch rm_serial_driver serial_driver.launch.py enable_vel_log:=true
```

### 日志位置

`/tmp/vel_log_<启动时间戳>.csv`，节点启动时打印完整路径。

### CSV 格式

```csv
timestamp_ns,vel_x,vel_y,vel_w
1712345678000000000,1.23,-0.45,3.14
```

- `timestamp_ns`：ROS 时间（纳秒）
- `vel_x`, `vel_y`：body 系线速度（m/s），经 fake_vel_transform 旋转 + spin_speed 叠加后的最终值
- `vel_w`：角速度（rad/s），含 spin_speed

### 离线分析

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('/tmp/vel_log_xxx.csv')
df['t'] = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) / 1e9

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 6))
for i, col in enumerate(['vel_x', 'vel_y', 'vel_w']):
    axes[i].plot(df['t'], df[col], linewidth=0.5)
    axes[i].set_ylabel(col)
    axes[i].grid(True, alpha=0.3)
axes[2].set_xlabel('time (s)')
plt.tight_layout()
plt.savefig('vel_debug.png', dpi=150)
plt.show()
```

### 性能影响

默认关闭（`enable_vel_log: false`），零开销。启用后 50Hz 写入约 2KB/s，对实时性无影响。调试完毕后建议关闭。

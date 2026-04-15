# 电控端串口通信示例

## 文件说明

| 文件 | 说明 |
|---|---|
| `navigation_auto.h` | 协议定义：收发包结构体、帧头、发送频率宏 |
| `navigation_auto.c` | 收发实现：分包发送、CRC 校验、接收解析 |

## 协议概览

### 电控 → ROS（3 个包，按频率分离）

| 帧头 | 包名 | 大小 | 默认频率 | 内容 |
|---|---|---|---|---|
| `0xA1` | IMU | 11B | ~1000Hz | pitch, yaw |
| `0xA2` | Status | 12B | ~10Hz | 比赛阶段、剩余时间、自身血量、弹量、颜色、RFID |
| `0xA3` | HP | 17B | ~2Hz | 己方 7 台机器人/建筑血量 |

### ROS → 电控（1 个包）

| 帧头 | 包名 | 大小 | 频率 | 内容 |
|---|---|---|---|---|
| `0xB5` | Nav | 15B | 跟随 cmd_vel | vel_x, vel_y, vel_w |

所有包格式：`[header 1B] [payload] [crc16 2B]`

## 使用方法

### 1. 复制文件

将 `navigation_auto.h` 和 `navigation_auto.c` 复制到 STM32 工程中。

### 2. 创建 FreeRTOS 任务

```c
#include "navigation_auto.h"

void NavigationTask(void const *argument)
{
    Navigation_Init();

    for (;;) {
        Navigation_Task();
        osDelay(1);  // 1ms -> ~1000Hz
    }
}
```

`Navigation_Task()` 内部自动完成：
- 每次调用：检查是否收到 ROS 速度指令
- 按 `SEND_*_INTERVAL` 宏控制的频率分包发送

### 4. 读取 ROS 下发的速度

```c
float vx = navigation.lx;       // x 线速度 (m/s)
float vy = navigation.ly;       // y 线速度 (m/s)
float w  = navigation.vel_w;    // 角速度 (rad/s)
```

## 需要适配的地方

### 裁判系统变量名

`navigation_auto.c` 中引用了以下裁判系统结构体，需与你的工程保持一致：

| 代码中的引用 | 用途 |
|---|---|
| `INS.Pitch` / `INS.Yaw` | IMU 姿态 |
| `Robot_Status.robot_id` | 判断红蓝方 |
| `Robot_Status.current_HP` | 自身血量 |
| `Game_Status.game_progress` | 比赛阶段 |
| `Game_Status.stage_remain_time` | 阶段剩余时间 |
| `Game_Robot_HP.ally_*` | 己方全队血量 |
| `Projectile_Allowance.projectile_allowance_17mm` | 弹量 |
| `RFID_Status.rfid_status` | RFID 增益点 |

字段名不一致时，修改 `.c` 中对应赋值即可。

### 发送频率

默认假设任务以 1ms 周期运行（~1000Hz）。如果 `osDelay` 不同，调整 `.h` 中的宏：

```c
#define SEND_IMU_INTERVAL       1    // 每 N 个 tick 发一次
#define SEND_STATUS_INTERVAL  100
#define SEND_HP_INTERVAL      500
```

### 通信接口

当前使用 USB CDC（`CDC_Transmit_FS` / `CDC_Receive_FS`）。如使用 UART 硬件串口，替换 `.c` 中的收发调用。
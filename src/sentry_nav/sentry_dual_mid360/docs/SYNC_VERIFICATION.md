# Livox Mid360 双雷达硬件同步验证指南

> **Maintainer**: Boombroke <boombroke@icloud.com>
> **Scope**: 验证 `front_mid360` 与 `back_mid360` 两台 Livox Mid360 的 header.stamp 是否源自同一个硬件时钟源。
> **Why it matters**: Point-LIO 以 `msg->header.stamp` 作为权威时间。pointcloud_merger 的 ApproximateTime 同步器默认 `slop=10ms`；若两路时间戳由各自设备的独立时钟各自分配，两台 Mid360 的 header.stamp 会线性漂移，数小时后超出 slop，匹配率断崖式下降，融合结果退化为单雷达。
> **禁止**: 不要假设"已经拉了黄线主从"就等于硬件同步，必须实测。若硬件未同步，**必须修硬件**（PTP / PPS+GPRMC），**严禁**在 merger / Point-LIO 里做软件补偿。

---

## 1. Livox 官方支持的三种同步模式

Livox Mid360 数据手册和官方 Wiki 只承认下列三种时间同步源（写作本文时以官方文档为准）：

| 模式 | 信号源 | 接线 | 精度量级 | 典型场景 |
|------|--------|------|----------|----------|
| **PTP (IEEE 1588v2)** | 以太网 master clock（grandmaster） | 以太网一根到交换机或主机 | 亚微秒 | 多传感器工控网络 |
| **gPTP (IEEE 802.1AS)** | 以太网 master clock（AVB/TSN 子集） | 同上，交换机要求支持 gPTP | 亚微秒 | 车载以太网 |
| **GPS 模式（PPS + GPRMC UART）** | GNSS 接收机 | PPS 物理线（硬边沿触发）+ GPRMC NMEA 文本（授时） | 小于 ~1 微秒的 edge，加 UART 行率语义秒级 | 户外 / GNSS 可用环境 |

**参考**：
- Mid-360 time-sync methods: <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html#supported-time-synchronization-methods>
- Livox 通用时间同步说明: <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html>

> **官方列表之外没有第四种。** "黄线主从" 这种叫法在 Livox 官方文档中**不存在**，只是项目内部口口相传的别名。它**可能**对应 GPS 模式里的 PPS 物理触发线、**也可能**只是一根 Livox 同步线分接（该线拓扑不构成硬件同步）。本文件存在的意义就是把这个含糊带过的事情实测清楚。

### 1.1 为什么不能跳过验证

- Point-LIO 读 `msg->header.stamp`，不读 `timebase`，不读设备厂商字段。
- merger 的 ApproximateTime 默认 `slop=10ms`，一旦实际漂移超过 slop，两路消息就不会同时进入 callback，fused 点云越来越空。
- 双雷达各自用自己的内置 RTC 时，温度、晶振老化会让两块 RTC 按 ppm 级别漂移，60 秒稳定的系统一两小时就会拉开几十毫秒差距。

---

## 2. 决策流程（总览）

```
┌───────────────────────────────┐
│ 1. 查询 Mid360 当前 time sync │
│    source（SDK2 / Viewer）    │
└──────────────┬────────────────┘
               │
     官方列出的 3 种模式之一?
               │
        ┌──────┴──────┐
        │             │
       YES           NO / 未知
        │             │
┌───────▼────────┐   ┌▼──────────────────────────┐
│ 2. 检查物理接线│   │ 决策：当前是软同步或未配置│
│   (PTP | PPS)  │   │ → 回到 §5 / §6 配置硬同步 │
└───────┬────────┘   └───────────────────────────┘
        │
┌───────▼─────────────────────────┐
│ 3. 跑 verify_dual_mid360_sync   │
│    --expected-result 匹配实测   │
└───────┬─────────────────────────┘
        │
  ┌─────┴─────┬───────────────┐
  │           │               │
HARDWARE_    DRIFTING        NOT_SYNCED
  SYNC        │               │
  │           │               │
  ▼           ▼               ▼
进入 T16    修硬件/修       大问题，彻查
台架        PTP 链          线缆/驱动版本
```

严格按照这个流程走。不要用"帧率正常"判断同步，**帧率正常 ≠ 同步对齐**。

---

## 3. 手动验证四步法

### 3.1 Step 1：读 Mid360 当前的 time sync source

**SDK2 CLI（推荐）**

Livox SDK2 发布后，`livox_ros_driver2` 启动日志会打印每个 LiDAR 的同步源。观察启动日志：

```bash
ros2 launch sentry_dual_mid360 dual_mid360_launch.py 2>&1 | tee /tmp/driver_boot.log

# 关键字段（SDK2 日志用语以驱动实际输出为准）：
grep -E "time.?sync|ptp|gPTP|gprmc|pps|sync.?source" /tmp/driver_boot.log
```

**Livox Viewer（图形界面）**

- 在 Windows / Ubuntu 桌面连接 Mid360，切换到 "Device" 面板。
- "Time Sync" 字段应明确列出 `PTP`、`gPTP` 或 `GPS`。若显示 `None` / `Internal`，则雷达没有外部时钟源，两块雷达用各自 RTC 独立走时。
- 两台 Mid360 都单独检查一遍，**两个都必须是同一种且非 None 的模式**，否则就是 `NOT_SYNCED`。

**物理现场快速判断（只能辅助）**

- 雷达 RJ45 + 同步专用线（Livox 文档叫 "PPS/GPRMC cable" 或 "time sync cable"）同时接到主机。
- 只有网线 + 一条短 Livox 专用线从 front 接到 back，不代表硬件同步。那条线**可能**只是 Livox 自定义协议线，拓扑上不是 grandmaster→slave 的 PTP 关系。必须落回 SDK2 / Viewer 读状态。

### 3.2 Step 2：检查物理接线

| 模式 | 硬件检查清单 |
|------|-------------|
| **PTP / gPTP** | 主机或交换机需支持 PTP；Linux 侧 `ptp4l -i <iface> -m` 能看到报文；不需要额外的硬件线。 |
| **GPS（PPS + GPRMC）** | 接收机提供 PPS 方波（通常 TTL 3.3V / 5V）与 NMEA GPRMC 语句；参考 Livox Mid360 数据手册的同步接口定义，按手册给出的引脚分配接线；不要凭记忆硬写引脚号，出厂线缆在两端的印字 / 数据手册是唯一真相源。 |
| **未配置** | 只有网线，或只有一条非官方"桥接线"；读 Viewer 为 None。 |

> **硬件路径由机械/电控同学落实**。本仓库范围内不配置 PTP 服务、不接 GPS，只验证。

### 3.3 Step 3：比对 header.stamp

拿到两路 CustomMsg 后，观察相邻帧 stamp 差：

```bash
ros2 topic echo --once /livox/lidar_front --no-arr | grep -E "sec:|nanosec:"
ros2 topic echo --once /livox/lidar_back  --no-arr | grep -E "sec:|nanosec:"
```

人工比较两个 `sec.nanosec`，差值应 < 1ms（硬件同步）。一次采样不具备统计意义，进入 Step 4。

### 3.4 Step 4：跑 60 秒漂移统计

```bash
ros2 run sentry_dual_mid360 verify_dual_mid360_sync.py \
    --sample-count 100 \
    --sync-tolerance-ms 1.0 \
    --front-topic /livox/lidar_front \
    --back-topic  /livox/lidar_back
```

脚本在 §4 详述。核心判据：中位数差 < 1ms 且 60s 内漂移 < 0.5ms 才能认定是真硬件同步；任一项超标，说明两雷达在各自走各自的时钟。

---

## 4. `verify_dual_mid360_sync.py` 脚本使用说明

### 4.1 功能

- 订阅前后两路 `livox_ros_driver2/msg/CustomMsg`。
- 用 `message_filters.ApproximateTimeSynchronizer` 对齐，记录约 100 对（可配）相邻帧的 `header.stamp` 差。
- 输出统计量：median / max / stddev / drift slope / drift range，并按阈值给出结论 `HARDWARE_SYNC` / `DRIFTING` / `NOT_SYNCED`。
- 支持 `--mock-data` 模式离线回归（CI / 本地无雷达环境）。

### 4.2 常用命令

```bash
# 打印帮助
python3 src/sentry_nav/sentry_dual_mid360/scripts/verify_dual_mid360_sync.py --help

# 无 ROS 环境：mock 数据跑一遍，期望得到 HARDWARE_SYNC
python3 src/sentry_nav/sentry_dual_mid360/scripts/verify_dual_mid360_sync.py \
    --mock-data --expected-result HARDWARE_SYNC

# 实车：接到真实话题跑 100 对样本
source install/setup.bash
ros2 run sentry_dual_mid360 verify_dual_mid360_sync.py \
    --front-topic /livox/lidar_front --back-topic /livox/lidar_back \
    --sample-count 100
```

### 4.3 返回码语义

| exit code | 含义 |
|-----------|------|
| `0` | 结论符合 `--expected-result`（或未指定 expected 时结论为 `HARDWARE_SYNC`） |
| `1` | 结论与 `--expected-result` 不符 |
| `2` | 采样不足 / 话题未到齐 / 运行异常 |

---

## 5. 结果分支：HARDWARE_SYNC / DRIFTING / NOT_SYNCED

### 5.1 HARDWARE_SYNC（通过）

**判据**（全部满足）：

- `median_diff_ms < 1.0`
- `max_diff_ms   < 3.0`
- `stddev_ms     < 0.5`
- 60 秒 drift slope 绝对值 < 0.5 ms/min 且 drift range < 0.5 ms

**含义**：两雷达共享同一硬件时钟源，header.stamp 抖动基本来自 USB/以太网栈与 ROS 时间戳派发。

**下一步**：

- 允许进入 T16 实车静态台架测试。
- 可把 merger 的 `sync_tolerance_ms` 维持在默认 `10.0`（已大幅留有余量）。

### 5.2 DRIFTING（不通过，但有救）

**判据**（任一满足，且 `median_diff` 起始时仍然较小）：

- 60 秒内 drift slope 绝对值 ≥ 0.5 ms/min。
- drift range ≥ 0.5 ms。
- median 起步 < 5 ms，但趋势明显向一侧偏。

**含义**：两雷达挂着各自的 RTC，同步线只是控制线或只做"软同步"，没有真正的硬件时钟共用。一开机差得不多，越跑越远。

**不要做**：

- 禁止改 merger 让它"补偿 drift"。这是在埋雷，温度/时间段都会变。
- 禁止提高 `sync_tolerance_ms` 来遮盖（只是把崩溃时间往后推）。

**要做**：

- 走 §6 的 PTP 路径或 §7 的 GNSS/PPS+GPRMC 路径，真正配置硬件同步。
- 修完以后再跑一次脚本，目标 `HARDWARE_SYNC`。

### 5.3 NOT_SYNCED（严重问题）

**判据**（任一）：

- `median_diff_ms >= 10.0`（直接超 merger slop）。
- 差值符号抖动剧烈，无明显线性趋势（雷达各自跑各自的，且有断崖式跳变）。
- 采样采不到对齐帧（ApproximateTime 全部超 slop 被丢弃）。

**含义**：两雷达几乎没有可用的时间基础，或其中一台雷达掉线。

**排查**：

1. 两台 Mid360 的 Viewer 状态是不是都显示同一种 sync source，还是一台显示 `PTP` 另一台显示 `None`？
2. 驱动版本是否一致？`livox_ros_driver2` 在 `multi_topic=1` 下要求两台雷达固件 / SDK 版本对齐。
3. 网线、PoE 供电是否正常？是否出现了 PoE 降压导致某台雷达重启走 RTC？
4. 若启用了 PTP，`ptp4l` 是否跑起来了？`pmc` 诊断的 `PORT_DATA_SET` 是否是 `MASTER` / `SLAVE`？

**修完后**务必重跑整个 §3 流程。

---

## 6. 路径 A：在 Linux 主机启用 PTP（推荐）

> **本文只给模板和流程，不在此 Task 真正配置。** 实际部署请由机械/电控同学执行，并把最终配置回填到本节。

### 6.1 安装 linuxptp

```bash
sudo apt update
sudo apt install -y linuxptp
```

`linuxptp` 提供 `ptp4l`（协议栈）、`phc2sys`（PHC→系统时钟同步）、`pmc`（诊断）。官方文档：<https://linuxptp.sourceforge.net/>。

### 6.2 ptp4l 配置模板

`/etc/linuxptp/ptp4l-mid360.conf`（示例，实际接口名替换）：

```ini
[global]
# 典型 Mid360 工控环境：以太网链路 + 硬件打戳
priority1               128
priority2               128
domainNumber            0
tx_timestamp_timeout    50
logAnnounceInterval     1
logSyncInterval         0
logMinDelayReqInterval  0
network_transport       UDPv4
clock_type              OC

[eth0]
# 仅在实际网卡上启用；如果是多网卡请每块网卡一段
```

### 6.3 systemd 单元模板

`/etc/systemd/system/ptp4l-mid360.service`：

```ini
[Unit]
Description=PTP time sync for dual Mid360
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l-mid360.conf -m
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

配套 `phc2sys`（让系统时钟跟随 PHC）：

```ini
[Service]
ExecStart=/usr/sbin/phc2sys -a -r -m
```

### 6.4 验证

```bash
sudo systemctl enable --now ptp4l-mid360.service
sudo systemctl status ptp4l-mid360.service
pmc -u -b 0 'GET CURRENT_DATA_SET'

# 观察雷达侧：Viewer / SDK2 日志显示 time_sync_source = PTP
```

配置完成后回到 §3 重跑验证脚本，目标 `HARDWARE_SYNC`。

---

## 7. 路径 B：GNSS + PPS + GPRMC 授时

在户外或需要绝对时间源的场景，Mid360 可通过 GPS 模式（PPS 硬边沿 + GPRMC 文本）接收 GNSS 接收机授时。

### 7.1 接线清单

- **GNSS 接收机**：输出 PPS（TTL 3.3V 或 5V，上升沿触发）和 GPRMC NMEA 文本。
- **PPS 信号线**：接到 Mid360 同步接口中数据手册指定的 PPS 引脚；**具体引脚编号以 Mid360 数据手册为准**，不要在这里硬写死。
- **GPRMC UART**：由接收机的 TX 接到 Mid360 同步接口的 RX 引脚，波特率以 Livox Wiki GPS 同步章节规定为准（常见 9600 / 115200，以文档为准）。
- **共地**：GNSS 接收机与 Mid360 必须共地，否则 PPS 边沿跳变不干净。
- **GNSS 有星历**：室内无信号时退化为无 PPS，雷达会自动回到内部时钟，再次陷入 `NOT_SYNCED`。

### 7.2 典型验证

- Mid360 Viewer 的 "Time Sync" 应显示 `GPS`。
- PPS 上升沿在示波器上抖动 ≤ 1μs。
- GPRMC 每秒一帧，秒级时间来自 GNSS。
- 两台 Mid360 均配相同 GNSS 源（通常一个接收机同时驱动两台，或两台 Mid360 各接同步信号分线）。

### 7.3 常见坑

- 某些车载 GNSS 模块的 PPS 是 1Hz 占空比 50% 的方波，不是窄脉冲。Mid360 对上升沿触发通常没问题，但仍需对照数据手册确认。
- 波特率/数据位/校验配置错误会表现为 "PPS 能收但 GPRMC 无效"，日志里会看到秒级时间不更新，但 header.stamp 仍在跳。这种情况下同步不完整，仍需修正。

配完以后回到 §3 重跑验证脚本，目标 `HARDWARE_SYNC`。

---

## 8. 什么绝对不能做

- **禁止在 pointcloud_merger 里做"软件时间补偿"**：不要引入"检测到 drift 就加/减常量"这种逻辑，它在温度变化和长时间运行时会失效。
- **禁止修改 Point-LIO 源码**（见 plan MN1、MN6）。不要把 `timebase` 当时间基础。
- **禁止扩大 `sync_tolerance_ms`** 来掩盖问题（slop 是安全裕度，不是漂移补偿器）。
- **禁止** 把 `line` 字段偏移到 4-7（MN5）；即便为了绕开同步问题，这条红线仍然存在。
- **禁止** 用 `ExactTime` 同步策略：两台雷达即使硬件同步，ns 精度也不保证完全相等。
- **禁止** 在本仓库里起 `ptp4l` 服务、改网络配置、动 `/etc/systemd`，那属于部署作业，本 Task 只提供配置模板。

---

## 9. 参考清单

- Livox Mid-360 官方 Wiki: <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html>
- Livox Time Sync 通用章节: <https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html>
- linuxptp 项目: <https://linuxptp.sourceforge.net/>
- 项目规划文件: `.sisyphus/plans/dual-mid360-fusion.md` §T7
- 本仓库 MEMORY/AGENTS.md 中 Point-LIO `header.stamp` 单调性约束（Metis G2/G3）

## 10. 变更记录

- 2026-05-06：初版。覆盖官方三种同步模式、四步验证流程、三分支判据、PTP 与 GNSS 两条修复路径、绝对禁止事项。

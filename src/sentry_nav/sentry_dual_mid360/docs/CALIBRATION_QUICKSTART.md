# 双 Mid360 标定 · 新手速通

> **看这一篇就够了**：从雷达插电到 xmacro 里装上测量值，最短路径。
> 更深的原理、参数含义、三种同步模式、PTP/GNSS 配接等，看
> `SYNC_VERIFICATION.md` 和 `ARCHITECTURE.md`。

## 这套流程是做什么的

把两台 Mid360（front / back）的**相对安装位姿**（6-DoF：x y z roll pitch yaw）
测准，结果写进：

```
src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro
                                                └── secondary_lidar_pose="..."
```

系统启动后 Point-LIO、Nav2、pointcloud_merger 都从这个 xmacro 拿 TF。
标定不准 ⇒ 两路点云叠不上 ⇒ 定位漂移 / 感知错位。

## 前置条件（只需一次）

| 硬件 | 要求 |
|---|---|
| 两台 Mid360 | Front IP `192.168.1.145`，Back IP `192.168.1.144` |
| 主机网卡 | 静态 IP `192.168.1.100/24`，ping 两台雷达都要通 |
| 场地 | 雷达稳定静止，**周围有墙角/立柱/箱体**（纯空地标不出来） |

| 软件 | 装法 |
|---|---|
| ROS 2 Jazzy | `source /opt/ros/jazzy/setup.bash` |
| 本仓库 | `colcon build --symlink-install`，之后 `source install/setup.bash` |
| 标定依赖 | 见下方「依赖自检」一节，脚本会列清单 |

## 一次完整标定（5 步）

### 1. 起驱动

```bash
ros2 launch sentry_dual_mid360 dual_mid360_driver_launch.py
```

留着这个终端，**不要 Ctrl-C**。起成功后 `/livox/lidar_primary`、
`/livox/lidar_secondary` 两个话题都在发 10 Hz 的 CustomMsg。

> 注意：这个 launch **只起雷达驱动**，不起 nav2/slam/merger。全实车系统用
> `rm_navigation_reality_launch.py`，但那个路径在录标定 bag 时会跟下游
> 订阅者打架，不推荐。

### 2. 依赖自检

```bash
bash src/sentry_nav/sentry_dual_mid360/scripts/calib/calibrate_dual_mid360.sh --check-deps
```

输出 `All prerequisites satisfied` 才继续。缺什么它会一条条列出来，
按提示装。常见缺的：

- `teaserpp_python` / `open3d` / `pandas` / `ros2_numpy`：Python 包
- `multi_lidar_calibrator`：Multi_LiCa ROS 包，看脚本提示的 remove/build/restore
  工作流
- TEASER-plusplus `.so`：要从 `src/third_party/Multi_LiCa/TEASER-plusplus` build

### 3. 验证时间同步（可选但强烈推荐）

```bash
python3 src/sentry_nav/sentry_dual_mid360/scripts/calib/verify_dual_mid360_sync.py \
    --front-topic /livox/lidar_primary \
    --back-topic  /livox/lidar_secondary \
    --sample-count 600 --timeout-s 120
```

看最后一行 `verdict`：

- `HARDWARE_SYNC` ✅ 直接进下一步
- `DRIFTING` ⚠️ 如果只是 `stddev` 或 `|slope|` 轻微超阈，而 median 和
  monotonic drift 都在 1 ms 以内，可以继续标定（空间配准对 ms 级抖动不敏感）
- `NOT_SYNCED` ❌ 中位数差 ≥ 10ms，两台雷达时钟完全不一致，**先修硬件再做任何事**。
  看 `SYNC_VERIFICATION.md` §6/§7

### 4. 录一包 60 秒的静态数据

**让雷达保持 60 秒不动**，执行：

```bash
bash src/sentry_nav/sentry_dual_mid360/scripts/calib/record_calib_bag.sh \
    --env real --duration 60
```

脚本会：
- 自检 `/livox/lidar_primary` 和 `/livox/lidar_secondary` 是否在发（缺一个就直接
  abort，不会录空包）
- 自动跳过当前没人发的 `/tf`、`/tf_static`（仅启驱动模式下这俩不会有，
  Multi_LiCa 也用不上）
- 把 bag 写到 `logs/evidence/calib-bags/real-<时间戳>/` 下，同目录还有
  一份 `metadata.yaml` 记录录包参数和实际录到的 topic 列表

结束后确认：

```bash
LATEST=$(ls -td logs/evidence/calib-bags/real-* | head -1)
ros2 bag info "$LATEST"/*.bag | grep -E 'Duration|Messages|Count'
```

Duration 应该 ~60s，两个 lidar topic 的 Count 都应该 ≈ 600。**如果有
一个为 0，回到第 1 步看驱动日志**。

### 5. 跑标定

先拿到刚才录的 bag 路径（`--bag` 必须指到 `*.bag` 这一级，不是外层运行目录）：

```bash
LATEST=$(ls -td logs/evidence/calib-bags/real-* | head -1)
BAG="$LATEST"/$(basename "$LATEST").bag
```

**首次标定 / xmacro 里 CAD 值还不准**（绝大多数第一次做的场景）：

```bash
bash src/sentry_nav/sentry_dual_mid360/scripts/calib/calibrate_dual_mid360.sh \
    --bag "$BAG" \
    --output-report logs/evidence/calib-report.md \
    --bootstrap --write-xmacro
```

`--bootstrap` 告诉脚本：当前 xmacro 里的 `primary_lidar_pose` /
`secondary_lidar_pose` 是占位估计，别拿它们跟测量结果比差值。只要
Multi_LiCa 自己的点云配准质量够（`fitness ≥ 0.99`、`RMSE ≤ 10cm`），
就判 PASS 并回写。

---

**日常微调 / xmacro 已经是上次标定过的好值**：去掉 `--bootstrap`：

```bash
bash src/sentry_nav/sentry_dual_mid360/scripts/calib/calibrate_dual_mid360.sh \
    --bag "$BAG" \
    --output-report logs/evidence/calib-report.md \
    --write-xmacro
```

严格模式会要求新测量值相对 xmacro 现有值变化 < 2cm、< 0.5°，
超过就 FAIL，不会改 xmacro。用于"雷达拆装后复检"这类场景。

## 看结果

标定结束后：

```bash
cat logs/evidence/calib-report.md                  # 人看的总结
cat logs/evidence/task-11-precision-check.txt     # 机读的判据明细
cat logs/evidence/task-11-xmacro-update.txt        # 改了什么 / 为什么没改
grep secondary_lidar_pose src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro
```

判决码：

| 退出码 | 含义 | 怎么办 |
|---|---|---|
| 0 | PASS，xmacro 已改（如果传了 `--write-xmacro`） | 重新 `colcon build`，跑实车验证 |
| 1 | FAIL，指标不达标，xmacro 没改 | 看 `precision-check.txt` 的 fail_reasons |
| 2 | BLOCKED，前置条件不满足 | 看 `task-11-blocker.md`，按提示补 |

## 结果回写后怎么生效

xmacro 是 sdf 生成源。改完要：

```bash
colcon build --symlink-install --packages-select sentry_robot_description sentry_dual_mid360
source install/setup.bash
```

下次起实车 launch 就会用新的 `secondary_lidar_pose`。

## 想先在 3D 里看看 xmacro 是否合理

标定回写 xmacro（或者机械手动改 `primary_lidar_pose` / `secondary_lidar_pose`）之后，
可以不起实车就在 Gazebo 里预览一下传感器挂载是否合理：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash src/sentry_nav/sentry_dual_mid360/scripts/tools/preview_real_xmacro.sh
```

脚本做三件事：`xmacro4sdf` 展开 `wheeled_biped_real.sdf.xmacro` → 组一个空世界 →
`gz sim` 起一个纯可视化的 Gazebo（没有 nav2/slam/Point-LIO）。关窗口即退出。
加 `--headless` 只校验 SDF 能否加载，加 `--keep-sdf` 保留展开结果便于和上一次
做 diff。

## 下位机没接 / 只想在 rviz 里验证融合点云

标定 PASS 之后想肉眼看"墙是薄一层、立柱是一根"，但实车下位机 / 串口没接、
不想起整套 `rm_sentry_launch.py`（会卡在等 serial）：

```bash
bash src/sentry_nav/sentry_dual_mid360/scripts/tools/lidar_only_debug.sh
```

（`merger` 和 `rviz` 默认会一起起；无屏 / ssh 环境加 `--no-rviz` 跳过。）

为什么不直接看 `/livox/lidar_primary` / `_back` / `/livox/lidar`：它们是
`livox_ros_driver2/CustomMsg`，rviz 不能 render（Add 列表里灰色不能选）。
脚本起的 `pointcloud_merger` 会同时发 `/livox/lidar_pc2`
（`sensor_msgs/PointCloud2` 镜像），rviz 就能直接看。

一个终端内包办：livox driver + robot_state_publisher（xmacro 静态 TF） +
`map→odom` / `odom→base_footprint` identity fake TF + `pointcloud_merger`
（同时发 CustomMsg 和 PC2 副本）+ `rviz2`。Ctrl-C 一次把所有子进程一起收尾。

rviz 里：
- **Fixed Frame**: 手打 `primary_mid360`（直接编辑 Global Options 第一行）
- `Add → PointCloud2 → /livox/lidar_pc2`（**merger 输出的 PC2 镜像**）
- **必须改**：该 PointCloud2 面板展开 `Topic → Reliability Policy → Best Effort`
  （默认 Reliable 和 merger 的 Best Effort publisher QoS 不兼容，会 silently
  drop 所有消息，看不到点云）
- Size 调 3、Color Transformer 选 `Intensity` 或 `AxisColor`、Decay Time `0`
- 看墙是薄一层、立柱是一根、地面单平面 → 标定 OK
- 双层墙 / 错位柱 / V 字形 → 回标定查 xmacro

如果要看 Point-LIO 的 `/cloud_registered` 或做整车级验证（回环、定位
精度、重定位等），直接走完整实车 bringup：
```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py
```
它带 Point-LIO + SLAM + Nav2 + odom_bridge，有整车 IMU 稳定条件 ESKF
能收敛。桌面摆雷达模式看 `/livox/lidar_pc2` 已足够验证外参是否正确。

## 常见坑

- **bag 录完只有几 KB，Count=0**：驱动没跑，或 QoS 被下游吃了。确认第 1 步的
  终端还活着，用 `ros2 topic hz` 隔几秒看一眼（注意要 source install/setup.bash，
  不然 CustomMsg 类型支持缺失会报"message type invalid"）
- **`verdict: FAIL`，`rotation_error_deg` 接近 180°**：90% 是 xmacro 里
  `primary_lidar_pose` 的 yaw 方向跟实装不一致。检查物理朝向，或加 `--bootstrap`
  先跑通再说
- **`BLOCKED: multi_lidar_calibrator is not discoverable`**：Multi_LiCa ROS
  包没 build。按 `--check-deps` 提示的 remove COLCON_IGNORE → colcon build →
  touch COLCON_IGNORE 三步做
- **`node crashed: ModuleNotFoundError: teaserpp_python`**：TEASER++ 的 Python
  binding 装残了（只有 `__init__.py` 没有 `.so`）。先
  `pip uninstall -y teaserpp_python`，再从 `src/third_party/Multi_LiCa/TEASER-plusplus`
  源码重 build

## 相对 vs 绝对测量

脚本会同时产出两种数值：

- **相对外参** `T_secondary→primary`（`relative_secondary_in_primary_xyz_rpy_rad`）:
  纯点云测的，不依赖 xmacro 里任何值。以后机械定版了 `primary_lidar_pose`，
  拿这个相对值一次矩阵乘就能推出新 `secondary_lidar_pose`，**不用重录 bag / 重标**。
- **绝对 pose** `secondary_lidar_pose`（相对 `gimbal_pitch`）: `primary_base @ T_secondary→primary`。
  这个值绑定在当前 xmacro 的 `primary_lidar_pose` 上，front 改了它就得重算。

所以**只要 primary_lidar_pose 还没定版，你拿到的 secondary_lidar_pose 就是暂时的**——
真正稳定的是 relative 那组数字。

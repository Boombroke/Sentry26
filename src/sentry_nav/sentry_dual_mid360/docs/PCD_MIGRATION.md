# 双 Mid360 升级 PCD 迁移指南

> **Maintainer**: Boombroke <boombroke@icloud.com>
> **Scope**: 单 Mid360 → 双 Mid360 升级后，先验 PCD（prior_pcd）的重建与迁移 SOP。面向操作员。
> **定位**: 本文只讲**操作层面的 PCD 迁移流程**，不讨论双 Mid360 架构本身；架构视角见同目录 [`ARCHITECTURE.md`](ARCHITECTURE.md)，硬件同步流程见 [`SYNC_VERIFICATION.md`](SYNC_VERIFICATION.md)，Point-LIO / GICP 参数调优见仓库根 `src/docs/TUNING_GUIDE.md`。

---

## 1. 背景与适用范围

Sentry26 在 2026 赛季把定位雷达从单 Mid360 升级到**前后反向双 Mid360**。两颗 Mid360 通过 `pointcloud_merger` 外部前融合成单路 `/livox/lidar`（`livox_ros_driver2/msg/CustomMsg`，frame=`front_mid360`）喂给 Point-LIO，Point-LIO 源码零改动。

对 `small_gicp_relocalization` 而言，**运行时输入发生了语义变化**：

- 单 Mid360 时代：`/registered_scan` 只覆盖**前向半球**（约 160° 前向水平 FOV），PCD 同样只有前向点。
- 双 Mid360 升级后：`/registered_scan` 覆盖**近全周**，后向也有真实 intensity 返回。

因此：**任何在单 Mid360 时代生成的 PCD 文件，都不能在双 Mid360 运行时继续使用**。

本指南适用的场景：

- 从单 Mid360 既有 workspace 升级到当前双 Mid360 分支后，仓库里旧的 `prior_pcd.prior_pcd_map_path` 仍指向单雷达时代的 PCD。
- 已经完成 T11 双雷达外参标定、`pointlio_dual_overrides.yaml` 新鲜（`verify_pointlio_overrides_fresh.py` PASS），但还没在实车上重建 PCD。
- `/home/<user>/Documents/Sentry26_PCD/` 目录为空或仅含单雷达遗产文件。

不适用的场景：

- 纯仿真回归测试（`rm_navigation_simulation_launch.py`，使用 `src/sentry_nav_bringup/pcd/simulation/`）。仿真 PCD 是单雷达 coverage，本来就是占位资源，不在本指南迁移范围内；仿真侧双雷达桥接尚未落地（见 `ARCHITECTURE.md` §15）。
- 单雷达 downgrade 场景（`use_dual_mid360:=False`）。该路径 merger 不启动、生成的 override YAML 不加载、dual 驱动 override / per-device remap 均不注入；Livox driver 走基础 `configured_params`，Point-LIO 保持基础 YAML 的 `common.lid_topic: livox/lidar`，仍与旧单雷达 PCD 兼容；但一旦再次切回 `use_dual_mid360:=True`，本指南重新生效。

---

## 2. 为什么旧单 Mid360 PCD 不兼容

`small_gicp_relocalization` 把 `prior_pcd_file` 直接加载为 GICP target，实时 `/registered_scan` 作为 source；GICP 逐点寻找最近邻并按协方差计算 residual。当 source / target 的几何分布不再互为子集时：

1. **后向点云全部变成 outlier**。双 Mid360 运行时 `/registered_scan` 里后向点数往往占 30–50%；若 target PCD 只有前向点，后向点会被 KD-tree 匹配到**几何上任意远处的前向点**上。
2. **inlier_ratio 劣化**。AGENTS.md §9 GICP 质量门控要求 `inlier_ratio > 0.5`；旧 PCD 下后向点被误匹配后 inlier 估计会被拖低，触发 `small_gicp_relocalization` 的质量拒绝路径，表现为 "no valid correction" 日志。
3. **fitness_error 上升并震荡**。误匹配点的协方差残差不稳定，`fitness_error` 从 0.05 量级飘到 0.5 以上；即使通过质量门控，修正量也会在正确解附近反复横跳。
4. **极端情况下触发 Emergency 误收敛**。连续失败达到 `emergency_consecutive_failures` 后会触发 Emergency 多初值搜索；若此时 PCD 与 scan 存在系统性前向偏置，Emergency 的分数阈值可能把一个**错误的刚体变换**误判成合法收敛，导致 `map→odom` 被拉偏数米。

结论：**旧 PCD 无法自动转换、无法通过数据清洗修补、也无法通过调松 GICP 阈值"救活"**。`GICP target 必须与运行时 scan 的几何分布一致`这一条是算法性约束，不是参数问题。唯一正确做法是重新建图。

另一个硬约束来自坐标系语义。按 AGENTS.md §9，`prior_pcd_file` 必须与 `/registered_scan` 一致处于 odom/map 系。Point-LIO 的 `pcd_save` 原始输出在 `lidar_odom` 系，需要退出时依赖 odom_bridge 的 latched `/odom_to_lidar_odom` 转换成 odom/map 系。**早于该 latched 话题落地的单雷达 PCD 连坐标系都不对**，更不能继续使用。

---

## 3. 迁移前检查清单

在进入实车重建前必须先过一遍下列清单；任何一条没过就不要启动慢速建图。

### 3.1 识别所有"旧 PCD 使用点"

```bash
cd /path/to/Sentry26
grep -RIn --include='*.yaml' --include='*.py' -E "prior_pcd|\.pcd" \
    src/sentry_nav_bringup src/third_party/small_gicp_relocalization
ls -la ~/Documents/Sentry26_PCD/ 2>/dev/null || echo "MISSING: Sentry26_PCD dir"
ls -la src/sentry_nav_bringup/pcd/reality/ 2>/dev/null
```

需要重点确认的字段：

- `src/sentry_nav_bringup/config/reality/nav2_params.yaml` 里 `small_gicp_relocalization` 节点的 `prior_pcd.prior_pcd_map_path`（以及 launch 层通过 `prior_pcd_file` 参数注入的值）。
- `rm_navigation_reality_launch.py` / `rm_sentry_launch.py` 里的 `prior_pcd_file` 默认推导规则（通常是 `pcd/reality/{world}.pcd`）。

**明确禁止复用的文件**（MH11）：

- `src/sentry_nav_bringup/pcd/simulation/*.pcd`：仿真占位，不是实车数据，也不是双雷达数据。
- `src/third_party/point_lio/PCD/scans.pcd`：上游 Point-LIO 样例，不是 Sentry26 地图。
- 任何在单 Mid360 时代（即双 Mid360 merger / codegen 链路落地前）由本仓库生成的 `~/Documents/Sentry26_PCD/*.pcd`。

### 3.2 识别"是否正在错误使用旧 PCD"

升级分支拉下来后，即使你已经在跑双 Mid360，只要 `prior_pcd_file` 还指向旧路径，运行时就处于错误状态。检测信号：

```bash
# 信号 1：GICP 持续质量拒绝
LOG=$(ls -t ~/.ros/log/latest/small_gicp_relocalization*.log 2>/dev/null | head -1)
grep -E "inlier_ratio|fitness_error|no valid correction|Emergency" "$LOG" | tail -40

# 信号 2：map→odom 持续漂移或瞬跳
python3 src/sentry_nav/sentry_dual_mid360/scripts/analyze_map_odom_stability.py \
    --duration 60 --jitter-threshold-cm 10

# 信号 3：Point-LIO 外参过期
python3 src/sentry_nav/sentry_dual_mid360/scripts/verify_pointlio_overrides_fresh.py
```

如果信号 1 频繁出现 `inlier_ratio < 0.5` 或 Emergency 事件，并且信号 3 是 PASS（override 新鲜），基本确认 PCD 是根因。

### 3.3 前置任务门控

- **T11 外参 accepted**：`calibrate_dual_mid360.sh --check-deps` PASS，且 `wheeled_biped_real.sdf.xmacro` 的 `back_lidar_pose` 已按 Multi_LiCa 结果写回（或确认 CAD 默认可用，MH8 规则）。
- **Override 新鲜**：`verify_pointlio_overrides_fresh.py` 必须 PASS；否则 Point-LIO 的 `extrinsic_T/R` 和 xmacro 不同步，建图出来的 PCD 带着错误外参，重建也没用。
- **Merger 可用**：`colcon build --packages-select sentry_dual_mid360 --symlink-install`；`ros2 launch sentry_dual_mid360 pointcloud_merger_launch.py --print` 能成功解析。
- **实车硬件**：双 Mid360 分别 ping 通 192.168.1.144 / 192.168.1.145；BMI088 IMU 在线；串口、轮足驱动正常。

---

## 4. 旧资产备份流程

重建前**必须先备份**，理由有两条：

1. 若新 PCD 建失败（撞墙、丢帧、退出顺序错误），回滚到已知状态好过陷入"新 PCD 坏掉 + 旧 PCD 已删"的两无状态。
2. 旧 PCD + 旧日志仍是有效的历史证据，未来追查漂移或做算法 A/B 可能需要。

### 4.1 备份 PCD 与地图

```bash
STAMP=$(date +%Y%m%d-%H%M%S)
BACKUP_ROOT="${HOME}/Documents/Sentry26_PCD_backup/${STAMP}"
mkdir -p "${BACKUP_ROOT}"

# 备份本地 PCD 目录（若存在）
if [ -d "${HOME}/Documents/Sentry26_PCD" ]; then
    cp -a "${HOME}/Documents/Sentry26_PCD" "${BACKUP_ROOT}/Sentry26_PCD"
fi

# 备份 2D map（若存在）
if [ -d "${HOME}/Documents/Sentry26_maps" ]; then
    cp -a "${HOME}/Documents/Sentry26_maps" "${BACKUP_ROOT}/Sentry26_maps"
fi

# 仓库 tracked 的 reality PCD（通常只有占位符）
if compgen -G "src/sentry_nav_bringup/pcd/reality/*.pcd" > /dev/null; then
    mkdir -p "${BACKUP_ROOT}/repo_reality_pcd"
    cp -a src/sentry_nav_bringup/pcd/reality/*.pcd "${BACKUP_ROOT}/repo_reality_pcd/"
fi

ls -la "${BACKUP_ROOT}"
```

### 4.2 冻结当前 YAML 引用

```bash
grep -RIn --include='*.yaml' -E "prior_pcd|\.pcd" \
    src/sentry_nav_bringup > "${BACKUP_ROOT}/prior_pcd_yaml_refs.txt"
git -C . log -1 --oneline > "${BACKUP_ROOT}/commit.txt"
```

这一步确保即使你在之后的步骤里改了 YAML，仍能回查"升级前 YAML 是怎么指的"。

### 4.3 绝对禁止事项

- **禁止把 PCD / 地图二进制 commit 进 git**（本仓库 `.gitignore` 已经覆盖常见后缀；操作员手动新增时不要破坏这个约定）。
- **禁止删除备份目录里的任何文件**，直到新 PCD 在导航模式下至少完成一次 Nav Goal smoke PASS。
- **禁止把备份 PCD 直接复制成"新 PCD"**绕过重建。单雷达 PCD 就是不兼容，复制一份改名也不会变兼容。

---

## 5. 实车双 Mid360 慢速建图与新 PCD 生成

以下整套流程必须**在实车现场执行**，本地 CI / 开发机不具备双 Mid360 硬件与 live 导航栈运行态（见 `.sisyphus/evidence/task-21-blocker.md`）。本指南不伪造可以在开发机完成的建图过程。

### 5.1 启动 SLAM 栈

终端 A：

```bash
source install/setup.bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py \
    slam:=True use_dual_mid360:=True
```

终端 B（**建图前**先做一次健康检查）：

```bash
# 两路 Livox CustomMsg 都在（~10 Hz）
ros2 topic hz /livox/lidar_front
ros2 topic hz /livox/lidar_back

# merger 合并输出在（~10 Hz），frame_id=front_mid360
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --field header.frame_id | head -5

# Point-LIO 已收敛（/aft_mapped_to_init 有平滑输出）
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered

# odom_bridge 已发 odom→base_footprint + /registered_scan + latched /odom_to_lidar_odom
ros2 topic info /odom_to_lidar_odom
ros2 topic hz /registered_scan

# 2D SLAM 正在跑（slam_toolbox lifecycle active，/map 在更新）
ros2 topic hz /map
```

任何一条异常都**立即停栈**，不要开始绕场。继续建图只会污染 PCD。

### 5.2 慢速绕场地一圈

操作规则（来自 AGENTS.md §9 与 T21 blocker）：

- 线速度 **< 0.3 m/s**。快了 Point-LIO 和 merger 的 ApproximateTime 都会挤压，出 `lidar loop back` 和 `drop_sync_` 计数飙升。
- 避免撞墙；撞击会让 Point-LIO 的 IMU 短暂饱和，后续位姿跳变会被烧进 PCD。
- 避免急速原地旋转（`|wz| > 0.6 rad/s`）。
- 尽量同时覆盖地面、低矮障碍、上场平台；后向半球也要扫到（主要靠绕场路径自然完成，不要故意背对感兴趣区）。
- 绕完闭环后回到起点附近再停，给 SLAM 一次闭环机会。
- 全程大约 **5 分钟**左右，不要走半小时，数据越长漂移累积越明显。

### 5.3 保存 2D map（SLAM 仍在运行）

```bash
mkdir -p ~/Documents/Sentry26_maps
ros2 run nav2_map_server map_saver_cli \
    -f ~/Documents/Sentry26_maps/<map_name>_dual
```

得到 `<map_name>_dual.pgm` + `<map_name>_dual.yaml`。命名加 `_dual` 后缀，避免与单雷达时代的地图文件互相覆盖。

### 5.4 停栈并让 Point-LIO 落盘 PCD

**关键**：先 `Ctrl-C` **终端 A 的 SLAM 栈**，让 Point-LIO 正常退出；**不要**先 kill odom_bridge。退出顺序会决定 PCD 的坐标系是否正确：

1. Point-LIO 退出时会把内存里的全局 map 点云写到磁盘。
2. Point-LIO 原始输出在 `lidar_odom` 系；odom_bridge 通过 latched `/odom_to_lidar_odom` 提供 `odom ↔ lidar_odom` 的静态变换。
3. **只有在 odom_bridge 仍在发 `/odom_to_lidar_odom` 时关 Point-LIO**，下游 pcd 保存路径才能写出 odom/map 系的点云。

落盘路径默认是 `~/Documents/Sentry26_PCD/<timestamp>.pcd`（由 Point-LIO 配置决定，实车 nav2_params 的 `pcd_save.pcd_save_en=True` 在 slam 分支下启用）。

落盘后再关 odom_bridge / 其它节点。

---

## 6. 新 PCD 验证

### 6.1 文件存在与点数/包围盒统计

```bash
pip3 install open3d --break-system-packages  # 一次性

LATEST_PCD=$(ls -t ~/Documents/Sentry26_PCD/*.pcd | head -1)
python3 - <<PY
import open3d as o3d
pcd = o3d.io.read_point_cloud("${LATEST_PCD}")
bbox = pcd.get_axis_aligned_bounding_box()
n = len(pcd.points)
print(f"Path:       ${LATEST_PCD}")
print(f"Points:     {n}")
print(f"Bounds min: {bbox.min_bound}")
print(f"Bounds max: {bbox.max_bound}")
print(f"Extent:     {bbox.get_extent()}")
assert n > 100_000, f"point count {n} below 100k threshold"
PY
```

验收基线（来自 T21 blocker）：

- 点数 `> 100 000`（典型半场 5 分钟慢走应能到 30 万+）。
- AABB 覆盖预期场地，例如 RMUC 2026 半场 `dx ≥ 10 m`、`dy ≥ 20 m`。
- 不出现明显 NaN / Inf。

### 6.2 替换 `prior_pcd_file`

在导航模式 launch 里把 `prior_pcd_file` 指向新 PCD。两种典型做法：

**做法 A — 复制到仓库 tracked 位置**（推荐，便于团队共享且不依赖绝对 home 路径，但要确认新 PCD **不会被 git 追踪**，`.gitignore` 已覆盖 `*.pcd`）：

```bash
mkdir -p src/sentry_nav_bringup/pcd/reality
cp "${LATEST_PCD}" src/sentry_nav_bringup/pcd/reality/<map_name>_dual.pcd
```

这样 launch 层既有的 `pcd/reality/{world}.pcd` 推导规则即可命中新文件（只要把实车 launch 的 `world:=<map_name>_dual` 用起来）。

**做法 B — launch 参数显式指定**（不复制文件）：

```bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py \
    slam:=False world:=<map_name>_dual use_dual_mid360:=True \
    prior_pcd_file:=${HOME}/Documents/Sentry26_PCD/<timestamp>.pcd
```

无论哪种做法，**必须同时更新** 2D `map_yaml` 指向新 `<map_name>_dual.yaml`；单雷达时代的 `.pgm / .yaml` 不要混用。

### 6.3 重定位（GICP）闭环验证

终端 A：

```bash
source install/setup.bash
ros2 launch sentry_nav_bringup rm_sentry_launch.py \
    slam:=False world:=<map_name>_dual use_dual_mid360:=True
```

等 15–20 秒让 GICP 完成初次收敛。终端 B：

```bash
# 6.3.1 节点/话题存在性
ros2 node info /small_gicp_relocalization
ros2 topic list | grep -E "reloc|small_gicp|map_clearing|cloud_clearing"

# 6.3.2 日志层 GICP 质量（当前节点不发 /relocalization_diagnostics，T19 已记录）
LOG=$(ls -t ~/.ros/log/latest/small_gicp_relocalization*.log | head -1)
grep -E "inlier_ratio|fitness_error|per_point_error|num_inliers|Emergency" "$LOG" | tail -40

# 6.3.3 map→odom 抖动
python3 src/sentry_nav/sentry_dual_mid360/scripts/analyze_map_odom_stability.py \
    --duration 60 --jitter-threshold-cm 10
```

验收（AGENTS.md §9 + T21 Scenario 2）：

- `inlier_ratio > 0.5`
- `fitness_error < 0.1`
- 最近 30 秒内无 `Emergency relocalization failed`
- `analyze_map_odom_stability.py` PASS（jitter ≤ 门限）

### 6.4 Nav Goal smoke

用 `sentry_tools/sentry_toolbox.py` 的地图拾取器从新 2D map 上挑一个安全、空旷、内部的目标点，发送一次 Nav Goal（不要随手发一个没在 costmap 里验证过的点）：

```bash
ros2 action send_goal -f /navigate_to_pose \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'},
             pose: {position: {x: <x>, y: <y>, z: 0.0},
                    orientation: {x: 0.0, y: 0.0, z: <yaw_z>, w: <yaw_w>}}}}" \
    2>&1 | tee /tmp/task-22-nav-goal.log
```

监控 30 秒。验收：至少一次 `SUCCEEDED`，或持续 `RUNNING` 且无 `ABORTED`。

### 6.5 本地 CI/开发机运行时限制（诚实声明）

本指南中 §5 / §6.1 / §6.3 / §6.4 的所有实机步骤**在当前 worktree 开发机上未执行**，与 T21 记录一致（见 `.sisyphus/evidence/task-21-blocker.md`）：本机无双 Mid360 硬件、无 `~/Documents/Sentry26_PCD/`、无 Nav2 运行态、`open3d` 未安装。本文**不是**运行结果报告，而是**操作员在实车现场按步骤复跑的 SOP**。任何把本地开发机"通过"当成 PCD 重建 PASS 的证据都是错误的。

---

## 7. 常见问题（FAQ）

**Q1. 我能不能用 CloudCompare / PCL 手动把旧 PCD 的缺失后向补齐？**
不行。单雷达 PCD 的缺失后向没有**真实 intensity 信息**，合成数据不带 intensity 语义，GICP 协方差估计会被污染；且 IntensityVoxelLayer 的下游用途（虽然 costmap 不直接读 prior_pcd）也会在其它节点里出现误导性表现。**唯一正确做法是实车重建**。

**Q2. 单雷达 PCD 能不能裁剪到 "只保留前向部分"，给双 Mid360 用？**
不推荐，原因同上 + 坐标系风险。即便坐标系对，裁剪后的 PCD 只覆盖前向，双 Mid360 的后向实时点会找不到对应的 target 结构，inlier_ratio 仍会劣化，只不过是**另一种形式的不对称**。只有在极特殊场景（例如临时应急且确认机器人只在前向视野充分区作业）才可以作为 30 分钟内的临时 workaround，不能作为长期配置。

**Q3. 我连单雷达 PCD 也没有，Sentry26_PCD 是空的，要怎么升级？**
直接跳过第 4 节备份（没什么可备份的），从第 3 节检查清单继续，然后做第 5 节实车慢速建图。本指南的迁移前提是"旧 PCD 存在且引用于 YAML 中"；空目录场景是"首次生成"，流程从第 5 节开始即可。

**Q4. 标定（T11）还没完成，能先重建 PCD 吗？**
不能。Point-LIO 的 `extrinsic_T / extrinsic_R` 由 `pointlio_dual_overrides.yaml` 注入，而该 YAML 由 xmacro 单源 codegen；如果 `back_lidar_pose` 还是 CAD 默认而没经 Multi_LiCa 精调，后雷达点在 `front_mid360` 系里的位置会有误差。带错误外参建图，PCD 就是错的。正确顺序是 **T11 → verify override fresh → 重建 PCD**。

**Q5. 新 PCD 比旧 PCD 大很多 / 小很多，正常吗？**
双 Mid360 下后向也有点，点数通常会比单雷达多 40–80%。如果点数**比旧 PCD 还少**，说明 merger 或 Point-LIO 没在正常跑；回到第 5.1 节的健康检查，特别是 `ros2 topic hz /livox/lidar_back` 和 `/livox/lidar`。如果点数**远大于 100 万**，说明绕场太久或下采样没生效，建议把慢走时长控制在 5 分钟左右再重建。

**Q6. GICP 日志里出现零星 `Emergency relocalization`，但频率很低，可以接受吗？**
短时的 Emergency 触发（例如过门、穿过视野几乎空的大厅时）属于预期范围，只要恢复后 `inlier_ratio` 与 `fitness_error` 立刻回到正常区间就可以。**持续**的 Emergency（每分钟多于一次、或 Emergency 后没有成功收敛）说明 PCD 结构与真实场景有系统性偏差，需要重新建图。

**Q7. 我想回滚到单雷达链路临时救急，怎么办？**
启动时加 `use_dual_mid360:=False`，merger 不启动、生成的 `pointlio_dual_overrides.yaml` 不加载、dual 驱动 override 与 per-device remap 均不注入；Livox driver 走基础 `configured_params`，Point-LIO 保持基础 YAML 的 `common.lid_topic: livox/lidar`。此时旧单雷达 PCD 可以重新生效（只要 `prior_pcd_file` 还指着它）。注意：一旦救急结束，切回 `use_dual_mid360:=True` 时**必须**回到新 PCD，否则立刻回到第 2 节描述的错误状态。

**Q8. 新 PCD 放哪里，要提交 git 吗？**
推荐放 `~/Documents/Sentry26_PCD/` 或 `src/sentry_nav_bringup/pcd/reality/`。**不要 commit**：本仓库的 `.gitignore` 已覆盖 `*.pcd`、`*.pgm` 等二进制，PCD 体积大、易与 git-lfs 冲突。团队共享建议走内部文件服务器或 rsync；版本管理走"PCD 文件名 + 生成日期 + commit hash 的配套 README" 的方式手动维护。

---

## 8. 相关文档

- 同目录 [`ARCHITECTURE.md`](ARCHITECTURE.md) §14 — 迁移规则的架构层摘要。
- 同目录 [`SYNC_VERIFICATION.md`](SYNC_VERIFICATION.md) — 硬件时钟同步四步法，建图前先确认同步质量。
- 仓库根 `src/docs/RUNNING_MODES.md` §4 — SLAM 建图模式总览（非双雷达专属）。
- 仓库根 `src/docs/TUNING_GUIDE.md` — Point-LIO / GICP 参数调优；PCD 不兼容引发的"假性调参"问题需先用本指南排除。
- `.sisyphus/evidence/task-21-blocker.md` — T21 实车建图 BLOCKED 证据，内含实车操作员可直接复制的命令序列。

## 9. 变更记录

- 2026-05-06：初版。覆盖迁移适用范围、旧 PCD 不兼容的算法根因、迁移前检查清单、旧资产备份、实车慢速建图与新 PCD 生成、新 PCD 验证流程与本地 CI 诚实声明、FAQ。

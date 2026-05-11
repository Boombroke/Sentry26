# sentry_dual_mid360

## 简介
本包是 Sentry26 机器人双 Mid360 激光雷达融合与相关资产的统一存放地，集中维护点云融合节点、双雷达 xmacro / 配置 / launch 资产，以及 Point-LIO 派生参数生成脚本。

## 包含模块
- **pointcloud_merger**: 将前 / 后 Mid360 的 Livox CustomMsg 点云融合到 `front_mid360` 系，保留逐点时间戳、反射率、tag、line 等 Point-LIO 需要的字段。
- **sim_pointcloud_to_custommsg_node**: 仿真侧 bridge。把 Gazebo gpu_lidar 发布的 `sensor_msgs/PointCloud2` 转换成 `livox_ros_driver2/msg/CustomMsg`，使 `MergerNode` 和 Point-LIO 在仿真与实车复用同一条 CustomMsg 链路（Point-LIO 源码零改动）。`launch/sim_custommsg_bridge_launch.py` 启动前后两实例，对应 `livox/lidar_front_points` / `livox/lidar_back_points` → `livox/lidar_front` / `livox/lidar_back`。
- **双 Mid360 资产**: `urdf/` 提供后雷达与 IMU TF xmacro fragment，`config/` 提供双雷达驱动与 merger 参数（相对话题命名，namespace 安全），`launch/` 提供 merger 与仿真 bridge 启动入口。
- **Point-LIO override codegen**: 构建时从 xmacro 单源派生 `mapping.gravity`、`gravity_init`、`extrinsic_T` 和 `extrinsic_R`，供后续 dual launch 覆盖 Point-LIO 参数。

## 目录结构
- `include/pointcloud_merger`: 头文件
- `src`: 源代码
- `urdf`: 机器人描述文件（双雷达配置）
- `config`: 配置文件
- `launch`: 启动文件
- `scripts`: 辅助脚本，按用途分子目录：
  - `codegen/`: 构建期 codegen 与 freshness 校验（`generate_pointlio_overrides.py` / `verify_pointlio_overrides_fresh.py`）
  - `calib/`: 标定主线 T7→T10→T11（`verify_dual_mid360_sync.py` / `record_calib_bag.sh` / `calibrate_dual_mid360.sh`）
  - `tools/`: 可视化与开发辅助（`preview_real_xmacro.sh` 3D 预览、`lidar_only_debug.sh` 一键起只含激光的最小调试栈）
  - `qa/`: 运行态诊断（`qa_merger_latency.py` / `qa_odom_bridge_dual.py` / `qa_terrain_coverage.py` / `analyze_map_odom_stability.py` / `analyze_static_drift.py`）
  - `e2e/`: 系统级静态回归（`test_real_dual_mid360_static.sh` / `test_sim_dual_mid360.sh`）
- `docs`: 相关文档

> `ros2 run sentry_dual_mid360 <script>` 仍然接受扁平脚本名——`CMakeLists.txt` 把各子目录脚本一起装到 `lib/sentry_dual_mid360/` 下，源码树分类、运行时扁平。

## Point-LIO override 生成流程

`pointlio_dual_overrides.yaml` 是构建产物，不在 `config/` 中手写提交。执行：

```bash
colcon build --symlink-install --packages-select sentry_dual_mid360 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

CMake 会运行 `scripts/codegen/generate_pointlio_overrides.py`，生成并安装：

```text
share/sentry_dual_mid360/config/pointlio_dual_overrides.yaml
```

生成器只读取两个 xmacro 输入：

- `src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro` 的 `front_lidar_pose`（Layer A：前雷达安装位姿，用于派生 gravity）。
- `src/sentry_nav/sentry_dual_mid360/urdf/mid360_imu_tf.sdf.xmacro` 的 IMU factory pose（Layer B：用于派生 `extrinsic_T/R`）。

缺失或非法输入会直接失败，不会静默回退到默认值，也不会读取环境变量、JSON、已有 YAML 或文档作为第二真相源。修改外参时请先改 xmacro，再重新构建本包。

## 文档索引

> **第一次做标定？直接看 [外参标定速通](docs/CALIBRATION_QUICKSTART.md)，一页纸跑通 T7→T10→T11。**

- [**外参标定速通** (新手必看)](docs/CALIBRATION_QUICKSTART.md)：从插电到 xmacro 写回的最短路径，附 `--bootstrap` 模式说明与常见坑。
- [架构设计](docs/ARCHITECTURE.md)：包级权威架构文档，覆盖双 Mid360 融合链路、TF 树、Point-LIO override codegen 与运行时 QA。
- [硬件同步验证 SOP](docs/SYNC_VERIFICATION.md)：Livox PTP / PPS 硬件同步四步判定与现场排查流程。
- [PCD 迁移 SOP](docs/PCD_MIGRATION.md)：从单 Mid360 向双 Mid360 切换时先验点云的备份、重建、校验流程。
- 外参标定入口脚本：[`scripts/calib/calibrate_dual_mid360.sh`](scripts/calib/calibrate_dual_mid360.sh)（`--check-deps` / `--dry-run` / `--bootstrap` / 标定流程说明均在脚本内）；依赖 `third_party/Multi_LiCa` 与标定 bag，具体先决条件见 `ARCHITECTURE.md`。

## 常用小工具

- **实车 xmacro 3D 预览**：改完 `wheeled_biped_real.sdf.xmacro` 的
  `front_lidar_pose` / `back_lidar_pose` 想立刻在 3D 里看效果，跑：
  ```bash
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  bash src/sentry_nav/sentry_dual_mid360/scripts/tools/preview_real_xmacro.sh
  ```
  脚本用 `xmacro4sdf` 展开到临时 SDF，起一个空世界的 Gazebo Harmonic 把模型
  spawn 在原点，只做可视化——不拉 nav2/slam/Point-LIO，关窗口即退出。
  加 `--headless` 只校验 SDF 是否能加载，加 `--keep-sdf` 保留展开结果便于
  diff 对比。

- **只看激光的最小调试栈**：下位机没接 / 只想在 rviz 里肉眼验证双雷达
  点云叠加，不用起整套 nav2/slam/serial：
  ```bash
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  bash src/sentry_nav/sentry_dual_mid360/scripts/tools/lidar_only_debug.sh --with-pointlio
  ```
  一个终端内起：livox driver + robot_state_publisher（发 xmacro 静态
  TF） + 两条 identity static TF（`map→odom`、`odom→base_footprint`） +
  merger + Point-LIO + rviz2。Ctrl-C 一次收尾全部子进程。

  rviz 里 Fixed Frame 手打 `map`（脚本已补 `map→camera_init` identity TF），
  加 `PointCloud2 → /cloud_registered` 就能看到融合点云。`--with-pointlio`
  隐含 `--with-merger`。因为 livox CustomMsg 本身 rviz 无法 render，必须
  通过 Point-LIO 得到 PointCloud2。`--no-rviz` / `--no-driver` / `--no-rsp`
  可按需关掉对应组件。

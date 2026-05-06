# sentry_dual_mid360

## 简介
本包是 Sentry26 机器人双 Mid360 激光雷达融合与相关资产的统一存放地，集中维护点云融合节点、双雷达 xmacro / 配置 / launch 资产，以及 Point-LIO 派生参数生成脚本。

## 包含模块
- **pointcloud_merger**: 将前 / 后 Mid360 的 Livox CustomMsg 点云融合到 `front_mid360` 系，保留逐点时间戳、反射率、tag、line 等 Point-LIO 需要的字段。
- **双 Mid360 资产**: `urdf/` 提供后雷达与 IMU TF xmacro fragment，`config/` 提供双雷达驱动与 merger 参数，`launch/` 提供 merger 启动入口。
- **Point-LIO override codegen**: 构建时从 xmacro 单源派生 `mapping.gravity`、`gravity_init`、`extrinsic_T` 和 `extrinsic_R`，供后续 dual launch 覆盖 Point-LIO 参数。

## 目录结构
- `include/pointcloud_merger`: 头文件
- `src`: 源代码
- `urdf`: 机器人描述文件（双雷达配置）
- `config`: 配置文件
- `launch`: 启动文件
- `scripts`: 辅助脚本
- `docs`: 相关文档

## Point-LIO override 生成流程

`pointlio_dual_overrides.yaml` 是构建产物，不在 `config/` 中手写提交。执行：

```bash
colcon build --symlink-install --packages-select sentry_dual_mid360 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

CMake 会运行 `scripts/generate_pointlio_overrides.py`，生成并安装：

```text
share/sentry_dual_mid360/config/pointlio_dual_overrides.yaml
```

生成器只读取两个 xmacro 输入：

- `src/sentry_robot_description/resource/xmacro/wheeled_biped_real.sdf.xmacro` 的 `front_lidar_pose`（Layer A：前雷达安装位姿，用于派生 gravity）。
- `src/sentry_nav/sentry_dual_mid360/urdf/mid360_imu_tf.sdf.xmacro` 的 IMU factory pose（Layer B：用于派生 `extrinsic_T/R`）。

缺失或非法输入会直接失败，不会静默回退到默认值，也不会读取环境变量、JSON、已有 YAML 或文档作为第二真相源。修改外参时请先改 xmacro，再重新构建本包。

## 文档索引
- [架构设计](docs/ARCHITECTURE.md) (待补充)
- [外参标定指南](docs/CALIBRATION.md) (待补充)

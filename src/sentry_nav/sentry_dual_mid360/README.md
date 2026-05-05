# sentry_dual_mid360

## 简介
本项目是 Sentry26 机器人双 Mid360 激光雷达融合与相关资产的统一存放地。当前仅包含功能包骨架。

## 包含模块 (规划中)
- **pointcloud_merger**: 计划负责将两个 Mid360 的点云融合成一个统一的点云流（待 T8 阶段实现）。

## 目录结构
- `include/pointcloud_merger`: 头文件
- `src`: 源代码
- `urdf`: 机器人描述文件（双雷达配置）
- `config`: 配置文件
- `launch`: 启动文件
- `scripts`: 辅助脚本
- `docs`: 相关文档

## 文档索引
- [架构设计](docs/ARCHITECTURE.md) (待补充)
- [外参标定指南](docs/CALIBRATION.md) (待补充)

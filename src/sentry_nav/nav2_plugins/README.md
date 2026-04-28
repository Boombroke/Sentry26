# nav2_plugins

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/nav2_plugins/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/nav2_plugins/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Overview

`nav2_plugins` 是一个用于扩展 `Navigation2`（Nav2）框架的插件库，当前提供自定义代价地图层以增强机器人在导航过程中的感知能力。

> `BackUpFreeSpace` 恢复行为已于 2026 赛季重构时删除。贴墙脱困由 `sentry_motion_manager` 的 recovery 状态机通过 `cmd_vel_recovery` 接管，不再依赖 Nav2 behavior_server 插件路径。

## 2. Plugins

### 2.1 Layers

#### 2.1.1 IntensityVoxelLayer

`IntensityVoxelLayer` 是一个用于处理点云数据中障碍物强度信息的代价地图层。它可以根据点云数据中的强度值来标记障碍物，并将这些障碍物信息添加到代价地图中，本插件推荐配合 [terrain_analysis](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis) 功能包使用。

"costmap 太密"通常指局部或全局代价地图中非零 cell 占比很高。它不等于全是实体障碍，很多 cell 是 inflation layer 生成的安全代价带；例如 `robot_radius=0.46` 且 `inflation_radius=0.90` 时，5m 局部窗口内墙边和障碍周围会出现大片非零代价。这样更保守、防擦角，但狭窄通道和恢复行为更容易认为可走空间不足。调参时应同时看 lethal cell、inflated cell 和真实点云，不应只凭"颜色很多"直接降低安全距离。

**Parameters:**

Common to [costmap-plugins/voxel.html](https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html)。

主要区别在于添加了 `min_obstacle_intensity` 和 `max_obstacle_intensity` 参数，注意此参数使用域在 `obstacle_layer` 下，而非 `observation_sources` 下。

**Example:**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.46
      plugins: ["static_layer", "intensity_voxel_layer", "inflation_layer"]
      intensity_voxel_layer:
        plugin: nav2_costmap_2d_plugins::IntensityVoxelLayer
        enabled: true
        track_unknown_space: true
        footprint_clearing_enabled: true
        publish_voxel_map: false
        combination_method: 1
        mark_threshold: 0
        origin_z: 0.0
        unknown_threshold: 5
        z_resolution: 0.05
        z_voxels: 16
        min_obstacle_height: 0.0
        max_obstacle_height: 2.0
        min_obstacle_intensity: 0.05
        max_obstacle_intensity: 2.0
        observation_sources: terrain_map
        terrain_map:
          data_type: PointCloud2
          topic: /terrain_map
          min_obstacle_height: 0.0
          max_obstacle_height: 2.0
          obstacle_max_range: 5.0
          obstacle_min_range: 0.2
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 8.0
        inflation_radius: 0.90
      always_send_full_costmap: False
```

## Acknowledgements

The Initial Developer of some parts of the repository (`IntensityVoxelLayer`), which are copied from, derived from, or
inspired by @PolarisXQ [SCURM_SentryNavigation](https://github.com/PolarisXQ/SCURM_SentryNavigation/tree/master/nav2_plugins/behavior_ext_plugins), @ros-navigation [navigation2](https://github.com/ros-navigation).
All Rights Reserved.

# nav2_plugins

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/nav2_plugins/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/nav2_plugins/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Overview

`nav2_plugins` 是一个用于扩展 `Navigation2`（Nav2）框架的插件库，当前提供提供了一些额外的行为逻辑和代价地图层以增强机器人在导航过程中灵活性。

> 当前默认导航 bringup 仍使用 `IntensityVoxelLayer`，但不再加载 `BackUpFreeSpace` 作为默认恢复行为。贴墙脱困后续由 `sentry_motion_manager` 的 recovery 状态机接管；本插件保留用于对比、手动实验和历史兼容。

## 2. Plugins

### 2.1 Behaviors

#### 2.1.1 BackUpFreeSpace

`BackUpFreeSpace` 插件是一个用于在机器人的导航过程中执行后退脱困的插件，主要功能：

1. **搜索后向可执行退出方向**：在 global costmap 中扫描车尾半平面，按自由距离和后向投影评分，而不是选侧前方自由扇区。
2. **差速可执行脱困**：输出 `vx + wz` 的后退回摆弧线，不依赖 `vy`；正后方可退时 `wz=0`，斜后方自由时才转向回摆。
3. **坐标系分离**：方向搜索使用 `global_frame` / global costmap，弧线碰撞检查使用 `local_frame` / local costmap，避免 map->odom 重定位偏移后把 map 位姿误喂给 local collision checker。
4. **保守失败策略**：若后向投影或安全距离不足，直接失败交给下一个 recovery，而不是发送无效侧移或盲退命令。

**Parameters:**

- `max_radius`: 搜索自由空间时的最大半径范围（default：1.0 m）。
- `service_name`: 获取代价图的服务名称（default："local_costmap/get_costmap"）。
- `min_backward_projection`: 自由空间方向在机体后向上的最小投影，低于该值则拒绝盲退（default：0.2）。
- `max_angular_vel`: recovery 回摆的最大角速度（default：1.0 rad/s）。
- `turn_gain`: 将自由空间夹角转换为角速度的比例系数（default：1.5）。
- `min_escape_clearance`: 候选方向最小无碰撞射线长度，低于该值拒绝后退（default：0.3 m）。
- `unknown_cost_threshold`: costmap cell 达到该值即视为不可穿越，默认 253 表示 unknown/lethal 都阻断恢复射线（default：253）。
- `visualize`: 是否启用可视化功能。启用后会在 RViz 中显示自由空间和目标位置（default：false）。

**Optional example (not used by default bringup):**

```yaml
behavior_server:
ros__parameters:
   use_sim_time: true
   local_costmap_topic: local_costmap/costmap_raw
   global_costmap_topic: global_costmap/costmap_raw
   local_footprint_topic: local_costmap/published_footprint
   global_footprint_topic: global_costmap/published_footprint
   cycle_frequency: 10.0
   behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
   spin:
      plugin: "nav2_behaviors/Spin"
   backup:
      plugin: "nav2_behaviors_plugins/BackUpFreeSpace"
   drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
   wait:
      plugin: "nav2_behaviors/Wait"
   assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
   local_frame: odom
   global_frame: map
   robot_base_frame: base_footprint
   transform_tolerance: 0.1
   simulate_ahead_time: 2.0
   max_rotational_vel: 1.0
   min_rotational_vel: 0.4
   rotational_acc_lim: 3.2
   # params for nav2_behaviors_plugins/BackUpFreeSpace
   max_radius: 2.0
   service_name: "global_costmap/get_costmap"
   min_backward_projection: 0.2
   max_angular_vel: 6.3
   turn_gain: 2.0
   min_escape_clearance: 0.3
   unknown_cost_threshold: 253.0
   visualize: True
```

### 2.2 Layers

#### 2.2.1 IntensityVoxelLayer

`IntensityVoxelLayer` 是一个用于处理点云数据中障碍物强度信息的代价地图层。它可以根据点云数据中的强度值来标记障碍物，并将这些障碍物信息添加到代价地图中，本插件推荐配合 [terrain_analysis](https://github.com/SMBU-PolarBear-Robotics-Team/terrain_analysis) 功能包使用。

“costmap 太密”通常指局部或全局代价地图中非零 cell 占比很高。它不等于全是实体障碍，很多 cell 是 inflation layer 生成的安全代价带；例如 `robot_radius=0.46` 且 `inflation_radius=0.90` 时，5m 局部窗口内墙边和障碍周围会出现大片非零代价。这样更保守、防擦角，但狭窄通道和恢复行为更容易认为可走空间不足。调参时应同时看 lethal cell、inflated cell 和真实点云，不应只凭“颜色很多”直接降低安全距离。

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

The Initial Developer of some parts of the repository (`BackUpFreeSpace`, `IntensityVoxelLayer`, `IntensityVoxelLayer`), which are copied from, derived from, or
inspired by @PolarisXQ [SCURM_SentryNavigation](https://github.com/PolarisXQ/SCURM_SentryNavigation/tree/master/nav2_plugins/behavior_ext_plugins), @ros-navigation [navigation2](https://github.com/ros-navigation).
All Rights Reserved.

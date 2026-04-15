# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

Given a registered pointcloud (based on the odom frame) and prior pointcloud (mapped using [pointlio](https://github.com/LihanChen2004/Point-LIO) or similar tools), the node will calculate the transformation between the two point clouds and publish the correction from the `map` frame to the `odom` frame.

The published map→odom transform is constrained to 2D (x, y, yaw only), consistent with standard 2D navigation localization (e.g. AMCL).

## Dependencies

- ROS2 Humble
- small_gicp (>= v1.0.0, C++17)
- pcl
- OpenMP

## Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/LihanChen2004/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install -DCMAKE_BUILD_TYPE=release
    ```

## Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `num_threads` | int | 4 | OpenMP thread count for GICP |
| `num_neighbors` | int | 20 | Neighbors for covariance estimation |
| `global_leaf_size` | float | 0.25 | Voxel leaf size for prior map downsampling |
| `registered_leaf_size` | float | 0.25 | Voxel leaf size for accumulated scan downsampling |
| `max_dist_sq` | float | 1.0 | Max squared distance for GICP correspondence rejection |
| `max_iterations` | int | 20 | Max LM optimizer iterations for GICP |
| `accumulated_count_threshold` | int | 20 | Number of scan frames to accumulate before registration |
| `min_range` | double | 0.5 | Min point range filter (removes near-range noise) |
| `min_inlier_ratio` | double | 0.3 | Min inlier ratio to accept GICP result |
| `max_fitness_error` | double | 1.0 | Max per-inlier fitness error to accept GICP result |
| `enable_periodic_relocalization` | bool | false | Enable periodic re-registration after initial localization |
| `relocalization_interval` | double | 30.0 | Interval (seconds) between periodic relocalization attempts |
| `map_frame` | string | "map" | Map frame ID |
| `odom_frame` | string | "odom" | Odom frame ID |
| `base_frame` | string | "" | Robot base frame (for PCD map transform) |
| `robot_base_frame` | string | "" | Robot base frame (for initialpose callback) |
| `lidar_frame` | string | "" | LiDAR frame ID |
| `prior_pcd_file` | string | "" | Path to prior PCD map file |
| `init_pose` | double[] | [0,0,0,0,0,0] | Initial pose [x, y, z, roll, pitch, yaw] |

## Usage

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Adjust the transformation between `base_frame` and `lidar_frame`

    The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the `lidar_odom` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    If not set, empty transformation will be used.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```

## Quality Gates

The node validates GICP results before accepting them:

1. **Convergence check**: GICP must report convergence
2. **Inlier ratio**: `num_inliers / source_points >= min_inlier_ratio`
3. **Fitness error**: `total_error / num_inliers <= max_fitness_error`
4. **Periodic correction bound**: Periodic relocalization rejects corrections > 2m

If the initial registration fails quality checks, the node automatically retries with newly accumulated frames.

# Copyright 2026 Boombroke
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Standalone dual Mid360 livox driver launcher.

Starts ONLY `livox_ros_driver2_node` configured for the dual-Mid360 topology
(CustomMsg, multi_topic=1, 10 Hz) with per-device topic remaps to
`/livox/lidar_front`, `/livox/lidar_back` (plus matching IMU topics).

Purpose: give standalone scripts (T10 calibration bag recording, T7 sync
verification, driver-only debugging) a way to bring the dual Mid360 driver
up without pulling in the full nav2/slam/merger/Point-LIO stack that
`rm_navigation_reality_launch.py` layers on top. Using the full reality
bringup for these workflows has in the past caused QoS-incompatible
subscribers to starve the driver path and empty bag recordings — this
launcher isolates the driver from those concerns.

NOT for production: the real robot path is still
`rm_navigation_reality_launch.py`, which layers this same driver config
on top of the rest of the stack. Keep behavior equivalent by reusing the
same JSON (`mid360_user_config_dual.json`) and override YAML
(`livox_driver_dual_override.yaml`).
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def _livox_ip_suffix(ip: str) -> str:
    return ip.replace(".", "_")


def _load_dual_mid360_ip_suffixes():
    dual_share = get_package_share_directory("sentry_dual_mid360")
    json_path = os.path.join(dual_share, "config", "mid360_user_config_dual.json")
    with open(json_path) as f:
        cfg = json.load(f)
    configs = cfg.get("lidar_configs", [])
    if len(configs) < 2:
        raise RuntimeError(
            f"mid360_user_config_dual.json must have at least 2 lidar_configs entries "
            f"(found {len(configs)}): {json_path}"
        )
    return _livox_ip_suffix(configs[0]["ip"]), _livox_ip_suffix(configs[1]["ip"])


def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS namespace to apply to the Livox driver node.",
    )
    # livox_driver_dual_override.yaml defaults xfer_format=1 (CustomMsg only)
    # — that is what Point-LIO and the merger consume in production. rviz2
    # cannot render CustomMsg directly, so pass xfer_format:=4 when the goal
    # is pure lidar debugging / calibration visualization; the driver then
    # publishes /livox/lidar_<ip> as sensor_msgs/PointCloud2 on the AllMsg
    # code path while still producing CustomMsg for Point-LIO.
    # Do NOT enable this on the production real-robot bringup — the extra
    # PC2 doubles the driver's publish CPU for no downstream benefit.
    declare_xfer_format_cmd = DeclareLaunchArgument(
        "xfer_format",
        default_value="1",
        description=(
            "Livox xfer_format: 1 = CustomMsg only (default, production); "
            "4 = AllMsg (CustomMsg + PointCloud2, rviz-friendly debug)."
        ),
    )

    front_ip_sfx, back_ip_sfx = _load_dual_mid360_ip_suffixes()

    # Per-device topic remaps mirror rm_navigation_reality_launch.py so that
    # downstream consumers expecting /livox/lidar_front, /livox/lidar_back,
    # /livox/imu, /livox/imu_back see identical topics whether the driver was
    # brought up standalone or as part of the full reality bringup.
    livox_dual_remappings = [
        (f"livox/lidar_{front_ip_sfx}", "livox/lidar_front"),
        (f"livox/imu_{front_ip_sfx}",   "livox/imu"),
        (f"livox/lidar_{back_ip_sfx}",  "livox/lidar_back"),
        (f"livox/imu_{back_ip_sfx}",    "livox/imu_back"),
    ]

    livox_dual_override_file = PathJoinSubstitution(
        [
            FindPackageShare("sentry_dual_mid360"),
            "config",
            "livox_driver_dual_override.yaml",
        ]
    )

    start_livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            ParameterFile(livox_dual_override_file, allow_substs=True),
            # Last parameters entry wins — this lets `xfer_format:=4` on the
            # CLI override whatever is baked in livox_driver_dual_override.yaml.
            # ParameterValue(value_type=int) coerces the CLI string to the
            # integer type the driver expects; a raw str would be rejected.
            {"xfer_format": ParameterValue(LaunchConfiguration("xfer_format"), value_type=int)},
        ],
        remappings=livox_dual_remappings,
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_xfer_format_cmd,
        start_livox_driver,
    ])

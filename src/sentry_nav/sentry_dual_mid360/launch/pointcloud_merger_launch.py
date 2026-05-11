# Copyright 2026 Boombroke
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    publish_pc2_preview = LaunchConfiguration("publish_pc2_preview")

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("sentry_dual_mid360"),
            "config",
            "pointcloud_merger_params.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            # Off by default (production path); rviz debug paths pass True to
            # get a sensor_msgs/PointCloud2 mirror of the merged CustomMsg.
            DeclareLaunchArgument(
                "publish_pc2_preview",
                default_value="false",
                description=(
                    "Also publish the merged cloud as sensor_msgs/PointCloud2 "
                    "on /livox/lidar_pc2 so rviz can visualize it. Off by "
                    "default since the production consumer (Point-LIO) reads "
                    "CustomMsg only."
                ),
            ),
            Node(
                package="sentry_dual_mid360",
                executable="merger_node",
                name="pointcloud_merger",
                namespace=namespace,
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                    # ROS params "last wins" — this overrides the yaml default.
                    {"publish_pc2_preview": ParameterValue(
                        publish_pc2_preview, value_type=bool)},
                ],
            ),
        ]
    )

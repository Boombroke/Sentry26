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
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

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
            Node(
                package="sentry_dual_mid360",
                executable="merger_node",
                name="pointcloud_merger",
                namespace=namespace,
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )

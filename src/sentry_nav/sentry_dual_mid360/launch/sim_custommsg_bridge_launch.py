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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    front_input_topic = LaunchConfiguration("front_input_topic")
    back_input_topic = LaunchConfiguration("back_input_topic")
    front_output_topic = LaunchConfiguration("front_output_topic")
    back_output_topic = LaunchConfiguration("back_output_topic")
    front_frame_id = LaunchConfiguration("front_frame_id")
    back_frame_id = LaunchConfiguration("back_frame_id")
    line_count = ParameterValue(LaunchConfiguration("line_count"), value_type=int)
    scan_period_s = ParameterValue(
        LaunchConfiguration("scan_period_s"), value_type=float
    )
    reflectivity = ParameterValue(
        LaunchConfiguration("reflectivity"), value_type=int
    )
    tag = ParameterValue(LaunchConfiguration("tag"), value_type=int)

    front_node = Node(
        package="sentry_dual_mid360",
        executable="sim_pointcloud_to_custommsg_node",
        name="sim_pc2_to_custom_front",
        namespace=namespace,
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "input_topic": front_input_topic,
                "output_topic": front_output_topic,
                "frame_id": front_frame_id,
                "lidar_id": 0,
                "line_count": line_count,
                "scan_period_s": scan_period_s,
                "reflectivity": reflectivity,
                "tag": tag,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    back_node = Node(
        package="sentry_dual_mid360",
        executable="sim_pointcloud_to_custommsg_node",
        name="sim_pc2_to_custom_back",
        namespace=namespace,
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "input_topic": back_input_topic,
                "output_topic": back_output_topic,
                "frame_id": back_frame_id,
                "lidar_id": 1,
                "line_count": line_count,
                "scan_period_s": scan_period_s,
                "reflectivity": reflectivity,
                "tag": tag,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "front_input_topic", default_value="livox/lidar_front_points"
            ),
            DeclareLaunchArgument(
                "back_input_topic", default_value="livox/lidar_back_points"
            ),
            DeclareLaunchArgument(
                "front_output_topic", default_value="livox/lidar_front"
            ),
            DeclareLaunchArgument(
                "back_output_topic", default_value="livox/lidar_back"
            ),
            DeclareLaunchArgument("front_frame_id", default_value="front_mid360"),
            DeclareLaunchArgument("back_frame_id", default_value="back_mid360"),
            DeclareLaunchArgument("line_count", default_value="4"),
            DeclareLaunchArgument("scan_period_s", default_value="0.1"),
            DeclareLaunchArgument("reflectivity", default_value="10"),
            DeclareLaunchArgument("tag", default_value="16"),
            front_node,
            back_node,
        ]
    )

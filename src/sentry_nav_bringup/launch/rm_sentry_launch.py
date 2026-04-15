# Copyright 2025 Lihan Chen
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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("sentry_nav_bringup")
    serial_dir = get_package_share_directory("rm_serial_driver")
    launch_dir = os.path.join(bringup_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    use_foxglove = LaunchConfiguration("use_foxglove")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether to run SLAM instead of localization",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="rmul_2024",
        description="Map name (rmul_2024 / rmuc_2024 etc.)",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to start RViz",
    )

    declare_use_foxglove_cmd = DeclareLaunchArgument(
        "use_foxglove",
        default_value="False",
        description="Whether to start foxglove_bridge",
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "rm_navigation_reality_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "world": world,
            "use_rviz": use_rviz,
            "use_foxglove": use_foxglove,
        }.items(),
    )

    serial_config = os.path.join(serial_dir, "config", "serial_driver.yaml")

    serial_driver_node = Node(
        package="rm_serial_driver",
        executable="rm_serial_driver_node",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[serial_config],
        arguments=["--ros-args", "--log-level", "rm_serial_driver:=info"],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_foxglove_cmd)

    ld.add_action(navigation_cmd)
    ld.add_action(serial_driver_node)

    return ld

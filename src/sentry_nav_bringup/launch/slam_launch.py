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
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.descriptions import ParameterFile
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory("sentry_nav_bringup")
    dual_mid360_share = get_package_share_directory("sentry_dual_mid360")

    # Input parameters declaration
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_dual_mid360 = LaunchConfiguration("use_dual_mid360")

    # Variables
    # Jazzy Nav2 bringup: slam_toolbox manages its own lifecycle (use_lifecycle_manager=false
    # by default in online_sync_launch.py); lifecycle_manager_slam only manages map_saver.
    lifecycle_nodes = ["map_saver"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_use_dual_mid360_cmd = DeclareLaunchArgument(
        "use_dual_mid360",
        default_value="True",
        description=(
            "When True (default), start pointcloud_merger before Point-LIO, load "
            "sentry_dual_mid360/config/pointlio_dual_overrides.yaml (generated from "
            "xmacro at build time) on top of base Point-LIO params, and point "
            "Point-LIO at the merger output topic `livox/lidar`. When False, the "
            "merger is disabled and Point-LIO keeps the base params_file topic "
            "`livox/lidar` for single-lidar operation."
        ),
    )

    # Point-LIO override YAML generated at build time by sentry_dual_mid360 codegen
    # from wheeled_biped_real.sdf.xmacro + mid360_imu_tf.sdf.xmacro. Only loaded in
    # dual mode; single mode keeps base nav2_params.yaml Point-LIO section untouched.
    pointlio_dual_overrides_file = PathJoinSubstitution(
        [
            FindPackageShare("sentry_dual_mid360"),
            "config",
            "pointlio_dual_overrides.yaml",
        ]
    )

    # Explicit pointcloud_merger params file (plan T13 requires explicit pass-through
    # rather than relying on pointcloud_merger_launch.py's built-in default).
    pointcloud_merger_params_file = PathJoinSubstitution(
        [
            FindPackageShare("sentry_dual_mid360"),
            "config",
            "pointcloud_merger_params.yaml",
        ]
    )

    start_map_saver_server_cmd = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
    )

    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    start_pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", "terrain_map_ext"),
            ("scan", "obstacle_scan"),
        ],
    )

    start_sync_slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/map_updates", "map_updates"),
        ],
    )

    configure_slam_toolbox_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate_slam_toolbox_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_sync_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart),
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"prior_pcd.enable": False},
            {"pcd_save.pcd_save_en": True},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        condition=UnlessCondition(use_dual_mid360),
    )

    start_point_lio_node_dual = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            pointlio_dual_overrides_file,
            {"common.lid_topic": "livox/lidar"},
            {"common.imu_topic": "livox/imu"},
            {"preprocess.lidar_type": 1},
            {"preprocess.scan_line": 4},
            {"prior_pcd.enable": False},
            {"pcd_save.pcd_save_en": True},
        ],
        arguments=["--ros-args", "--log-level", log_level],
        condition=IfCondition(use_dual_mid360),
    )

    start_pointcloud_merger_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_mid360_share, "launch", "pointcloud_merger_launch.py")
        ),
        condition=IfCondition(use_dual_mid360),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": pointcloud_merger_params_file,
        }.items(),
    )

    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_dual_mid360_cmd)

    # Running Map Saver Server
    ld.add_action(start_map_saver_server_cmd)

    ld.add_action(start_pointcloud_to_laserscan_node)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(configure_slam_toolbox_event)
    ld.add_action(activate_slam_toolbox_event)
    ld.add_action(start_pointcloud_merger_cmd)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_point_lio_node_dual)
    ld.add_action(start_static_transform_node)

    # lifecycle_manager 在被管理节点之后启动，减少 bond 超时误报
    ld.add_action(start_lifecycle_manager_cmd)

    return ld

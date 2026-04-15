import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    world_sdf_path = LaunchConfiguration("world_sdf_path")
    ign_config_path = LaunchConfiguration("ign_config_path")
    headless = LaunchConfiguration("headless")

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        default_value=os.path.join(
            pkg_simulator, "resource", "worlds", "rmul_2024_world.sdf"
        ),
        description="Path to the world SDF file",
    )

    declare_ign_config_path = DeclareLaunchArgument(
        "ign_config_path",
        default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
        description="Path to the Ignition Gazebo GUI configuration file",
    )

    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI (server only, saves GPU resources)",
    )

    append_enviroment_worlds = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.path.join(pkg_simulator, "resource", "worlds"),
    )

    append_enviroment_models = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.path.join(pkg_simulator, "resource", "models"),
    )

    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "6",
            "gz_args": [
                world_sdf_path,
                TextSubstitution(text=" --gui-config "),
                ign_config_path,
            ],
        }.items(),
        condition=UnlessCondition(headless),
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "6",
            "gz_args": [
                world_sdf_path,
                TextSubstitution(text=" -s --headless-rendering"),
            ],
        }.items(),
        condition=IfCondition(headless),
    )

    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_sdf_path)
    ld.add_action(declare_ign_config_path)
    ld.add_action(declare_headless)
    ld.add_action(append_enviroment_worlds)
    ld.add_action(append_enviroment_models)
    ld.add_action(gazebo_with_gui)
    ld.add_action(gazebo_headless)
    ld.add_action(robot_ign_bridge)

    return ld

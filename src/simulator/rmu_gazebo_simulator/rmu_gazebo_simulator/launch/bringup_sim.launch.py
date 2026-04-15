import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")

    world_sdf_path = os.path.join(
        pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf"
    )
    ign_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    manual_gz = LaunchConfiguration("manual_gz")
    headless = LaunchConfiguration("headless")
    spawn_delay = LaunchConfiguration("spawn_delay")

    declare_manual_gz_cmd = DeclareLaunchArgument(
        "manual_gz",
        default_value="false",
        description="If true, skip Gazebo launch (user starts it manually first)",
    )

    declare_headless_cmd = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI (passed to gazebo.launch.py)",
    )

    declare_spawn_delay_cmd = DeclareLaunchArgument(
        "spawn_delay",
        default_value="5.0",
        description="Seconds to wait before spawning robots (gives Gazebo time to stabilize)",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
            "headless": headless,
        }.items(),
        condition=UnlessCondition(manual_gz),
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        launch_arguments={
            "gz_world_path": gz_world_path,
            "world": selected_world,
        }.items(),
    )

    referee_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "referee_system.launch.py")
        )
    )

    # When Gazebo is auto-launched, delay spawning to let it stabilize
    delayed_spawn = TimerAction(
        period=spawn_delay,
        actions=[spawn_robots_launch, referee_system_launch],
        condition=UnlessCondition(manual_gz),
    )

    # When Gazebo is manual, spawn immediately (user already ensured it's ready)
    immediate_spawn_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        launch_arguments={
            "gz_world_path": gz_world_path,
            "world": selected_world,
        }.items(),
        condition=IfCondition(manual_gz),
    )

    immediate_referee = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "referee_system.launch.py")
        ),
        condition=IfCondition(manual_gz),
    )

    ld = LaunchDescription()

    ld.add_action(declare_manual_gz_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_spawn_delay_cmd)

    ld.add_action(gazebo_launch)
    ld.add_action(delayed_spawn)
    ld.add_action(immediate_spawn_robots)
    ld.add_action(immediate_referee)

    return ld

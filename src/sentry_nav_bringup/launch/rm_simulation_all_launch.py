import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    bringup_dir = get_package_share_directory("sentry_nav_bringup")

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as f:
        selected_world = yaml.safe_load(f).get("world", "rmuc_2026")

    world_sdf_path = os.path.join(
        pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf"
    )

    world = LaunchConfiguration("world")
    slam = LaunchConfiguration("slam")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    nav_delay = LaunchConfiguration("nav_delay")

    declare_world_cmd = DeclareLaunchArgument(
        "world", default_value=selected_world,
    )
    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="True",
    )
    declare_headless_cmd = DeclareLaunchArgument(
        "headless", default_value="false",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True",
    )
    declare_nav_delay_cmd = DeclareLaunchArgument(
        "nav_delay", default_value="15.0",
        description="Seconds to wait after Gazebo before starting navigation",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "bringup_sim.launch.py")
        ),
        launch_arguments={"headless": headless}.items(),
    )

    unpause_cmd = ExecuteProcess(
        cmd=[
            "gz", "service",
            "-s", "/world/default/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "5000",
            "--req", "pause: false",
        ],
        output="screen",
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "rm_navigation_simulation_launch.py")
        ),
        launch_arguments={
            "world": world,
            "slam": slam,
            "use_rviz": use_rviz,
        }.items(),
    )

    delayed_unpause = TimerAction(period=8.0, actions=[unpause_cmd])
    delayed_nav = TimerAction(period=nav_delay, actions=[nav_launch])

    ld = LaunchDescription()
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_nav_delay_cmd)
    ld.add_action(gazebo_launch)
    ld.add_action(delayed_unpause)
    ld.add_action(delayed_nav)

    return ld

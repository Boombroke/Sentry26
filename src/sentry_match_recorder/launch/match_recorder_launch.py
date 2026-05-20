"""Launch file for sentry_match_recorder.

Spawns the match recorder node which auto-starts/stops `ros2 bag record`
based on referee game_progress.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_match_recorder')
    default_config = os.path.join(pkg_share, 'config', 'recorder.yaml')

    record_dir_arg = DeclareLaunchArgument(
        'record_dir',
        default_value='logs/match-bags',
        description='Directory to store recorded bag sessions',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to recorder.yaml parameters',
    )

    recorder_node = Node(
        package='sentry_match_recorder',
        executable='match_recorder_node',
        name='match_recorder',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {'record_dir': LaunchConfiguration('record_dir')},
        ],
    )

    return LaunchDescription([
        record_dir_arg,
        config_file_arg,
        recorder_node,
    ])

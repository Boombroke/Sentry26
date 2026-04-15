from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('odom_interpolator')

    params_file = os.path.join(pkg_dir, 'config', 'pid_controller.yaml')

    odom_interpolator_node = Node(
        package='odom_interpolator',
        executable='odom_interpolator',
        name='odom_interpolator',
        output='screen',
        parameters=[params_file]
    )
    
    return LaunchDescription([
        odom_interpolator_node
    ])
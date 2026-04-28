import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    referee_config_path = os.path.join(
        pkg_simulator, "config", "referee_system_1v1.yaml"
    )

    enable_pose_bridge = LaunchConfiguration("enable_pose_bridge")

    declare_enable_pose_bridge_cmd = DeclareLaunchArgument(
        "enable_pose_bridge",
        default_value="false",
        description=(
            "是否启动 pose_bridge 节点。"
            "默认 false，因为 pose_bridge 当前存在构造函数竞态导致 SIGSEGV（exit -11），"
            "且导航栈不依赖 /referee_system/pose_info。"
            "如需调试裁判系统位姿信息，可传入 enable_pose_bridge:=true。"
        ),
    )

    referee_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace="referee_system",
        arguments=[
            "/referee_system/attack_info@std_msgs/msg/String[gz.msgs.StringMsg",
            "/referee_system/shoot_info@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
    )

    referee_ign_pose_bridge = Node(
        package="rmoss_gz_bridge",
        executable="pose_bridge",
        namespace="referee_system",
        condition=IfCondition(enable_pose_bridge),
    )

    referee_ign_rfid_bridge = Node(
        package="rmoss_gz_bridge",
        executable="rfid_bridge",
        namespace="referee_system",
    )

    referee_system = Node(
        package="rmu_gazebo_simulator",
        executable="simple_competition_1v1.py",
        namespace="referee_system",
        parameters=[referee_config_path],
    )

    ld = LaunchDescription()

    ld.add_action(declare_enable_pose_bridge_cmd)

    ld.add_action(referee_ign_bridge)
    ld.add_action(referee_ign_pose_bridge)
    ld.add_action(referee_ign_rfid_bridge)
    ld.add_action(referee_system)

    return ld

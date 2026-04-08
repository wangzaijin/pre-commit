from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_arg = DeclareLaunchArgument(
        "config",
        default_value="configs/turtlebot3.yaml",
        description="Path to the TongRobot config YAML file",
    )

    bridge_node = Node(
        package="tongrobot_bridge",
        executable="bridge_node",
        name="tongrobot_bridge",
        output="screen",
        arguments=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, bridge_node])

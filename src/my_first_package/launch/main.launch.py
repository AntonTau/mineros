from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
        ),
        Node(
            package="my_first_package",
            executable="my_first_node",
            arguments=["--ros-args", "--log-level", log_level],
         ),
        Node(
            package="my_first_package",
            executable="helper_node",
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])

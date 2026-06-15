"""Standalone launch for the dashboard bridge.

Use this when the rest of the stack (demo.launch.py) is already running and you
just want the web API on its own, e.g.:

    ros2 launch autobee_ui ui.launch.py port:=8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = DeclareLaunchArgument("port", default_value="8080")
    enable_camera = DeclareLaunchArgument("enable_camera", default_value="true")

    web_ui = Node(
        package="autobee_ui",
        executable="web_ui_node",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "enable_camera": LaunchConfiguration("enable_camera"),
        }],
    )

    return LaunchDescription([port, enable_camera, web_ui])

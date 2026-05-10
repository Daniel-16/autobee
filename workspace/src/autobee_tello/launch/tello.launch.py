"""Bring up the Tello driver alone (driver-only smoke test, no perception/LLM)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_max = DeclareLaunchArgument("linear_max_mps",  default_value="1.0")
    angular_max = DeclareLaunchArgument("angular_max_rps", default_value="1.0")
    cam_fps = DeclareLaunchArgument("camera_fps", default_value="10.0")
    auto_takeoff = DeclareLaunchArgument("auto_takeoff_on_first_cmd", default_value="true")

    return LaunchDescription([
        linear_max, angular_max, cam_fps, auto_takeoff,
        Node(
            package="autobee_tello",
            executable="tello_driver_node",
            output="screen",
            parameters=[{
                "linear_max_mps":  LaunchConfiguration("linear_max_mps"),
                "angular_max_rps": LaunchConfiguration("angular_max_rps"),
                "camera_fps":      LaunchConfiguration("camera_fps"),
                "auto_takeoff_on_first_cmd": LaunchConfiguration("auto_takeoff_on_first_cmd"),
            }],
        ),
    ])

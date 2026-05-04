"""Bring up ORB-SLAM3 monocular wrapper.

Default paths assume:
  - vocab file at /opt/autobee/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt
    (download via scripts/fetch_vocab.sh after build_orbslam3.sh)
  - settings file from this package's share dir

Override either with launch args:
    ros2 launch autobee_slam orb_slam3.launch.py \\
        vocab_path:=/path/to/ORBvoc.txt \\
        settings_path:=/path/to/Tello.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_VOCAB = "/opt/autobee/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"


def generate_launch_description():
    pkg_share = get_package_share_directory("autobee_slam")
    default_settings = os.path.join(pkg_share, "config", "autobee_camera_sim.yaml")

    vocab = LaunchConfiguration("vocab_path")
    settings = LaunchConfiguration("settings_path")
    use_viewer = LaunchConfiguration("use_viewer")

    return LaunchDescription([
        DeclareLaunchArgument(
            "vocab_path", default_value=DEFAULT_VOCAB,
            description="Path to ORBvoc.txt",
        ),
        DeclareLaunchArgument(
            "settings_path", default_value=default_settings,
            description="Path to ORB-SLAM3 camera settings YAML",
        ),
        DeclareLaunchArgument(
            "use_viewer", default_value="false",
            description="Open the Pangolin viewer (requires X11)",
        ),
        Node(
            package="autobee_slam",
            executable="orb_slam3_node",
            output="screen",
            parameters=[{
                "vocab_path": vocab,
                "settings_path": settings,
                "use_viewer": use_viewer,
            }],
        ),
    ])

"""Bring up the campus sim and the mission executor in one shot.

Usage:  ros2 launch autobee_core demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_launch = os.path.join(
        get_package_share_directory("autobee_sim"), "launch", "sim.launch.py"
    )

    bring_up_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch)
    )

    # Give Gazebo + bridge a few seconds before the executor starts polling /drone/pose.
    mission_executor = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="autobee_core",
                executable="mission_executor_node",
                output="screen",
            ),
        ],
    )

    return LaunchDescription([bring_up_sim, mission_executor])

"""Bring up the campus sim, the LLM agent, and the mission executor in one shot.

Usage:
    ros2 launch autobee_core demo.launch.py

Then in another terminal, send a natural-language command:
    ros2 topic pub --once /user_intent std_msgs/String \
        "data: 'check if the field is overcrowded'"
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

    llm_agent = Node(
        package="autobee_agent",
        executable="llm_agent_node",
        output="screen",
    )

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

    return LaunchDescription([bring_up_sim, llm_agent, mission_executor])

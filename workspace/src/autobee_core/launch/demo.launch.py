"""Bring up the campus sim, real YOLOv8 perception, the LLM agent, and the mission executor.

Usage:
    ros2 launch autobee_core demo.launch.py

Then in another terminal, send a natural-language command:
    ros2 topic pub --times 3 --rate 1 /user_intent std_msgs/msg/String \
        '{data: "check if the field is overcrowded"}'

This launch is Tello-portable: replacing the autobee_sim include with a tello
driver (e.g. tello_socket) keeps the rest of the topic graph identical.
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

    report = Node(
        package="autobee_agent",
        executable="report_node",
        output="screen",
    )

    yolo_perception = Node(
        package="autobee_perception",
        executable="yolov8_perception_node",
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

    return LaunchDescription([
        bring_up_sim,
        llm_agent,
        report,
        yolo_perception,
        mission_executor,
    ])

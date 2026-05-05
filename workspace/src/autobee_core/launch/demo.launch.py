"""Bring up the campus sim, real YOLOv8 perception, the LLM agent, and the mission executor.

Usage:
    ros2 launch autobee_core demo.launch.py
    ros2 launch autobee_core demo.launch.py voice:=true       # add Whisper STT + gTTS TTS
    ros2 launch autobee_core demo.launch.py slam:=true        # bring up ORB-SLAM3 wrapper
    ros2 launch autobee_core demo.launch.py voice:=true slam:=true

Then in another terminal, send a natural-language command (or just talk into
the mic if voice:=true):
    ros2 topic pub --times 3 --rate 1 /user_intent std_msgs/msg/String \
        '{data: "check if the field is overcrowded"}'

This launch is Tello-portable: replacing the autobee_sim include with a tello
driver (e.g. tello_socket) keeps the rest of the topic graph identical.
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    voice_arg = DeclareLaunchArgument(
        "voice", default_value="false",
        description="Bring up Whisper STT + gTTS TTS (requires audio devices in container).",
    )
    slam_arg = DeclareLaunchArgument(
        "slam", default_value="false",
        description="Bring up ORB-SLAM3 wrapper for /drone/pose feedback.",
    )
    tts_backend_arg = DeclareLaunchArgument(
        "tts_backend", default_value="auto",
        description="TTS backend: auto | gtts | pyttsx3 | print",
    )
    whisper_model_arg = DeclareLaunchArgument(
        "whisper_model", default_value="base",
        description="Whisper model size: tiny | base | small | medium",
    )

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

    crowd_analytics = Node(
        package="autobee_perception",
        executable="crowd_analytics_node",
        output="screen",
    )

    anomaly_detection = Node(
        package="autobee_perception",
        executable="anomaly_detection_node",
        output="screen",
    )

    looming_detector = Node(
        package="autobee_perception",
        executable="looming_detector_node",
        output="screen",
    )

    mission_logger = Node(
        package="autobee_core",
        executable="mission_logger_node",
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

    voice_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("voice")),
        actions=[
            Node(
                package="autobee_voice",
                executable="whisper_stt_node",
                output="screen",
                parameters=[{"model": LaunchConfiguration("whisper_model")}],
            ),
            Node(
                package="autobee_voice",
                executable="tts_node",
                output="screen",
                parameters=[{"backend": LaunchConfiguration("tts_backend")}],
            ),
        ],
    )

    # SLAM bringup is optional and lives in its own launch file under
    # autobee_slam (vendored). Only included if the user opts in.
    try:
        slam_launch_path = os.path.join(
            get_package_share_directory("autobee_slam"), "launch", "orb_slam3.launch.py"
        )
        slam_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            condition=IfCondition(LaunchConfiguration("slam")),
        )
    except PackageNotFoundError:
        # autobee_slam not built/installed yet -- skip silently. The slam:=true
        # arg will then have no effect rather than crash the whole launch.
        slam_group = GroupAction(
            condition=IfCondition(LaunchConfiguration("slam")),
            actions=[],
        )

    return LaunchDescription([
        voice_arg,
        slam_arg,
        tts_backend_arg,
        whisper_model_arg,
        bring_up_sim,
        llm_agent,
        report,
        yolo_perception,
        crowd_analytics,
        anomaly_detection,
        looming_detector,
        mission_logger,
        mission_executor,
        voice_group,
        slam_group,
    ])

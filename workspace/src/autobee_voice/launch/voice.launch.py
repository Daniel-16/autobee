"""Standalone voice I/O launch — STT + TTS only.

For the full demo, prefer the unified launch in autobee_core (demo.launch.py),
which includes voice behind a `voice:=true` argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend = LaunchConfiguration("tts_backend")
    whisper_model = LaunchConfiguration("whisper_model")

    return LaunchDescription([
        DeclareLaunchArgument(
            "tts_backend", default_value="auto",
            description="TTS backend: auto | gtts | pyttsx3 | print",
        ),
        DeclareLaunchArgument(
            "whisper_model", default_value="base",
            description="Whisper model size: tiny | base | small | medium",
        ),
        Node(
            package="autobee_voice",
            executable="whisper_stt_node",
            output="screen",
            parameters=[{"model": whisper_model}],
        ),
        Node(
            package="autobee_voice",
            executable="tts_node",
            output="screen",
            parameters=[{"backend": backend}],
        ),
    ])

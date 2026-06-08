"""Full AutoBee pipeline on a real Tello (no Gazebo).

Same node graph as demo.launch.py, but the bottom layer is autobee_tello
instead of autobee_sim, and parameters are tightened for indoor flight.

Usage (laptop must be on Tello Wi-Fi, in the dev container):

    ros2 launch autobee_core demo_tello.launch.py

Then in another terminal:

    ros2 topic pub --times 3 --rate 1 /user_intent std_msgs/msg/String \
        '{data: "do a quick check of the area"}'
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
        description="Bring up Whisper STT + gTTS TTS.",
    )
    slam_arg = DeclareLaunchArgument(
        "slam", default_value="false",
        description="Bring up ORB-SLAM3 wrapper for /drone/pose feedback.",
    )

    try:
        _default_tello_yaml = os.path.join(
            get_package_share_directory("autobee_slam"), "config",
            "autobee_camera_tello.yaml"
        )
    except PackageNotFoundError:
        _default_tello_yaml = ""

    slam_settings_path_arg = DeclareLaunchArgument(
        "slam_settings_path", default_value=_default_tello_yaml,
        description=(
            "ORB-SLAM3 camera settings YAML to use when slam:=true. "
            "Defaults to the Tello YAML (approximate intrinsics — "
            "run a checkerboard calibration and point here for accuracy)."
        ),
    )

    # Indoor flight envelope. Conservative defaults.
    cruise_alt_max = DeclareLaunchArgument("cruise_altitude_max_m", default_value="1.8")
    default_cruise_alt = DeclareLaunchArgument("default_cruise_altitude_m", default_value="1.8")
    default_radius = DeclareLaunchArgument("default_survey_radius_m", default_value="1.9")
    radius_max = DeclareLaunchArgument("survey_radius_max_m", default_value="1.5")
    max_xy_speed = DeclareLaunchArgument("max_xy_speed", default_value="0.3")
    max_z_speed = DeclareLaunchArgument("max_z_speed", default_value="0.3")
    max_yaw_rate = DeclareLaunchArgument("max_yaw_rate", default_value="0.4")

    # Tello driver scaling. cmd_vel of `linear_max_mps` -> 100% RC stick.
    linear_max = DeclareLaunchArgument("linear_max_mps", default_value="1.5")
    angular_max = DeclareLaunchArgument("angular_max_rps", default_value="1.5")

    # Sim-specific obstacle avoidance OFF — campus.sdf coords don't match reality.
    # Looming + YOLO backstop stays ON as the runtime safety net.
    avoid_known = DeclareLaunchArgument("avoid_known_obstacles", default_value="false")
    perception_backstop = DeclareLaunchArgument("perception_backstop", default_value="true")
    # Named landmarks (library/admin/etc.) don't exist indoors — refuse landmark
    # transits so detect_anomaly_buildings degrades to a local hover.
    landmarks_enabled = DeclareLaunchArgument("landmarks_enabled", default_value="false")

    tello_driver = Node(
        package="autobee_tello",
        executable="tello_driver_node",
        output="screen",
        parameters=[{
            "linear_max_mps":  LaunchConfiguration("linear_max_mps"),
            "angular_max_rps": LaunchConfiguration("angular_max_rps"),
            "camera_fps": 10.0,
            "imu_hz": 10.0,
            "auto_takeoff_on_first_cmd": True,
        }],
    )

    # Feedback contract: Tello telemetry -> /drone/odom (real half).
    tello_odometry = Node(
        package="autobee_control", executable="tello_odometry_node", output="screen",
    )
    # Reactive inner loop: /drone/goal + /drone/odom -> /drone/cmd_vel.
    # Indoor speed envelope mirrors the executor's conservative limits.
    flight_controller = Node(
        package="autobee_control", executable="flight_controller_node", output="screen",
        parameters=[{
            "max_xy_speed":  LaunchConfiguration("max_xy_speed"),
            "max_z_speed":   LaunchConfiguration("max_z_speed"),
            "max_yaw_rate":  LaunchConfiguration("max_yaw_rate"),
            "avoidance_enabled": LaunchConfiguration("perception_backstop"),
        }],
    )

    llm_agent = Node(
        package="autobee_agent", executable="llm_agent_node", output="screen",
    )
    report = Node(
        package="autobee_agent", executable="report_node", output="screen",
    )
    yolo_perception = Node(
        package="autobee_perception", executable="yolov8_perception_node", output="screen",
    )
    crowd_analytics = Node(
        package="autobee_perception", executable="crowd_analytics_node", output="screen",
    )
    anomaly_detection = Node(
        package="autobee_perception", executable="anomaly_detection_node", output="screen",
    )
    looming_detector = Node(
        package="autobee_perception", executable="looming_detector_node", output="screen",
    )
    mission_logger = Node(
        package="autobee_core", executable="mission_logger_node", output="screen",
        parameters=[{
            "log_root": "/workspaces/AUTOBEE/mission_results",
            "save_frames": True,
            "frame_topic": "/perception/image_annotated",
            "frame_interval_s": 1.0,
            "zip_on_finish": True,
        }],
    )

    # Mission executor with indoor-safe params + obstacle table off.
    mission_executor = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="autobee_core",
                executable="mission_executor_node",
                output="screen",
                parameters=[{
                    "default_cruise_altitude_m": LaunchConfiguration("default_cruise_altitude_m"),
                    "cruise_altitude_max_m":     LaunchConfiguration("cruise_altitude_max_m"),
                    "default_survey_radius_m":   LaunchConfiguration("default_survey_radius_m"),
                    "survey_radius_max_m":       LaunchConfiguration("survey_radius_max_m"),
                    "max_xy_speed":              LaunchConfiguration("max_xy_speed"),
                    "avoid_known_obstacles":     LaunchConfiguration("avoid_known_obstacles"),
                    "landmarks_enabled":         LaunchConfiguration("landmarks_enabled"),
                }],
            ),
        ],
    )

    voice_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("voice")),
        actions=[
            Node(package="autobee_voice", executable="whisper_stt_node", output="screen"),
            Node(package="autobee_voice", executable="tts_node", output="screen"),
        ],
    )

    try:
        slam_launch_path = os.path.join(
            get_package_share_directory("autobee_slam"), "launch", "orb_slam3.launch.py"
        )
        slam_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                "settings_path": LaunchConfiguration("slam_settings_path"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("slam")),
        )
    except PackageNotFoundError:
        slam_group = GroupAction(
            condition=IfCondition(LaunchConfiguration("slam")),
            actions=[],
        )

    return LaunchDescription([
        voice_arg, slam_arg, slam_settings_path_arg,
        cruise_alt_max, default_cruise_alt, default_radius, radius_max,
        max_xy_speed, max_z_speed, max_yaw_rate,
        linear_max, angular_max,
        avoid_known, perception_backstop, landmarks_enabled,
        tello_driver,
        tello_odometry,
        flight_controller,
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

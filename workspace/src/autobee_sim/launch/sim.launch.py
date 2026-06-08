"""Launch the AutoBee campus world with the drone spawned and ros_gz_bridge wired."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("autobee_sim")
    world_path = os.path.join(pkg_share, "worlds", "campus.sdf")
    drone_sdf = os.path.join(pkg_share, "models", "autobee_drone", "model.sdf")
    bridge_config = os.path.join(pkg_share, "config", "bridge.yaml")

    headless = LaunchConfiguration("headless")
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo in server-only mode (no GUI).",
    )

    models_dir = os.path.join(pkg_share, "models")
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=models_dir,
    )
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=models_dir,
    )

    gz_sim_gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "3", world_path],
        output="screen",
        condition=UnlessCondition(headless),
    )

    gz_sim_headless = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", "-v", "3", world_path],
        output="screen",
        condition=IfCondition(headless),
    )

    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "autobee_drone",
            "-file", drone_sdf,
            "-x", "0", "-y", "0", "-z", "2",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # Sim half of the feedback contract: turn Gazebo ground-truth /drone/pose
    # into the unified /drone/odom (pose + Tello-like body twist). Mirrors
    # autobee_control/tello_odometry_node on the real drone.
    sim_odometry = Node(
        package="autobee_control",
        executable="sim_odometry_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        headless_arg,
        gz_resource_path,
        ign_resource_path,
        gz_sim_gui,
        gz_sim_headless,
        spawn_drone,
        bridge,
        sim_odometry,
    ])

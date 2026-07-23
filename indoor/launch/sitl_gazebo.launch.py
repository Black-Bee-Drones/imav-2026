"""
IMAV 2026 indoor arena on Nectar ArduPilot SITL.

Composes Nectar's indoor vehicle stack with model://imav and a default
spawn before the first gate

Prerequisites:
    Terminal 1:  make sim-start FIRMWARE=ardupilot ENV=indoor  (from nectar-sdk)

Usage:
    ros2 launch indoor sitl_gazebo.launch.py
    ros2 launch indoor sitl_gazebo.launch.py spawn_pose:="-6.0 0 0.25 0 0 0"
    ros2 launch indoor sitl_gazebo.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Before first gate (~x=-5.5); inside floor box X in (-7, 7)
_DEFAULT_SPAWN = "-6.2 0 0.25 0 0 0"


def generate_launch_description():
    indoor_share = get_package_share_directory("indoor")
    models_dir = os.path.join(indoor_share, "simulation", "models")
    nectar_share = get_package_share_directory("nectar")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_pose",
                default_value=_DEFAULT_SPAWN,
                description='Iris pose "x y z roll pitch yaw" (degrees)',
            ),
            DeclareLaunchArgument(
                "mavros",
                default_value="false",
                description="Start MAVROS or Gazebo-only for direct MAVLink",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Gazebo server-only (no GUI)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nectar_share, "launch",
                                 "sitl_gazebo.launch.py")
                ),
                launch_arguments={
                    "world": "indoor",
                    "vision": "true",
                    "scenery": "model://imav",
                    "spawn_pose": LaunchConfiguration("spawn_pose"),
                    "resource_path": models_dir,
                    "mavros": LaunchConfiguration("mavros"),
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
        ]
    )

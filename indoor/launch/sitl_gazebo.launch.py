"""
IMAV 2026 indoor arena on Nectar ArduPilot SITL.

Loads the custom imav_world.sdf directly.
Note: spawn_pose must be edited directly inside imav_world.sdf when loading a full world.

Prerequisites:
    Terminal 1:  make sim-start FIRMWARE=ardupilot ENV=indoor  (from nectar-sdk)

Usage:
    ros2 launch indoor sitl_gazebo.launch.py
    ros2 launch indoor sitl_gazebo.launch.py mavros:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nectar_share = get_package_share_directory("nectar")
    indoor_share = get_package_share_directory("indoor")
    
    imav_world_path = os.path.join(indoor_share, "simulation", "models", "imav", "imav_world.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mavros",
                default_value="false",
                description="Start MAVROS (true) or leave Gazebo-only for direct pymavlink (false)",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Gazebo server-only (no GUI)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nectar_share, "launch", "sitl_gazebo.launch.py")
                ),
                launch_arguments={
                    "world": imav_world_path,
                    "vision": "true",
                    "spawn_pose": "-6.0 3.0 0.25 0 0 0", 
                    "mavros": LaunchConfiguration("mavros"),
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
        ]
    )
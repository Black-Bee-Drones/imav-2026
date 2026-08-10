"""
IMAV 2026 indoor arena on Nectar ArduPilot SITL.

Loads the custom imav_world.sdf directly (full world with iris + arena).
vision:=true is required because world is a path, not world:=indoor.
resource_path points at simulation/models so model://imav meshes resolve.

Note: spawn pose is edited inside imav_world.sdf (path worlds ignore spawn_pose).

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

    models_dir = os.path.join(indoor_share, "simulation", "models")
    imav_world_path = os.path.join(models_dir, "imav", "imav_world.sdf")

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
                    "resource_path": models_dir,
                    "mavros": LaunchConfiguration("mavros"),
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
        ]
    )

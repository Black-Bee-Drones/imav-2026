import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='indoor',
            executable='mangalarga',
            name='mangalarga',
            parameters=[
                os.path.join(get_package_share_directory('indoor'), 'config', 'default.yaml'),
                os.path.join(get_package_share_directory('indoor'), 'config', 'simulation.yaml'),
            ],
            output='screen',
        ),
    ])

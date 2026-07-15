from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='indoor',
            executable='mangalarga',
            name='mangalarga',
            parameters=[
                {
                    # Drone
                    'drone_type': 'mavlink',
                    'connection_string': 'tcp://127.0.0.1:5762',
                    'image_source': '/front_camera/image',

                    # Camera
                    'front_image_source': 'ros',
                    'front_ros_topic': '/front_camera/image',
                    'down_image_source': 'ros',
                    'down_ros_topic': '/donw_camera/image',
                }
            ],
            output='screen',
        ),
    ])

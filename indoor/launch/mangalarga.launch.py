from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='default',
        description='Configuration profile to load (e.g., stil, cleitinho, jorge)'
    )

    mangalarga_node = Node(
        package='indoor',
        executable='mangalarga',
        name='mangalarga',
        output='screen',
        arguments=['--config', LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        mangalarga_node
    ])
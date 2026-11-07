"""Launch the titania_extender driver node with its YAML configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('titania_extender'),
                'config',
                'titania_extender.yaml',
            ]),
            description='Path to the titania_extender YAML configuration.',
        ),
        Node(
            package='titania_extender',
            executable='titania_extender_node',
            name='titania_extender_node',
            output='screen',
            parameters=[config_file],
        ),
    ])

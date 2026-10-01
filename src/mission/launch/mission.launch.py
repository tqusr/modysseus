from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument('saga_token', default_value=''),
        Node(
            package='mission',
            executable='stub_action_servers',
            name='stub_action_servers',
            output='screen',
        ),
        Node(
            package='mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                'saga_url': LaunchConfiguration('saga_url'),
                'saga_token': LaunchConfiguration('saga_token'),
            }],
        ),
    ])

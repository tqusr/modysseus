from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    return [
        # stub_action_servers is NOT launched here: it registers action servers
        # under the SAME names (`navigate_to_pose`, `dock_robot`) that the real
        # Nav2 bt_navigator and tag_dock_server advertise, and ROS2 silently
        # allows two nodes to advertise one action name with non-deterministic
        # goal routing. It is a test double for the mission tree's tests.
        Node(
            package='mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                # Performed to text so a numeric-looking token stays a string.
                'saga_url': LaunchConfiguration('saga_url').perform(context),
                'saga_token': LaunchConfiguration('saga_token').perform(context),
                # VehicleStatusOK/Localize compare TF stamps against this
                # node's own clock, so it must run in the same time domain as
                # the rest of the stack: wall clock, there is no /clock on the
                # robot. A bool, not text.
                'use_sim_time': False,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument('saga_token', default_value=''),
        OpaqueFunction(function=_launch_nodes),
    ])

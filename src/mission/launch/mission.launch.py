from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    """Launch arguments are text; node parameters must be typed."""
    return text.strip().lower() in ('true', '1')


def _launch_nodes(context):
    # Optional parameter file, e.g. config/health_hardware.yaml: the topics
    # VehicleStatusOK requires to keep arriving. Empty = no topic checks.
    health_config = LaunchConfiguration('health_config').perform(context)
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
            parameters=([health_config] if health_config else []) + [{
                # Performed to text so a numeric-looking token stays a string.
                'saga_url': LaunchConfiguration('saga_url').perform(context),
                'saga_token': LaunchConfiguration('saga_token').perform(context),
                # VehicleStatusOK/Localize compare TF stamps against this
                # node's own clock, so it must run in the same time domain as
                # the rest of the stack: wall clock, there is no /clock on the
                # robot. A bool, not text.
                'use_sim_time': False,
                'safe_stop_zero_hold': _as_bool(
                    LaunchConfiguration('safe_stop_zero_hold').perform(context)),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument('saga_token', default_value=''),
        DeclareLaunchArgument('health_config', default_value=''),
        DeclareLaunchArgument(
            'safe_stop_zero_hold', default_value='true',
            description='false = safe stop does not publish zero velocity '
                        '(diagnostic only)'),
        OpaqueFunction(function=_launch_nodes),
    ])

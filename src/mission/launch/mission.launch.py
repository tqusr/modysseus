from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    """Launch arguments are text; node parameters must be typed."""
    return text.strip().lower() in ('true', '1')


def _launch_nodes(context):
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    # Optional parameter file, e.g. config/health_hardware.yaml: the topics
    # VehicleStatusOK requires to keep arriving. Empty = no topic checks.
    health_config = LaunchConfiguration('health_config').perform(context)
    return [
        # stub_action_servers is NOT launched here anymore (Task 7). It used to
        # stand in for NavigateToPose/DockRobot, but it registers action servers
        # under the SAME names (`navigate_to_pose`, `dock_robot`) that the real
        # Nav2 bt_navigator (Task 6, navigation/launch/navigation.launch.py) and
        # the real opennav_docking docking_server (Task 5,
        # perception/launch/docking.launch.py) now advertise. ROS2 silently
        # allows two nodes to advertise the same action name with non-
        # deterministic goal routing between them, which Task 5's review
        # flagged as a real collision risk if both ever ran together. The stub
        # code itself is left in place (mission/stub_action_servers.py, still
        # covered by test_stub_action_servers.py) for isolated mission-tree
        # testing/development without the full real stack running -- run it
        # manually (`ros2 run mission stub_action_servers`) for that, never
        # alongside the real servers.
        Node(
            package='mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=([health_config] if health_config else []) + [{
                # Performed to text so a numeric-looking token stays a string.
                'saga_url': LaunchConfiguration('saga_url').perform(context),
                'saga_token': LaunchConfiguration('saga_token').perform(context),
                # Required since Task 7: VehicleStatusOK/Localize compare TF
                # stamps against this node's own clock, so it must run in the same
                # time domain as the rest of the stack (the launch argument
                # use_sim_time, default true = sim /clock). In simulation,
                # without this mission_node's wall clock is a different time
                # domain from the sim-clock TF stamps, which made every TF
                # recency check fail permanently (confirmed against the real
                # stack, not assumed). The real robot has no /clock, so
                # hardware passes use_sim_time:=false for every node.
                'use_sim_time': use_sim_time,
                'safe_stop_zero_hold': _as_bool(
                    LaunchConfiguration('safe_stop_zero_hold').perform(context)),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument('saga_token', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('health_config', default_value=''),
        DeclareLaunchArgument(
            'safe_stop_zero_hold', default_value='true',
            description='false = safe stop does not publish zero velocity '
                        '(diagnostic only)'),
        OpaqueFunction(function=_launch_nodes),
    ])

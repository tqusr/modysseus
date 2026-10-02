from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument('saga_token', default_value=''),
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
            parameters=[{
                'saga_url': LaunchConfiguration('saga_url'),
                'saga_token': LaunchConfiguration('saga_token'),
                # Required since Task 7: VehicleStatusOK/Localize compare TF
                # stamps (sim-clock, since every other node in this stack runs
                # with use_sim_time:true) against this node's own clock. Without
                # this, mission_node's clock is real wall-clock -- a completely
                # different time domain from the sim-clock TF stamps -- which
                # made every TF recency check fail permanently when confirmed
                # against the real stack (not assumed).
                'use_sim_time': True,
            }],
        ),
    ])

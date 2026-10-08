"""Terminal 2: docking and mission, with operator-facing logs.

    ros2 launch mission mission.launch.py saga_token:=<token>

Runs next to `hardware.launch.py` (terminal 1: odometry, SLAM, Nav2) and
starts `docking_server`, `tag_dock_server` (/dock_robot, /undock_robot) and
`mission_node`. Nav2 need not be up first: the mission waits and says so.
Typical log lines:

    [route] 12: pick up at A, deliver to F
    [nav] going to the pre-dock pose of dock A (1.20, -0.30)
    [dock] docking at A
    dock A: obstacle in the bay (...), waiting
    dock A: the pre-dock pose is taken; ... navigating straight to the dock pose with Nav2
    [mission] waiting: Nav2 lifecycle manager does not report active

Arguments:
    saga_url: Saga server URL (default http://localhost:8000).
    saga_token: Saga bearer token.
    mission: `false` starts only the docking servers, for testing docking
        without Saga (default true).
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_nodes(context):
    """Returns the docking include and, unless `mission:=false`, mission_node."""
    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('perception'), 'launch', 'docking.launch.py'])))
    run_mission = LaunchConfiguration('mission').perform(context).strip().lower() in (
        'true', '1')
    return [
        docking,
        # stub_action_servers is deliberately not launched: it would advertise
        # the same action names as the real servers, and ROS 2 allows that
        # silently with non-deterministic routing. It is a test double only.
        *([Node(
            package='mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                # Performed to text so a numeric-looking token stays a string.
                'saga_url': LaunchConfiguration('saga_url').perform(context),
                'saga_token': LaunchConfiguration('saga_token').perform(context),
                # Wall clock like the rest of the stack: the TF checks compare
                # stamps against this node's clock. A bool, not text.
                'use_sim_time': False,
            }],
        )] if run_mission else []),
    ]


def generate_launch_description():
    """Declares the arguments and defers node creation to `_launch_nodes`."""
    return LaunchDescription([
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument('saga_token', default_value=''),
        DeclareLaunchArgument('mission', default_value='true',
                              description='false = docking servers only, no mission_node'),
        OpaqueFunction(function=_launch_nodes),
    ])

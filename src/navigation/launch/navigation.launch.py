"""Bring up the Nav2 stack (costmaps, planner, controller, safety chain).

Structurally based on nav2_bringup's own navigation_launch.py
(/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py), trimmed to
what this task actually needs: no AMCL (slam_toolbox's localize.launch.py,
Task 3, already publishes map->odom - a second localizer would fight it for
the same TF edge), no composition (this stack is small enough that per-
process nodes are simpler to reason about and restart individually), no
multi-robot namespacing, no route_server/smoother_server/waypoint_follower/
docking_server (not needed for a bare NavigateToPose - see nav2_params.yaml's
own header comment for why).

cmd_vel chain (the "safety chain" the task brief asks for), matching the
stock bringup's own wiring:
    controller_server --(cmd_vel, remapped)--> cmd_vel_nav
    velocity_smoother  --(subscribes cmd_vel_nav, publishes cmd_vel_smoothed)
    collision_monitor  --(cmd_vel_in_topic=cmd_vel_smoothed, cmd_vel_out_topic=cmd_vel)--> /cmd_vel
behavior_server's own cmd_vel (used by Spin/BackUp/DriveOnHeading) is
remapped the same way, so recovery behaviors go through the same smoother
and collision monitor as normal path following - nothing bypasses the chain
to reach the robot directly.

Lifecycle: nav2_lifecycle_manager is Nav2's own purpose-built, widely-used
manager for exactly this multi-node bring-up - unlike the vendor event-based
launch activation that bit slam_toolbox hard in Tasks 2/3, this is not an
ad-hoc launch-file event handler. Verify with
`ros2 service call /lifecycle_manager_navigation/is_active \
std_srvs/srv/Trigger` or `ros2 service call /<node>/get_state
lifecycle_msgs/srv/GetState` per node, same as it's verified in this task's
report - do not assume activation succeeded just because the manager itself
started cleanly.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
    'collision_monitor',
]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('navigation'), 'config', 'nav2_params.yaml')

    # Unbuffered stdout, matching nav2_bringup's own launch files, so node
    # logs interleave correctly when backgrounded/redirected to a file.
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    common_args = {
        'output': 'screen',
        'parameters': [config, {'use_sim_time': True}],
    }

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        **common_args,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        **common_args,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        **common_args,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        **common_args,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        **common_args,
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        **common_args,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': LIFECYCLE_NODES,
            # Generous: bt_navigator/controller_server can take a while to
            # come up on a loaded machine (same reasoning as Task 2/3's
            # watchdog timeout).
            'bond_timeout': 10.0,
        }],
    )

    return LaunchDescription([
        stdout_linebuf_envvar,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager,
    ])

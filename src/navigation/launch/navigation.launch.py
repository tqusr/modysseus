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
and collision monitor as normal path following - nothing in THIS launch file
bypasses the chain to reach the robot directly.

ONE KNOWN, DELIBERATE EXCEPTION, OUTSIDE THIS FILE: `docking_server`
(perception/launch/docking.launch.py, opennav_docking) publishes /cmd_vel
DIRECTLY during docking and undocking legs - it does not go through this
velocity smoother or this collision monitor. So while a docking or undocking
action is running, the chain above is not in the loop at all, and the
polygons configured for collision_monitor in nav2_params.yaml are not
protecting the robot. Collision protection during those legs comes solely
from docking_server's own `controller.use_collision_detection`
(perception/config/docking.yaml), enabled in the final-review fix wave,
which checks its projected approach trajectory against this stack's
local_costmap. Rerouting docking through this chain was considered and
rejected as too risky to the proven live delivery routes: the stop polygon
would trip on the dock's own back wall on every approach. See
docking.launch.py's own docstring for the full rationale.

Lifecycle: nav2_lifecycle_manager is Nav2's own purpose-built, widely-used
manager for exactly this multi-node bring-up, rather than an ad-hoc
launch-file event handler like the vendor event-based activation that bit
slam_toolbox hard in Tasks 2/3.

That does NOT make it immune to the same underlying failure class, and an
earlier version of this docstring wrongly implied it did. The failure is in
the transport, not in who drives the transition: this environment drops
`change_state` service RESPONSES under load (Nav2's own manager logs
"failed to send response to /planner_server/change_state (timeout): client
will not receive response"), and a manager waiting on a response that was
lost never completes bring-up. OBSERVED HERE, not theorised: 1 of 4
full-stack launches during the final-review fix wave wedged on exactly that
signature and never reached `active`; the documented kill-and-relaunch got
past it.

So the same mitigation that localization uses (localization/scripts/
slam_lifecycle_watchdog.py - poll the node's state directly and retry,
instead of trusting the response) is warranted for nav2_lifecycle_manager
too. It is deliberately NOT implemented here yet: it is tracked as its own
follow-up item because generalising that watchdog across a 6-node managed
set is real work, not a docstring fix. Until then this is an operational
procedure, not an automatic recovery: if the stack has not logged "Managed
nodes are active" within ~30 s of this launch file starting, kill the whole
launch and relaunch rather than waiting it out.

Verify with
`ros2 service call /lifecycle_manager_navigation/is_active \
std_srvs/srv/Trigger` or `ros2 service call /<node>/get_state
lifecycle_msgs/srv/GetState` per node, same as it's verified in this task's
report - do not assume activation succeeded just because the manager itself
started cleanly.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
    'collision_monitor',
]


def _as_bool(text):
    """Launch arguments are text; node parameters must be typed."""
    return text.strip().lower() in ('true', '1')


def _launch_nodes(context):
    # Default true (simulation); the real robot passes false.
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    config = LaunchConfiguration('params_file').perform(context) or os.path.join(
        get_package_share_directory('navigation'), 'config', 'nav2_params.yaml')

    # Unbuffered stdout, matching nav2_bringup's own launch files, so node
    # logs interleave correctly when backgrounded/redirected to a file.
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # wz_max, when given, replaces every angular-velocity limit in the file at
    # once (MPPI's wz_max, the rotation shim's pivot rate, the smoother's
    # limits), so they cannot disagree. Empty keeps the file's values. The
    # hardware launch uses it with wheel_drive_node, where a commanded rate is
    # the real rate; the file's 1.5 was tuned for the MCU's own kinematics,
    # which turned ~2.3x weaker than commanded.
    wz_text = LaunchConfiguration('wz_max').perform(context).strip()
    wz = float(wz_text) if wz_text else None
    controller_wz = ({'FollowPath.wz_max': wz,
                      'FollowPath.rotate_to_heading_angular_vel': wz}
                     if wz is not None else {})
    # velocity_smoother's arrays carry vx, vy, wz; only wz changes.
    smoother_wz = ({'max_velocity': [0.3, 0.0, wz],
                    'min_velocity': [-0.15, 0.0, -wz]}
                   if wz is not None else {})

    common_args = {
        'output': 'screen',
        'parameters': [config, {'use_sim_time': use_sim_time}],
    }

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time, **controller_wz}],
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

    # Replan only when the path is blocked or the goal changes; see the
    # bt_navigator comment in nav2_params.yaml.
    nav_to_pose_bt = LaunchConfiguration('bt_xml').perform(context) or os.path.join(
        get_package_share_directory('nav2_bt_navigator'), 'behavior_trees',
        'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml')
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time,
                             'default_nav_to_pose_bt_xml': nav_to_pose_bt}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time, **smoother_wz}],
    )

    # cmd_vel_out is where the safety chain ends. Default `cmd_vel` (the
    # vendor driver / the simulator); `cmd_vel_ctrl` hands it to
    # localization's wheel_drive_node instead.
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'cmd_vel_out_topic': LaunchConfiguration('cmd_vel_out').perform(context),
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': LIFECYCLE_NODES,
            # Generous: bt_navigator/controller_server can take a while to
            # come up on a loaded machine (same reasoning as Task 2/3's
            # watchdog timeout).
            'bond_timeout': 10.0,
        }],
    )

    return [
        stdout_linebuf_envvar,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Nav2 parameter file; empty = this package\'s '
                        'config/nav2_params.yaml (for tuning experiments)'),
        DeclareLaunchArgument(
            'cmd_vel_out', default_value='cmd_vel',
            description='Topic the collision monitor publishes the final '
                        'velocity on; cmd_vel_ctrl for wheel_drive_node'),
        DeclareLaunchArgument(
            'wz_max', default_value='',
            description='Override every angular-velocity limit (rad/s); '
                        'empty = the values in nav2_params.yaml'),
        DeclareLaunchArgument(
            'bt_xml', default_value='',
            description='NavigateToPose behavior tree; empty = the "replan only '
                        'if the path becomes invalid" tree'),
        OpaqueFunction(function=_launch_nodes),
    ])

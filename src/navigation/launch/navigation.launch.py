"""Brings up the Nav2 stack: costmaps, planner, controller and safety chain.

Based on nav2_bringup's `navigation_launch.py`, trimmed to a bare
NavigateToPose (see the `nav2_params.yaml` header): no AMCL (slam_toolbox
already publishes map->odom and a second localizer would fight it), no
composition, no namespacing and no route, smoother, waypoint or docking server.

The cmd_vel chain matches the stock bringup:

    controller_server --(cmd_vel remapped)--> cmd_vel_nav
    velocity_smoother: cmd_vel_nav --> cmd_vel_smoothed
    collision_monitor: cmd_vel_smoothed --> cmd_vel_ctrl

`behavior_server` (Spin, BackUp, DriveOnHeading) is remapped the same way, so
nothing in this file bypasses the chain.

One deliberate exception lies outside this file: `docking_server`
(perception/launch/docking.launch.py) publishes /cmd_vel directly during
docking and undocking, skipping the smoother and collision monitor. Protection
there comes only from its `controller.use_collision_detection`
(perception/config/docking.yaml), which checks the projected approach against
the local_costmap. Routing docking through this chain was rejected: the stop
polygon would trip on the dock's own back wall on every approach. See
`docking.launch.py`.

Lifecycle: `nav2_lifecycle_manager` is Nav2's own manager for this bring-up,
but it is not immune to the failure that bit slam_toolbox. This environment
drops `change_state` service responses under load (Nav2 logs "failed to send
response to /planner_server/change_state (timeout)"), and a manager waiting on
a lost response never finishes. Observed: 1 of 4 full-stack launches wedged
this way and never reached `active`. A watchdog like
`localization/scripts/slam_lifecycle_watchdog.py` is warranted but not
implemented; generalising it across the 6 managed nodes is a separate task.
Until then, if "Managed nodes are active" has not been logged within ~30 s,
kill the launch and relaunch.

Verify activation with
`ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger`
or `ros2 service call /<node>/get_state lifecycle_msgs/srv/GetState` per node.
Do not assume it from the manager starting cleanly.
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


def _launch_nodes(context):
    """Returns the Nav2 nodes, their lifecycle manager and the logging env var."""
    # A bool, not text. Wall clock: there is no /clock on the robot.
    use_sim_time = False
    config = LaunchConfiguration('params_file').perform(context) or os.path.join(
        get_package_share_directory('navigation'), 'config', 'nav2_params.yaml')

    # As in nav2_bringup, so logs interleave correctly when redirected.
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # wz_max, if given, replaces every angular-velocity limit at once (MPPI,
    # the rotation shim, the smoother) so they cannot disagree. The hardware
    # launch uses it because wheel_drive_node makes the commanded rate the real
    # one; the file's 1.5 was tuned for the MCU's kinematics, ~2.3x weaker.
    wz_text = LaunchConfiguration('wz_max').perform(context).strip()
    wz = float(wz_text) if wz_text else None
    controller_wz = ({'FollowPath.wz_max': wz,
                      'FollowPath.rotate_to_heading_angular_vel': wz}
                     if wz is not None else {})
    # The smoother's arrays hold vx, vy, wz; only wz changes.
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

    # Replan only when the path is blocked or the goal changes (see nav2_params.yaml).
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

    # The chain ends in cmd_vel_ctrl, which wheel_drive_node turns into wheel speeds.
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[config, {
            'use_sim_time': use_sim_time,
            'cmd_vel_out_topic': 'cmd_vel_ctrl',
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
            # Generous: nodes can be slow to come up on a loaded machine.
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
    """Declares `params_file`, `wz_max` and `bt_xml`, then launches Nav2."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Nav2 parameter file; empty = this package\'s '
                        'config/nav2_params.yaml (for tuning experiments)'),
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

"""Bring up opennav_docking's real `DockRobot` action server.

Replaces `mission`'s `stub_action_servers` DockRobot stub with the real
`opennav_docking` package's `docking_server` node, configured against the
real arena dock poses (`config/docking.yaml`, itself hand-transcribed from
`mission/config/docks.yaml` -- see that file's header comment for the
staging-pose formula and why fixed-pose docking was chosen over a vision-
corrected AprilTag approach for this iteration).

KNOWN, DELIBERATE EXCEPTION TO THE "COLLISION MONITOR IS ALWAYS LAST" RULE.

Everywhere else in this stack, a velocity command reaches the robot only
through controller -> velocity_smoother -> nav2_collision_monitor ->
`/cmd_vel` (see navigation/launch/navigation.launch.py, which remaps even
the recovery behaviours' `cmd_vel` into that chain so nothing bypasses it).
`docking_server` does NOT: it publishes `/cmd_vel` directly for the whole
docking and undocking leg, bypassing both the velocity smoother and the
standalone collision monitor.

That is accepted on purpose, not an oversight. Rerouting docking's output
through the chain was considered in the final-review fix wave and rejected
as the higher-risk option against two proven live delivery routes: the
collision monitor's stop polygon (+-0.25 m around base_footprint, see
navigation/config/nav2_params.yaml) would fire on the dock's own back wall
during every single approach, and the smoother's limits would fight the
docking controller's own pose-regulation profile.

Collision protection during docking/undocking therefore comes from exactly
one place: `docking_server`'s own `controller.use_collision_detection`
(config/docking.yaml), which is now ENABLED and checks the projected
approach trajectory against Nav2's local costmap, with
`dock_collision_threshold` carving out the dock structure itself. If that
parameter is ever turned off again, docking legs run with no obstacle
protection whatsoever.

LIFECYCLE -- same issue class as Tasks 2/3's `slam_toolbox` nodes, confirmed
(not assumed) before writing this file: `ros2 node info` against a running
`opennav_docking` process shows `/docking_server/change_state` and
`/docking_server/get_state` services, i.e. `opennav_docking::DockingServer`
is a `nav2_util::LifecycleNode` (it subclasses `rclcpp_lifecycle::LifecycleNode`)
like every other Nav2 server -- it does nothing (the `dock_robot` action
server isn't even active) until CONFIGUREd and ACTIVATEd. This environment
has shown `change_state` responses go missing for lifecycle nodes before
(slam_toolbox, Tasks 2/3), so this launch reuses the exact same proven
two-layer approach as `localization/launch/localize.launch.py`:

1. A fast-path `OnStateTransition` event handler that reacts the instant the
   real `configuring -> inactive` transition is observed, firing `ACTIVATE`
   immediately.
2. `slam_lifecycle_watchdog.py` (from the `localization` package -- it is
   node-name-agnostic, taking only a node name and timeout as argv, so it
   works unmodified here) as the actual authority: it polls `get_state`
   directly and re-issues whatever transition is needed, so a lost
   `change_state` response just means the next poll retries, and it fails
   loudly (ERROR + non-zero exit) if `docking_server` never reaches `active`.

Do not trust the vendor's own lifecycle-manager pattern (`nav2_lifecycle_manager`)
or a bare `LifecycleNode` with no activation path here without the watchdog --
see the lesson in this repo's task briefs: it is exactly the race that cost
real time in Tasks 2/3.

The `use_sim_time` launch argument (default true) exists for the real robot,
which runs on wall-clock time.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent
from lifecycle_msgs.msg import Transition

#: Generous, same rationale as localize.launch.py: only bounds how long we
#: wait before declaring the run dead.
ACTIVATION_TIMEOUT = 60.0


def _as_bool(text):
    """Launch arguments are text; node parameters must be typed."""
    return text.strip().lower() in ('true', '1')


def _launch_nodes(context):
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'docking.yaml')

    docking_node = LifecycleNode(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        namespace='',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        # tag_dock_server answers /dock_robot (measure the tag, align, then
        # call this server); this server's own docking action moves aside.
        # /undock_robot is unchanged.
        remappings=[('dock_robot', 'docking_server/dock_robot')],
    )

    # /dock_robot: measures the requested dock's AprilTag on the way in, aligns
    # square to it and calls docking_server (/docking_server/dock_robot) with
    # the measured pose. Reads its own
    # `tag_dock_server:` block and the dock list from the same docking.yaml.
    tag_dock_server = Node(
        package='perception',
        executable='tag_dock_server',
        name='tag_dock_server',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time, 'docking_config': config}],
    )

    def change_state(transition_id):
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(docking_node),
            transition_id=transition_id,
        ))

    # Fast path: react to the real transition the instant it is observed.
    activate_on_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=docking_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[change_state(Transition.TRANSITION_ACTIVATE)],
    ))

    # Authority: polls get_state, retries as needed, and fails loudly.
    watchdog = ExecuteProcess(
        cmd=['ros2', 'run', 'localization', 'slam_lifecycle_watchdog.py',
             '/docking_server', str(ACTIVATION_TIMEOUT)],
        output='screen',
    )

    report = RegisterEventHandler(OnProcessExit(
        target_action=watchdog,
        on_exit=lambda event, context: [LogInfo(msg=(
            'docking_server activation CONFIRMED - DockRobot is live.'
            if event.returncode == 0 else
            'ERROR: docking_server never reached "active". The DockRobot '
            'action server is NOT running. Kill this launch, check the '
            'docking_server log above, and retry.'
        ))],
    ))

    return [
        docking_node,
        tag_dock_server,
        activate_on_configured,
        TimerAction(period=3.0, actions=[change_state(Transition.TRANSITION_CONFIGURE)]),
        TimerAction(period=4.0, actions=[watchdog]),
        report,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_launch_nodes),
    ])

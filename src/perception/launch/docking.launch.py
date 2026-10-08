"""Bring up opennav_docking's real `DockRobot` action server.

Runs the `opennav_docking` package's `docking_server` node, configured against
the real arena dock poses (`config/docking.yaml`, itself hand-transcribed from
`mission/config/docks.yaml` -- see that file's header comment for the
staging-pose formula and why fixed-pose docking was chosen over a vision-
corrected AprilTag approach for this iteration).

EVERY DOCKING MOTION GOES THROUGH THE COLLISION MONITOR.

Everywhere in this stack a velocity command reaches the robot only through
controller/behavior_server -> `cmd_vel_nav` -> velocity_smoother ->
nav2_collision_monitor -> `/cmd_vel_ctrl` (navigation/launch/navigation.launch.py),
so the lidar and camera stop/slowdown zones guard docking and undocking too:
an obstacle can appear in a dock bay like anywhere else.

  - tag_dock_server (`/dock_robot`, `/undock_robot`, what the mission uses)
    publishes no velocity itself. It moves the robot only with Nav2's own
    NavigateToPose (to the pre-dock pose, then turned round onto the staging
    pose) and docking_server (which REVERSES the robot in, at the slow speed
    limits in docking.yaml), all inside that chain.
  - docking_server (explicit-pose goals passed through, and its own
    `/docking_server/*` actions) publishes `cmd_vel`, remapped below into
    `cmd_vel_nav`. It used to publish `/cmd_vel` directly, a deliberate
    exception from when the stop polygon was +-0.25 m and fired on the dock's
    back wall. The stop square is now +-0.15 m and tag docking ends with the
    front edge 0.15 m from the wall (0.273 m from base_footprint), so the
    wall is outside it; the wall is inside the 0.30 m slowdown circle, which
    only slows the last few cm. An explicit-pose goal that drives
    docking_server closer than ~3 cm to a wall WILL be stopped by the monitor:
    that is the point.

docking_server's own `controller.use_collision_detection` (config/
docking.yaml) stays on as a second, trajectory-ahead check against the local
costmap.

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

Every node runs on wall-clock time: there is no /clock on the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent
from lifecycle_msgs.msg import Transition

#: Generous, same rationale as localize.launch.py: only bounds how long we
#: wait before declaring the run dead.
ACTIVATION_TIMEOUT = 60.0


def _launch_nodes(context):
    # A bool, not text: rclpy declares `use_sim_time` as a bool.
    use_sim_time = False
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'docking.yaml')

    docking_node = LifecycleNode(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        namespace='',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        # tag_dock_server answers /dock_robot and /undock_robot (Nav2 to the
        # pre-dock pose, wait while the bay is occupied, measure the tag and
        # save the pose, turn round, then call this server to back in); this server's own
        # actions move aside and serve its explicit-pose goals.
        # cmd_vel -> cmd_vel_nav: into the velocity smoother and collision
        # monitor like every other motion (see the docstring above).
        remappings=[('dock_robot', 'docking_server/dock_robot'),
                    ('undock_robot', 'docking_server/undock_robot'),
                    ('cmd_vel', 'cmd_vel_nav')],
    )

    # /dock_robot + /undock_robot: Nav2 to the requested dock's pre-dock pose,
    # waits (without limit) while another robot or an obstacle is in the bay,
    # measures the dock's AprilTag, saves the measured pose, and has
    # docking_server drive in. Undocking backs out again. Reads its own
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
        OpaqueFunction(function=_launch_nodes),
    ])

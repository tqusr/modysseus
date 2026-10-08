"""Brings up opennav_docking's `DockRobot` action server and `tag_dock_server`.

Runs `docking_server` from `opennav_docking`, configured with the real arena dock
poses in `config/docking.yaml` (hand-transcribed from `mission/config/docks.yaml`;
see that file's header for the staging-pose formula).

Every docking motion goes through the collision monitor. Velocity commands
reach the robot only via controller or behavior_server -> `cmd_vel_nav` ->
velocity_smoother -> nav2_collision_monitor -> `/cmd_vel_ctrl`
(navigation/launch/navigation.launch.py), so the lidar and camera stop and
slowdown zones guard docking and undocking too.

  - `tag_dock_server` (`/dock_robot`, `/undock_robot`, used by the mission)
    publishes no velocity. It moves the robot with Nav2's NavigateToPose (to
    the pre-dock pose, then turned onto the staging pose) and docking_server
    (which reverses the robot in at the slow limits in docking.yaml), all inside
    that chain.
  - `docking_server` (explicit-pose goals passed through, and its own
    `/docking_server/*` actions) publishes `cmd_vel`, remapped below into
    `cmd_vel_nav`. It used to publish `/cmd_vel` directly because the stop
    polygon (+-0.25 m) fired on the dock's back wall. The stop square is now
    +-0.15 m and tag docking ends with the front edge 0.15 m from the wall
    (0.273 m from base_footprint), outside it; the wall is inside the 0.30 m
    slowdown circle, which only slows the last few cm. An explicit-pose goal
    that drives closer than ~3 cm to a wall will be stopped by the monitor.

docking_server's `controller.use_collision_detection` (config/docking.yaml)
stays on as a second, trajectory-ahead check against the local costmap.

Lifecycle: `opennav_docking::DockingServer` is a `nav2_util::LifecycleNode`
(`/docking_server/change_state` and `get_state` exist, confirmed with
`ros2 node info`) and does nothing until configured and activated. Because
`change_state` responses go missing in this environment (as with slam_toolbox),
this launch reuses the two-layer approach of
`localization/launch/localize.launch.py`:

1. A fast-path `OnStateTransition` handler fires ACTIVATE as soon as
   `configuring -> inactive` is observed.
2. `slam_lifecycle_watchdog.py` (from `localization`; it takes only a node name
   and timeout) is the authority. It polls `get_state`, re-issues transitions,
   and fails loudly (ERROR, non-zero exit) if `docking_server` never reaches
   `active`.

Do not rely on `nav2_lifecycle_manager` or a bare `LifecycleNode` without the
watchdog.

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

# Generous, as in localize.launch.py: bounds the wait before declaring the run dead.
ACTIVATION_TIMEOUT = 60.0


def _launch_nodes(context):
    """Returns docking_server, tag_dock_server and the activation machinery."""
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
        # tag_dock_server answers /dock_robot and /undock_robot and calls this
        # server to back in, so this server's own actions move aside and serve
        # explicit-pose goals. cmd_vel goes into the velocity smoother and
        # collision monitor like every other motion.
        remappings=[('dock_robot', 'docking_server/dock_robot'),
                    ('undock_robot', 'docking_server/undock_robot'),
                    ('cmd_vel', 'cmd_vel_nav')],
    )

    # /dock_robot and /undock_robot. Reads its `tag_dock_server:` block and the
    # dock list from the same docking.yaml.
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

    # Fast path: activate as soon as the transition is observed.
    activate_on_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=docking_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[change_state(Transition.TRANSITION_ACTIVATE)],
    ))

    # Authority: polls get_state, retries and fails loudly.
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
    """Returns the docking launch description."""
    return LaunchDescription([
        OpaqueFunction(function=_launch_nodes),
    ])

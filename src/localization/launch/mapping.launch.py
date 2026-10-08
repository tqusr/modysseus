"""Runs slam_toolbox in mapping mode on /scan_filtered (antennas removed).

One-time data collection: drive the robot around the arena while this runs,
then save the map with nav2_map_server's map_saver_cli. `localize.launch.py`
is the long-running counterpart that consumes the saved map.

Runs on wall-clock time (no /clock on the robot) with
`mapper_params_hardware.yaml`.

Uses `sync_slam_toolbox_node`, not the async one: over a multi-minute run the
async node crashed (unhandled `tf2::NoDataForExtrapolationException`, SIGABRT)
and produced badly distorted maps. The sync node has no async-queue pressure
and was stable.

Lifecycle: `*_slam_toolbox_node` is a managed lifecycle node that processes no
scans until configured and activated. Getting it there is unreliable here and
every failure is silent: launch looks healthy, `/map` is never published, and
the saved map comes back empty. Two failures seen in this repo:

1. slam_toolbox's `online_sync_launch.py` emits CONFIGURE and waits on an
   `OnStateTransition(configuring -> inactive)` handler to emit ACTIVATE. The
   node's `change_state` response is often lost ("failed to send response to
   /slam_toolbox/change_state (timeout)"), launch never sees the transition
   and the node stays inactive.
2. Fixed timers (CONFIGURE at 3 s, ACTIVATE at 10 s) are not chained to
   CONFIGURE's result. Under this stack's sustained load a slow CONFIGURE
   makes ACTIVATE fire too early and get rejected.

So this file uses three mechanisms and assumes none of them worked:

- CONFIGURE on a short timer (the node must exist first).
- An `OnStateTransition` handler that emits ACTIVATE once the node is seen
  `inactive`. This is the fast path, but failure 1 breaks it.
- `slam_lifecycle_watchdog.py`, the authority. It polls `get_state` every
  second, re-issues the transition the observed state calls for and never
  waits on a `change_state` response, so it survives both failures. If the
  node is not `active` by the timeout it logs an ERROR and exits non-zero,
  which this launch surfaces as its own ERROR. Wait for "slam_toolbox is
  ACTIVE" before trusting a mapping run.

Redundant ACTIVATEs are harmless: against an active node they log "No
transition matching 3 found for current state active" and change nothing.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (EmitEvent, ExecuteProcess, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# Generous: bounds the wait before declaring the run dead; a loaded machine
# can take a while to initialise Ceres.
ACTIVATION_TIMEOUT = 60.0


def generate_launch_description():
    """Returns the scan filter, slam_toolbox and the activation machinery."""
    config = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'mapper_params_hardware.yaml',
    )

    # Removes the robot's antennas from /scan (config/scan_filter.yaml).
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('localization'),
                         'config', 'scan_filter.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[config, {
            'use_lifecycle_manager': False,
            'use_sim_time': False,
        }],
    )

    def change_state(transition_id):
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=transition_id,
        ))

    # Fast path: activate as soon as the transition is observed.
    activate_on_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[change_state(Transition.TRANSITION_ACTIVATE)],
    ))

    # Authority: polls get_state, retries and fails loudly.
    watchdog = ExecuteProcess(
        cmd=['ros2', 'run', 'localization', 'slam_lifecycle_watchdog.py',
             '/slam_toolbox', str(ACTIVATION_TIMEOUT)],
        output='screen',
    )

    report = RegisterEventHandler(OnProcessExit(
        target_action=watchdog,
        on_exit=lambda event, context: [LogInfo(msg=(
            'slam_toolbox activation CONFIRMED - mapping is live.'
            if event.returncode == 0 else
            'ERROR: slam_toolbox never reached "active". It is processing NO '
            'scans; any map saved from this run will be empty or partial. '
            'Kill this launch, check the slam_toolbox log above, and retry.'
        ))],
    ))

    return LaunchDescription([
        scan_filter,
        slam_node,
        activate_on_configured,
        TimerAction(period=3.0, actions=[change_state(Transition.TRANSITION_CONFIGURE)]),
        TimerAction(period=4.0, actions=[watchdog]),
        report,
    ])

"""Run slam_toolbox in mapping mode against /scan_filtered (antennas removed).

One-time data-collection launch: drive the robot around the arena while this
runs, then save the resulting map with nav2_map_server's map_saver_cli. Task
3's localization launch is the long-running counterpart that consumes the
saved map instead of building one.

SIM OR HARDWARE, decided by the `use_sim_time` argument. It defaults to true,
because the simulator is what this file was written against and what every
existing caller means; `mapping_mission_hardware.launch.py` is the one that
passes false. The argument exists at all because
`mapper_params_online_async.yaml` hardcodes `use_sim_time: true`, and a node
left at true with no /clock publisher sits at time zero forever and looks
exactly like a hang (the same trap common.launch.py's own `use_sim_time`
comment describes in the vendor repo). The dict passed after the config file
below overrides that YAML key - launch applies parameter sources in order and
the last one wins - so the YAML can stay as it is while this launch still
decides the clock.

Nothing else in this file differs between the two modes: the lifecycle dance,
the watchdog and the sync-vs-async choice are all about this node, not about
where its scans come from.

Uses `sync_slam_toolbox_node`, not the async variant: this sim has two
ros_gz_bridge publishers on /clock, producing small time jumps. Over a
multi-minute mapping run, `async_slam_toolbox_node` proved unreliable against
that jitter in testing - observed failures included outright crashes
(unhandled `tf2::NoDataForExtrapolationException`, SIGABRT) and severely
distorted maps. `sync_slam_toolbox_node` processes scans synchronously with
no async-queue pressure, which fits this one-time offline mapping pass fine,
and proved stable where async did not.

LIFECYCLE - read this before adapting this launch file for Task 3.

`*_slam_toolbox_node` is a managed lifecycle node: it processes no scans at
all until CONFIGUREd and then ACTIVATEd. Getting it there is genuinely
unreliable here, and every failure mode is silent - the launch looks healthy,
`/map` is simply never published, and you only find out when a saved map
comes back empty. Two distinct ways it has actually failed in this repo:

1. slam_toolbox's shipped `online_sync_launch.py` emits CONFIGURE and then
   waits on an `OnStateTransition(configuring -> inactive)` handler to emit
   ACTIVATE. In this environment the node's `change_state` response is
   regularly lost ("failed to send response to /slam_toolbox/change_state
   (timeout)"), launch never observes the transition, the handler never
   fires, and the node sits inactive for the entire run.
2. Replacing that with fixed wall-clock timers (CONFIGURE at 3s, ACTIVATE at
   10s) swaps one silent failure for another: the timers are independent, not
   chained off CONFIGURE's actual result, so under the sustained ~15/16-core
   load this sim stack produces, a slow CONFIGURE (node allocation, Ceres
   solver init, parameter loading) means every ACTIVATE fires too early, gets
   rejected, and the node again sits inactive.

So this file uses three mechanisms, and critically does not assume any of
them worked:

- CONFIGURE on a short timer (the node must exist before it can be asked).
- An `OnStateTransition` handler that emits ACTIVATE the moment the node is
  actually observed reaching `inactive`. This is the fast path, and it is
  correctly gated on the real transition - but it is the mechanism failure
  mode 1 breaks, so it cannot be relied on alone.
- `slam_lifecycle_watchdog.py`, which is the authority. It polls `get_state`
  on a 1s interval, re-issues whatever transition the observed state calls
  for, and never waits on a `change_state` response (a lost one just means
  the next poll retries). It is therefore immune to both failure modes above,
  it has no assumed window for CONFIGURE to finish within, and it FAILS
  LOUDLY: if the node has not reached `active` within the timeout it logs an
  ERROR saying so and exits non-zero, which this launch file surfaces again
  as its own ERROR. Watch for "slam_toolbox is ACTIVE" in the launch output
  before trusting a mapping run.

Redundant ACTIVATEs are harmless - an ACTIVATE against an already-active node
is a no-op that logs "No transition matching 3 found for current state
active", not an error that affects mapping.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

#: Generous: it only bounds how long we wait before declaring the run dead,
#: and a heavily loaded machine can take a while to initialise Ceres.
ACTIVATION_TIMEOUT = 60.0


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'mapper_params_online_async.yaml',
    )

    # Removes the robot's own antennas from /scan; slam_toolbox reads the
    # result. See config/scan_filter.yaml.
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('localization'),
                         'config', 'scan_filter.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
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
            # Overrides the YAML's hardcoded `use_sim_time: true` - parameter
            # sources are applied in order and the last one wins. The bare
            # LaunchConfiguration is enough: launch_ros parses an untyped
            # substitution by YAML rules, so 'true'/'false' arrive as a bool.
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    def change_state(transition_id):
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=transition_id,
        ))

    # Fast path: react to the real transition the instant it is observed.
    activate_on_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[change_state(Transition.TRANSITION_ACTIVATE)],
    ))

    # Authority: polls get_state, retries as needed, and fails loudly.
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
        # Defaulted, unlike the vendor repo's common.launch.py, which
        # deliberately leaves this argument with no default so every entry
        # point has to state its mode. Here the default is the backward
        # compatibility: `ros2 launch localization mapping_mission.launch.py`
        # and every other existing caller means simulation and must keep
        # behaving exactly as before. mapping_mission_hardware.launch.py is
        # the one caller that passes false, and it does so explicitly.
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Take time from /clock. True in '
                                          'simulation, false on hardware.'),
        scan_filter,
        slam_node,
        activate_on_configured,
        TimerAction(period=3.0, actions=[change_state(Transition.TRANSITION_CONFIGURE)]),
        TimerAction(period=4.0, actions=[watchdog]),
        report,
    ])

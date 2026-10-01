"""Run slam_toolbox in localization mode against the saved arena pose graph.

This is Task 3's long-running counterpart to mapping.launch.py: instead of
building a map, it loads the one Task 2 built (src/localization/maps/arena)
and relocalizes the robot against it, publishing `map -> odom` (which,
combined with the sim's own `odom -> base_footprint` and
robot_state_publisher's static `base_footprint -> base_link`, makes
`map -> base_link` resolve too).

MAP FORMAT - read this before pointing this at a different saved map.

`map_file_name` here is NOT the nav2_map_server occupancy-grid pair
(arena.yaml/arena.pgm) that Task 2 produced with map_saver_cli. slam_toolbox's
own `mode: localization` loads a SERIALIZED POSE GRAPH instead, which it reads
as "<map_file_name>.posegraph" + "<map_file_name>.data" - a different,
slam_toolbox-internal (karto) format. Without arena.posegraph/arena.data,
on_configure fails to load a map and this node will not do anything useful -
easy to mistake for the lifecycle-activation race below if you don't check
for it first.

THE RULE, if you ever re-map this arena (not merely a note on how it was done
this time): regenerate `arena.yaml`/`arena.pgm` (map_saver_cli) AND
`arena.posegraph`/`arena.data` (serialize_map) from the SAME live mapping
run, before tearing anything down - call both save services back-to-back
against the one running slam_toolbox instance that just finished driving the
arena. Never update one pair without the other. They are two different
projections of the same pose graph, so a four-way byte-identical-run
guarantee is what makes a downstream consumer (Task 7's Nav2 costmap loads
arena.yaml while this launch's localizer loads arena.posegraph) see the same
world geometry from both. Regenerating one pair from a fresh, independent
mapping run while leaving the other pair in place silently reintroduces two
copies of the arena that only agree by coincidence of start pose - with
`do_loop_closing: false` (no global consistency correction), two independent
passes drift differently even from an identical start, and nothing in either
format's own self-consistency check (TF tree resolves, pixel grid looks
sane) can detect that kind of cross-artifact disagreement.

LIFECYCLE - identical reasoning and fix as mapping.launch.py; read that
file's docstring for the full history. Short version: `*_slam_toolbox_node`
is a managed lifecycle node that processes no scans until CONFIGUREd and then
ACTIVATEd, and in this environment the node's `change_state` response is
regularly lost, so anything that waits on that response (the vendor's own
`localization_launch.py` included) or on the state-change event launch
derives from it can leave the node silently stuck `inactive` forever. The fix
here is the same `slam_lifecycle_watchdog.py` used for mapping: it polls
`get_state` directly, re-issues whatever transition the observed state calls
for without ever waiting on a `change_state` response, and fails loudly (ERROR
log + non-zero exit) if the node does not reach `active` within the timeout.
The watchdog script is mode-agnostic - it only drives generic lifecycle
states by node name, so the exact same script works unmodified here.

Also uses `sync_slam_toolbox_node`'s localization counterpart
(`localization_slam_toolbox_node`) rather than the async variant, for the
same reason mapping.launch.py gives: this sim's dual `/clock` publishers
produce small time jumps that proved less reliable against the async node's
threading model during Task 2's investigation.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (EmitEvent, ExecuteProcess, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

#: Generous: it only bounds how long we wait before declaring the run dead,
#: and a heavily loaded machine can take a while to initialise Ceres.
ACTIVATION_TIMEOUT = 60.0


def generate_launch_description():
    share = get_package_share_directory('localization')
    config = os.path.join(share, 'config', 'localization_params.yaml')
    map_file_name = os.path.join(share, 'maps', 'arena')

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[config, {
            'use_lifecycle_manager': False,
            'use_sim_time': True,
            'map_file_name': map_file_name,
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
            'slam_toolbox activation CONFIRMED - localization is live.'
            if event.returncode == 0 else
            'ERROR: slam_toolbox never reached "active". It is processing NO '
            'scans; map->odom will never be published from this run. Kill '
            'this launch, check the slam_toolbox log above, and retry.'
        ))],
    ))

    return LaunchDescription([
        slam_node,
        activate_on_configured,
        TimerAction(period=3.0, actions=[change_state(Transition.TRANSITION_CONFIGURE)]),
        TimerAction(period=4.0, actions=[watchdog]),
        report,
    ])

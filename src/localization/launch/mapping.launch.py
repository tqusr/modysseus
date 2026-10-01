"""Run slam_toolbox in mapping mode against the sim's /scan topic.

One-time data-collection launch: drive the robot around the arena while this
runs, then save the resulting map with nav2_map_server's map_saver_cli. Task
3's localization launch is the long-running counterpart that consumes the
saved map instead of building one.

Uses `sync_slam_toolbox_node`, not the async variant: this sim has two
ros_gz_bridge publishers on /clock, producing small time jumps. Over a
multi-minute mapping run, `async_slam_toolbox_node` proved unreliable against
that jitter in testing - observed failures included outright crashes
(unhandled `tf2::NoDataForExtrapolationException`, SIGABRT) and severely
distorted maps. `sync_slam_toolbox_node` processes scans synchronously with
no async-queue pressure, which fits this one-time offline mapping pass fine,
and proved stable where async did not.

Lifecycle handling is done here with plain timers rather than by including
slam_toolbox's shipped `online_sync_launch.py`. That launch file emits
CONFIGURE, then relies on an `OnStateTransition(configuring -> inactive)`
event handler to emit ACTIVATE. In this environment the node's `change_state`
service response regularly times out ("failed to send response to
/slam_toolbox/change_state (timeout)"), the transition event is therefore
never observed by launch, the handler never fires, and the node sits in
`inactive` for the entire run - silently processing zero scans. That was
reproduced repeatedly. Firing CONFIGURE and then ACTIVATE on fixed timers
does not depend on the response or the event arriving, and a redundant second
ACTIVATE covers a slow configure (an ACTIVATE emitted when the node is already
active is a no-op failure, not an error that affects mapping).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent, TimerAction
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'mapper_params_online_async.yaml',
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[config, {'use_lifecycle_manager': False, 'use_sim_time': True}],
    )

    def change_state(transition_id):
        return EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=transition_id,
        ))

    return LaunchDescription([
        slam_node,
        TimerAction(period=3.0, actions=[change_state(Transition.TRANSITION_CONFIGURE)]),
        TimerAction(period=10.0, actions=[change_state(Transition.TRANSITION_ACTIVATE)]),
        TimerAction(period=18.0, actions=[change_state(Transition.TRANSITION_ACTIVATE)]),
    ])

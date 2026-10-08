"""Runs slam_toolbox in localization mode against the saved arena pose graph.

The long-running counterpart to `mapping.launch.py`: it loads the map built
earlier (`src/localization/maps/<map>`, default `arena_real`) and relocalizes
against it, publishing `map -> odom`.

Map format: `map_file_name` is not the nav2_map_server pair (<map>.yaml and
<map>.pgm). slam_toolbox's localization mode loads a serialized pose graph,
`<map>.posegraph` plus `<map>.data` (an internal karto format). Without them
on_configure fails to load a map and the node does nothing useful, which is
easy to mistake for the lifecycle race below.

When re-mapping the arena, regenerate both pairs from the same live mapping
run: `<map>.yaml` and `<map>.pgm` (map_saver_cli) and `<map>.posegraph` and
`<map>.data` (serialize_map). Call both save services back-to-back on the one
running slam_toolbox. The Nav2 costmap loads `<map>.yaml` while this launch
loads `<map>.posegraph`, and with `do_loop_closing: false` two independent
mapping passes drift differently, so the two would disagree on the world
geometry with no check to catch it.

Lifecycle: same problem and fix as `mapping.launch.py` (see its docstring).
slam_toolbox's `change_state` response is often lost, so
`slam_lifecycle_watchdog.py` polls `get_state`, re-issues transitions and
fails loudly if the node is not `active` in time. It is mode-agnostic, so it
works unchanged here. Like mapping, this uses the synchronous node
(`localization_slam_toolbox_node`).

Everything runs on wall-clock time. The `map` argument (default `arena_real`)
names the saved pose graph.

Start pose: `map_start_pose` ("x, y, yaw" in the map frame in m and rad,
default "0.0, 0.0, 0.0", the graph's origin) is required, otherwise
slam_toolbox logs "Map starting pose not specified", skips the load and
activates with an empty graph. It is only a seed: the scan matcher captures
about +-20 degrees of heading. If the robot starts elsewhere, pass its real
pose or correct it once active with `/initialpose` (RViz "2D Pose Estimate").
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# Generous: bounds the wait before declaring the run dead; a loaded machine
# can take a while to initialise Ceres.
ACTIVATION_TIMEOUT = 60.0


def _require_pose_graph(map_file_name):
    """Raises if the pose graph is missing.

    slam_toolbox only logs "Failed to read file" and still activates, so Nav2
    and the mission would otherwise run against an empty map.

    Args:
        map_file_name: Pose graph path without extension.

    Raises:
        RuntimeError: If `<map>.posegraph` or `<map>.data` does not exist.
    """
    missing = [path for path in (map_file_name + '.posegraph', map_file_name + '.data')
               if not os.path.exists(path)]
    if missing:
        raise RuntimeError(
            'slam_toolbox pose graph not found: ' + ', '.join(missing) + '. '
            'Save the map with /slam_toolbox/serialize_map into '
            'src/localization/maps/<map> and colcon build localization.')


def _parse_start_pose(text):
    """Parses "x, y, yaw" (brackets and spaces optional).

    Args:
        text: The pose text.

    Returns:
        `[x, y, yaw]` as floats.

    Raises:
        RuntimeError: If `text` is not three numbers.
    """
    parts = text.strip().strip('[]').replace(',', ' ').split()
    try:
        pose = [float(part) for part in parts]
    except ValueError:
        pose = []
    if len(pose) != 3:
        raise RuntimeError(
            f'map_start_pose must be "x, y, yaw" (three numbers), got {text!r}')
    return pose


def _launch_nodes(context):
    """Returns the scan filter, slam_toolbox and the activation machinery."""
    # A bool, not text: rclpy declares `use_sim_time` as a bool.
    use_sim_time = False
    share = get_package_share_directory('localization')
    config = os.path.join(share, 'config', 'localization_params.yaml')
    map_file_name = os.path.join(
        share, 'maps', LaunchConfiguration('map').perform(context))
    _require_pose_graph(map_file_name)
    map_start_pose = _parse_start_pose(
        LaunchConfiguration('map_start_pose').perform(context))

    # Removes the robot's antennas from /scan (config/scan_filter.yaml).
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[os.path.join(share, 'config', 'scan_filter.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
    )

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[config, {
            'use_lifecycle_manager': False,
            'use_sim_time': use_sim_time,
            'map_file_name': map_file_name,
            'map_start_pose': map_start_pose,
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
            'slam_toolbox activation CONFIRMED - localization is live.'
            if event.returncode == 0 else
            'ERROR: slam_toolbox never reached "active". It is processing NO '
            'scans; map->odom will never be published from this run. Kill '
            'this launch, check the slam_toolbox log above, and retry.'
        ))],
    ))

    return [
        scan_filter,
        slam_node,
        activate_on_configured,
        TimerAction(period=3.0, actions=[change_state(Transition.TRANSITION_CONFIGURE)]),
        TimerAction(period=4.0, actions=[watchdog]),
        report,
    ]


def generate_launch_description():
    """Declares `map` and `map_start_pose`, then launches the nodes."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='arena_real',
            description='Pose graph basename in localization/maps'),
        DeclareLaunchArgument(
            'map_start_pose', default_value='0.0, 0.0, 0.0',
            description='Seed pose "x, y, yaw" in the map frame (m, rad); '
                        'slam_toolbox does not load the pose graph without it'),
        OpaqueFunction(function=_launch_nodes),
    ])

"""Task 8: the one launch file that brings up the whole real stack.

Formalizes what Task 7 proved by hand (sim + EKF + slam_toolbox localization +
apriltag + oak_points_filter + docking + navigation + mission, all together,
driving a real NavigateToPose + DockRobot cycle) into a single
`ros2 launch mission full_stack.launch.py saga_token:=<token>` command.

ORDERING RATIONALE.

Nothing here is gated on a previous stage's lifecycle nodes actually reaching
`active` before the next stage's processes are *started* -- for two reasons
confirmed across Tasks 2-7:

1. Every lifecycle-managed subsystem in this stack (slam_toolbox's
   localize.launch.py, perception's docking.launch.py, navigation's
   nav2_lifecycle_manager) already owns its own activation story internally
   (the `slam_lifecycle_watchdog.py` pattern, or nav2_lifecycle_manager's own
   autostart retries). Re-deriving a second layer of "wait until active"
   logic here would just duplicate that, with no new information.
2. `mission`'s own behaviour tree (Task 7) is already self-healing against a
   not-yet-ready stack: `VehicleStatusOK`/`Localize` fail closed (not
   crash) until `nav2_lifecycle_manager`'s `is_active` service reports
   true AND `map->base_footprint` resolves and is recent, at which point
   the root `Selector` falls back to `safe_stop` (stop, report fault,
   sleep 2s) and retries next tick. So mission_node can be started early
   without a strict ordering dependency -- it will simply idle in
   `safe_stop` until the rest of the stack catches up.

What the staggering below DOES do is spread out STARTUP load (CPU/IO
contention launching ~15 heavyweight processes, including Gazebo itself, all
at once has been flaky in this environment -- see Tasks 2/3/6/7's own lessons
on lifecycle-activation reliability) and respect the few REAL data
dependencies that would otherwise just waste cycles spinning on missing
topics/services:

  t=0s   simulation.launch.py (world:=arena) -- Gazebo + bridge + sensors.
         Takes the longest to come up from cold, so it starts first.
  t=5s   EKF (localization.launch.py) -- needs /odom, wheel/imu topics from
         the bridge; 5s gives Gazebo a head start publishing them.
  t=5s   apriltag.launch.py, oak_points_filter.launch.py -- need the OAK-D
         image/pointcloud topics from the same bridge; same head start.
  t=7s   localize.launch.py (slam_toolbox, map->odom) -- needs /scan; its own
         internal TimerActions (configure at +3s, watchdog at +4s relative to
         ITS OWN start) then run from t=7s, i.e. activation is attempted
         around t=10-11s, well after the sim is alive.
  t=10s  docking.launch.py (opennav_docking) -- independent of slam_toolbox,
         but staggered after it so the two lifecycle watchdogs (each its own
         ExecuteProcess) don't contend for CPU at the same instant.
  t=18s  navigation.launch.py -- Nav2's global costmap static layer sources
         /map from slam_toolbox (no nav2_map_server in this stack, see
         navigation.launch.py's own docstring), so it needs slam_toolbox to
         have activated and be publishing /map first. 18s gives the t=7s
         localize launch a generous ~11s to finish its own <=60s-bounded
         activation in the common case (observed: a few seconds).
  t=25s  mission.launch.py -- last, once the rest of the stack has had time
         to settle. Not load-bearing per point 2 above, but avoids mission's
         own safe_stop/report-fault cycle spamming logs for the entire
         startup window.

`saga_token` has no default (empty string) deliberately -- the Saga AI mock
server rejects an unauthenticated/garbage-token truck outright, so forgetting
to pass it fails loudly and immediately rather than silently idling.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, launch_arguments=None):
    path = os.path.join(
        get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    saga_url = LaunchConfiguration('saga_url')
    saga_token = LaunchConfiguration('saga_token')

    simulation = _include(
        'einride_mini_truck_bringup', 'simulation.launch.py',
        {'world': world, 'headless': headless, 'rviz': use_rviz})

    ekf = _include('localization', 'localization.launch.py')
    apriltag = _include('perception', 'apriltag.launch.py')
    oak_points_filter = _include('perception', 'oak_points_filter.launch.py')
    localize = _include('localization', 'localize.launch.py')
    docking = _include('perception', 'docking.launch.py')
    navigation = _include('navigation', 'navigation.launch.py')
    mission = _include(
        'mission', 'mission.launch.py',
        {'saga_url': saga_url, 'saga_token': saga_token})

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='arena',
                               description='World file basename (without .sdf).'),
        DeclareLaunchArgument('headless', default_value='true',
                               description='Run the Gazebo server without the GUI.'),
        DeclareLaunchArgument('rviz', default_value='false',
                               description='Open RViz. Off by default to '
                                           'match headless and to keep the '
                                           'compute budget for the stack '
                                           'itself; pass rviz:=true on a dev '
                                           'machine.'),
        DeclareLaunchArgument('saga_url', default_value='http://localhost:8000'),
        DeclareLaunchArgument(
            'saga_token', default_value='',
            description='Bearer token for this truck, from the Saga AI mock '
                         "server's credentials.json. No default on purpose."),

        simulation,
        TimerAction(period=5.0, actions=[ekf, apriltag, oak_points_filter]),
        TimerAction(period=7.0, actions=[localize]),
        TimerAction(period=10.0, actions=[docking]),
        TimerAction(period=18.0, actions=[navigation]),
        TimerAction(period=25.0, actions=[mission]),
    ])

"""Mapping mission on the real robot: wheel and gyro fusion, SLAM, AprilTag capture.

    ros2 launch localization mapping_mission_hardware.launch.py

Includes `mapping.launch.py`, so the lifecycle watchdog and the sync-versus-async
choice live in one place.

`mapper_params_hardware.yaml` sets `odom_frame: odom`, so slam_toolbox looks up
`odom -> base_footprint` for every scan. Nothing published it on hardware
before, so mapping was broken. `wheel_odometry_node` now integrates the
`/wheel_encoders` stream from the vendor's `ugv02_serial_node` (accumulated
distance per side) into a pose and `/odom`.

The EKF owns `odom -> base_footprint`, not the wheels, because wheel odometry
measures turning badly (the scrub below). `localization.launch.py` runs here
too: `gyro_bias_node` removes the gyro's bias and the EKF fuses forward speed
from `/odom` with turn rate from the gyro (see `config/ekf.yaml`). The EKF
broadcasts the TF and `wheel_odometry_node`'s broadcast is off, since two
publishers of one edge fight. slam_toolbox is seeded from the fused estimate.

Turn calibration: measured on the real robot and kept in `config/hardware.yaml`,
the one place to change it (edit, then rebuild and re-source). `wheel_separation`
is 0.4084 m, not the geometric track of 0.17452 m: a skid-steer scrubs
its wheels sideways to turn, so the ideal model over-predicts yaw rate. The
value comes from 5 clockwise in-place turns on the arena floor
(|dR - dL| / (2 pi N)); `localization/wheel_odometry.py`'s docstring has the
method and numbers. It is a single clockwise run; a counter-clockwise one is
still to be averaged in. The Python constants are only code defaults. If the
map is rotationally wrong (corridors meeting at the wrong angle, a square room
closing as a parallelogram), suspect `wheel_separation` first.

Only two of the four driven wheels are sensed (6-wheel 4WD, front axle
instrumented only). The odometry cannot see a rear wheel slipping against the
front one, and skid-steer scrub guarantees some slip. It is still a real
motion prior for seeding each scan match.

Encoder units: the vendor labels `/wheel_encoders` `position` as metres but the
MCU counts whole centimetres (a measured ~2.00 m drive gave 201 counts). The
correction is `encoder_scale` in `config/hardware.yaml`; see
`localization/wheel_odometry.py`.

AprilTag detection: apriltag_ros runs alongside mapping so one drive builds
the map and captures the real dock poses. It starts 5 s after SLAM so it does
not compete with slam_toolbox's CONFIGURE/ACTIVATE window. It includes
`perception`'s `apriltag.launch.py` with `image_topic:=/oak/rgb/image_raw` and
`rectify:=true` (the OAK-D image is distorted and apriltag_ros ignores
distortion, so `image_proc` rectifies first and the detector reads
`/oak/rgb/image_rect`). Pass `apriltag:=false` for a lidar-only run, e.g. if
the Jetson cannot keep up. The camera driver is the vendor's.

  Needs `sudo apt install ros-jazzy-apriltag-ros ros-jazzy-image-proc`. The
  tag-orientation convention in `perception/dock_capture.py` has never been
  checked against a live tag, so the first capture is that check (see "STILL
  UNVERIFIED AGAINST A LIVE TAG" there; capture dock A first).

No device drivers: this starts odometry, fusion, SLAM and (by default)
AprilTag detection only. The serial link (`/wheel_encoders`), lidar (`/scan`),
OAK-D (`/oak/rgb/image_raw`) and RViz come from the vendor's
`hardware.launch.py`, which must already be running and which nothing here
restarts.

Params: `mapper_params_hardware.yaml`, loaded by `mapping.launch.py`.
`correlation_search_space_dimension` is 0.5 because a +-1 m window in this
symmetric room can lock onto the wrong wall and costs ~16x the matching work.
`transform_timeout: 1.0` stays: it absorbs the ~12 ms the encoder stamps are
back-dated by, and `capture_dock_pose.py` and `mission_tree.py` rely on
slam_toolbox stamping map->odom that far ahead. `do_loop_closing: false` is an
open trade-off to revisit once the narrower window has been tried. Read that
file's comments before changing it.

Terminals, in start order:

  0. The robot itself, already running:

         ros2 launch einride_mini_truck_bringup hardware.launch.py

  1. This launch (odometry, fusion, SLAM in mapping mode, AprilTag):

         ros2 launch localization mapping_mission_hardware.launch.py

     Keep the robot still until `gyro_bias_node` logs "gyro bias calibrated"
     (a few seconds), then wait for "slam_toolbox activation CONFIRMED" before
     driving. A run that never reaches `active` maps nothing (see
     `mapping.launch.py`).

  2. Teleop (the terminal must keep focus):

         ros2 run teleop_twist_keyboard teleop_twist_keyboard

     Drive slowly and smoothly. Skid-steer odometry is worst in fast turns and
     sharp direction changes, and every metre of error becomes a metre of
     error in the pose slam_toolbox seeds its scan match from.

  3. The capture tool, recording one dock pose per keypress:

         ros2 run perception capture_dock_pose

     Drive to a dock, stop with its tag square on at about 0.6 m, type `i` and
     Enter. Capture dock A (tag 0) first. The tool rewrites the dock lines in
     `~/modysseus/src/mission/config/docks.yaml` and
     `~/modysseus/src/perception/config/docking.yaml` on the robot.

  4. Save the map when the drive is done:

         ros2 run nav2_map_server map_saver_cli -f src/localization/maps/arena_real

  5. `git diff` in `~/modysseus` (a plausible pose on the arena wall, staging
     pose about 0.6 m out), then commit the captures on the robot so a later
     `git pull` does not conflict. They stay local.

Watching the map in RViz: the vendor's `hardware.launch.py` starts RViz, but
its config (`einride_mini_truck.rviz`) has no Map display. Add one by hand
(Displays -> Add -> By topic -> `/map` -> Map) and change Global Options ->
Fixed Frame from `base_footprint` to `map`. Otherwise the robot stays pinned
to the screen centre and the world slides under it. That `base_footprint`
default in `hardware.launch.py` predates the `odom` frame and is now stale.

Not included: `localize.launch.py`, Nav2, docking and the mission (mapping and
localization both publish map->odom, and nothing autonomous should drive while
a human does), and `oak_points_filter` (it feeds Nav2's collision monitor,
which is not running).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Returns odometry, mapping and (by default) AprilTag detection."""
    share = get_package_share_directory('localization')
    mapping_path = os.path.join(share, 'launch', 'mapping.launch.py')

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_path))

    # Wheel odometry and gyro fusion (the EKF owns odom -> base_footprint),
    # shared with hardware.launch.py.
    hardware_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'hardware_odometry.launch.py')))

    # AprilTag detection, so one drive builds the map and captures dock poses.
    # Not a declared package dependency: perception already depends on
    # localization, so that would be a cycle. FindPackageShare resolves it only
    # when the timer fires, so `apriltag:=false` runs without perception.
    # The vendor camera publishes distorted /oak/rgb/image_raw, hence rectify.
    apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('perception'),
                                  'launch', 'apriltag.launch.py'])),
        launch_arguments={
            'image_topic': '/oak/rgb/image_raw',
            'rectify': 'true',
        }.items())

    return LaunchDescription([
        # `apriltag:=false` gives a lidar-only run.
        DeclareLaunchArgument(
            'apriltag', default_value='true',
            description='Run AprilTag detection (rectified) alongside mapping.'),

        LogInfo(msg=(
            'mapping_mission_hardware: wheel odometry + gyro fusion (EKF) + '
            'SLAM (mapping mode). The robot\'s own drivers, camera included, '
            'come from einride_mini_truck_bringup hardware.launch.py, which '
            'must already be running. Drive with `ros2 run '
            'teleop_twist_keyboard teleop_twist_keyboard` in another '
            'terminal, SLOWLY and SMOOTHLY. Keep the robot still until "gyro '
            'bias calibrated", then wait for slam_toolbox ACTIVE below before '
            'driving. To watch the map grow, add a Map display on /map in '
            'RViz and set Fixed Frame to `map`.')),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('apriltag')),
            msg=(
                'mapping_mission_hardware: AprilTag detection is on '
                '(apriltag:=false to skip it). Capture dock poses with `ros2 '
                'run perception capture_dock_pose` in another terminal. Capture dock A (tag 0) first '
                'and sanity-check it against its existing docks.yaml '
                'entry.')),

        hardware_odometry,
        mapping,

        # Staggered so rectify and detect do not compete with slam_toolbox's
        # CONFIGURE/ACTIVATE window.
        TimerAction(
            period=5.0,
            actions=[apriltag],
            condition=IfCondition(LaunchConfiguration('apriltag'))),
    ])

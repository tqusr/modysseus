"""Mapping mission on the REAL robot: wheel + gyro fusion + SLAM + AprilTag capture.

    ros2 launch localization mapping_mission_hardware.launch.py

The hardware counterpart to `mapping_mission.launch.py`, which is
simulation's. Separate files rather than one file with a mode argument,
following the split the vendor repo already uses
(`einride_mini_truck_bringup`'s `simulation.launch.py` versus
`hardware.launch.py`): the two modes genuinely start different things, and a
single file would hide that behind conditions. `mapping.launch.py` is the
shared half and is INCLUDED by both, so the lifecycle watchdog, the
sync-vs-async choice and the reasoning behind both stay in exactly one place.

WHY THIS EXISTS. `mapper_params_online_async.yaml` sets `odom_frame: odom`,
so slam_toolbox looks up an `odom -> base_footprint` transform for every
scan - and until this launch, NOTHING published that transform on hardware.
There was no wheel odometry node at all. Mapping on the real robot was
therefore not merely degraded, it was broken: every scan's transform lookup
would fail, continuously, with no map to show for the drive.

WHAT NOW PUBLISHES IT: REAL WHEEL ODOMETRY, NOT A STUB. `wheel_odometry_node`
integrates the `/wheel_encoders` stream the vendor's `ugv02_serial_node`
publishes - accumulated distance per side, straight off the chassis MCU - into
a pose and publishes `/odom` - exactly the role Gazebo's DiffDrive plugin
fills in simulation, down to the topic and the frame names.

WHO OWNS `odom -> base_footprint`: THE EKF, NOT THE WHEELS. Wheel odometry
alone measures turning badly (the scrub described below). So
`localization.launch.py` also runs here: `gyro_bias_node` removes the chassis
gyro's bias, and the robot_localization EKF fuses forward speed from `/odom`
with turn rate from the gyro (`config/ekf.yaml` says what is fused and why).
The EKF broadcasts the TF (`publish_tf:=true`) and `wheel_odometry_node`'s own
broadcast is turned off, because two publishers of one TF edge fight - see
`ekf.yaml`. slam_toolbox reads that TF, so it is seeded from the fused
estimate. In simulation Gazebo still owns the edge and the EKF does not
publish it; see `ekf.yaml`.

  THE TURN CALIBRATION IS MEASURED ON THE REAL ROBOT, AND LIVES IN
  `config/hardware.yaml`. The node's `wheel_separation` is 0.4084 m there,
  which is NOT this chassis's geometric track (0.17452 m): a skid-steer has to
  scrub its wheels sideways to rotate, so the ideal differential model
  over-predicts yaw rate. 0.4084 is the effective separation from spinning 5
  clockwise turns in place on the arena floor (|dR - dL| / (2 pi N)); that
  file and `localization/localization/wheel_odometry.py`'s module docstring
  have the method and numbers. It is a SINGLE clockwise run, with a
  counter-clockwise one still to come to average. `config/hardware.yaml` is
  the one place to change the robot's calibration (edit, rebuild/re-source the
  install); the Python constants are only the node's code defaults.

  If the map comes out rotationally wrong - corridors meeting at the wrong
  angle, a square room closing as a parallelogram - `wheel_separation` is the
  first suspect.

  ONLY TWO OF THE FOUR DRIVEN WHEELS ARE SENSED. The chassis is 6-wheel, 4WD
  with the front and rear pairs driven, and only the front axle is
  instrumented - the same limitation `hardware.launch.py`'s
  `joint_state_relay` comment describes. So this is real odometry, not perfect
  odometry: it cannot see a rear wheel slipping differently from the front one
  on the same side, and skid-steer scrub guarantees some slip. It is still a
  real motion prior, which is what slam_toolbox needs to seed each scan match,
  and incomparably better than the nothing that was there before.

  ENCODER UNIT CALIBRATION. The real robot's `/wheel_encoders` `position`
  field is mislabelled by the vendor as metres, but the underlying MCU counts
  are actually whole CENTIMETRES. Confirmed by a physical calibration test:
  driving the robot a measured ~2.00 m produced a 201-count average delta
  across the left/right wheels, i.e. ~0.00995 m per count (~1 cm, a 0.5%
  match). This supersedes an earlier ~1/29 "tick count" assumption, which
  predicted only ~58 counts for the same run (off by 3.47x). See
  `localization/localization/wheel_odometry.py`'s module docstring ("ENCODER
  UNIT CALIBRATION") for the full writeup.

  This confirmed centimetre-to-metre correction is applied via
  `encoder_scale` in `config/hardware.yaml`.

APRILTAG DETECTION, FOR DOCK CAPTURE. `mapping_mission.launch.py` runs
apriltag_ros alongside mapping so one drive both builds the map and captures
the arena's real dock poses; so does this file, 5 s after SLAM starts (so it
does not compete with slam_toolbox's CONFIGURE/ACTIVATE window). It includes
`perception`'s `apriltag.launch.py` with `use_sim_time:=false`,
`image_topic:=/oak/rgb/image_raw`, `camera_info_topic:=/oak/rgb/camera_info`
and `rectify:=true`: the real OAK-D RGB image is distorted and apriltag_ros
ignores distortion, so `image_proc`'s rectify_node runs first and the detector
reads `/oak/rgb/image_rect`. Pass `apriltag:=false` for the lidar-only run
this launch used to be (e.g. if the Jetson cannot keep up). The camera driver
itself is the vendor's, started by `hardware.launch.py`.

  Needs `sudo apt install ros-jazzy-apriltag-ros ros-jazzy-image-proc` on the
  robot. The tag-orientation convention in `perception/perception/
  dock_capture.py` has never been checked against a live tag; the first
  capture on the robot is that check (see "STILL UNVERIFIED AGAINST A LIVE
  TAG" there and the dock-A advice in `mapping_mission.launch.py`).

NO SIM BRING-UP, AND NO DEVICE DRIVERS. This launch starts odometry, fusion,
SLAM and (by default) AprilTag detection, nothing else. The robot's own
drivers - the serial link that produces `/wheel_encoders` and accepts
`/cmd_vel`, the lidar that produces `/scan`, the OAK-D camera that produces
`/oak/rgb/image_raw`, and RViz - come from the vendor's `hardware.launch.py`,
which you are already
running in its own terminal to drive the robot at all. Nothing here would work
without it, and nothing here should restart it.

THE PARAMS FILE WAS TUNED IN SIMULATION. `mapper_params_online_async.yaml` is
used as-is, and several of its values were chosen against the simulator's
specific problems: `transform_timeout: 1.0` for the sim's dual-/clock jitter,
`minimum_travel_heading: 0.15` and `do_loop_closing: false` for a 6x6 m
symmetric arena mapped in sim. Read that file's comments before changing
anything; on hardware the generous transform timeout is now doing different
but still useful work, absorbing the ~12 ms the encoder stamps are back-dated
by.

THE TERMINALS, in the order they have to start.

  0. The robot itself, which you already have running:

         ros2 launch einride_mini_truck_bringup hardware.launch.py

  1. This launch - wheel odometry, gyro fusion, SLAM in mapping mode and
     AprilTag detection:

         ros2 launch localization mapping_mission_hardware.launch.py

     KEEP THE ROBOT STILL until `gyro_bias_node` logs "gyro bias
     calibrated" (a few seconds): it measures the gyro's offset while the
     wheels say the robot is parked. Then wait for "slam_toolbox is ACTIVE" / "slam_toolbox activation CONFIRMED"
     before driving anywhere. `mapping.launch.py`'s watchdog prints it, and a
     mapping run that never reaches `active` silently maps nothing - see that
     file's docstring, which is where the lifecycle story lives.

  2. Teleop - drive the robot:

         ros2 run teleop_twist_keyboard teleop_twist_keyboard

     This terminal must keep focus to steer. No `use_sim_time` here: there is
     no /clock on hardware and the default is already false.

     DRIVE SLOWLY AND SMOOTHLY. Wheel odometry on a skid-steer is at its worst
     exactly where it is pushed hardest: fast turns and sharp direction
     changes scrub the most, and scrub is the part of the motion the wheels
     cannot see. Every metre of error it makes is a metre of error in the pose
     slam_toolbox seeds its scan match from.

  3. The capture tool - record a dock pose per keypress, on the robot:

         ros2 run perception capture_dock_pose

     No `--ros-args -p use_sim_time:=true` here (that flag is for the
     simulator): there is no /clock on hardware, the tool's default is
     already false, and apriltag_ros runs on wall-clock time too. Drive to a
     dock, stop with its tag square on at about 0.6 m, type `i` + Enter.
     Capture dock A (tag 0) first. `mapping_mission.launch.py`'s docstring
     has the full workflow. The tool rewrites the dock lines in
     `~/modysseus/src/mission/config/docks.yaml` and
     `~/modysseus/src/perception/config/docking.yaml` on the robot.

  4. Save the map when the drive is done, exactly as for a sim mapping run:

         ros2 run nav2_map_server map_saver_cli -f src/localization/maps/arena

  5. `git diff` in `~/modysseus` (a plausible pose on the arena wall, staging
     pose about 0.6 m out), then COMMIT the captures there, on the robot, so a
     later `git pull` does not conflict. They stay local; nothing sends them
     to the laptop.

WATCHING THE MAP GROW IN RVIZ. The vendor's `hardware.launch.py` already
starts RViz, but its config
(`einride_mini_truck_bringup/config/einride_mini_truck.rviz`) has NO Map
display - checked, it carries Grid, RobotModel, TF, LaserScan, PointCloud2,
Odometry and three Image displays and nothing else. Add one by hand in the
running RViz: Displays panel -> "Add" -> "By topic" -> `/map` -> Map.

Also change Global Options -> Fixed Frame from `base_footprint` to `map`.
`hardware.launch.py` passes `fixed_frame: base_footprint` to the vendor's
`common.launch.py` because until now hardware had no `odom` frame at all (its
comment says to change it "in the same commit that starts publishing it" -
which is this one, in this repo, so that vendor-side default is now stale).
With the fixed frame left at `base_footprint` the robot stays nailed to the
centre of the screen and the world slides around underneath it, which makes a
growing map very hard to read.

WHAT THIS LAUNCH DOES NOT INCLUDE, and why.

  - `localize.launch.py` / Nav2 / docking / mission. Mapping mode and
    localization mode are mutually exclusive (both publish map->odom), and
    nothing on the autonomy side should be driving while a human is.
  - `oak_points_filter`. It feeds Nav2's collision monitor, which is not
    running here.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: There is no /clock on a real robot.
#:
#: Spelled twice, because the two places it goes want different TYPES and
#: getting that wrong does not degrade anything - it stops the node starting.
#: A launch ARGUMENT is text, so the include below gets the string; a node
#: PARAMETER is typed, and launch_ros passes a plain Python `str` straight
#: through as a string parameter rather than applying YAML rules to it (only
#: an unannotated *substitution* gets those). rclpy's own TimeSource has
#: already declared `use_sim_time` as a bool by then, so a string override is
#: rejected as a type mismatch and the node dies on construction. The vendor
#: repo's hardware.launch.py makes the same distinction - a `'false'` string
#: in its `launch_arguments`, a `False` bool in every `parameters`.
USE_SIM_TIME = False
USE_SIM_TIME_ARG = 'false'


def generate_launch_description():
    share = get_package_share_directory('localization')
    mapping_path = os.path.join(share, 'launch', 'mapping.launch.py')
    hardware_params = os.path.join(share, 'config', 'hardware.yaml')

    # mapping.launch.py defaults use_sim_time to true, for simulation's sake;
    # this is the one caller that says otherwise. A node left at true with no
    # /clock publisher sits at time zero forever and looks exactly like a hang.
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_path),
        launch_arguments={'use_sim_time': USE_SIM_TIME_ARG}.items())

    # The real odometry source, and the reason this launch can work at all.
    # Calibration (wheel_separation, encoder_scale) comes from
    # config/hardware.yaml. See the docstring above for the caveats and the
    # two-of-four-wheels limitation, and
    # localization/localization/wheel_odometry.py for the maths and the input
    # contract. Its TF broadcast is OFF: the EKF below owns
    # odom -> base_footprint, and two publishers of one edge fight.
    wheel_odometry = Node(
        package='localization',
        executable='wheel_odometry_node.py',
        name='wheel_odometry_node',
        output='screen',
        parameters=[hardware_params, {
            'use_sim_time': USE_SIM_TIME,
            'publish_tf': False,
        }],
    )

    # gyro_bias_node + EKF, fusing /odom with the bias-corrected gyro and
    # broadcasting odom -> base_footprint. Started before SLAM so the edge is
    # already in TF when slam_toolbox's first scan lookup happens.
    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': USE_SIM_TIME_ARG,
                          'publish_tf': 'true'}.items())

    # AprilTag detection, so one drive both builds the map and captures where
    # the docks physically are (`ros2 run perception capture_dock_pose`).
    # perception's launch is found through its share directory at run time and
    # NOT declared as a package dependency: perception exec_depends on
    # localization, so the reverse edge would be a cycle colcon refuses to
    # order (mapping_mission.launch.py, the sim twin, does the same).
    #
    # The camera is the vendor's: hardware.launch.py already runs it and
    # publishes /oak/rgb/image_raw (distorted, bgr8) and /oak/rgb/camera_info.
    # Hence rectify:=true (apriltag_ros ignores distortion) and wall-clock time.
    apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('perception'),
                         'launch', 'apriltag.launch.py')),
        launch_arguments={
            'use_sim_time': USE_SIM_TIME_ARG,
            'image_topic': '/oak/rgb/image_raw',
            'camera_info_topic': '/oak/rgb/camera_info',
            'rectify': 'true',
        }.items())

    return LaunchDescription([
        # `apriltag:=false` gives the lidar-only run this launch used to be.
        DeclareLaunchArgument(
            'apriltag', default_value='true',
            description='Run AprilTag detection (rectified) alongside mapping.'),

        LogInfo(msg=(
            'mapping_mission_hardware: wheel odometry + gyro fusion (EKF) + '
            'SLAM (mapping mode) + AprilTag detection (apriltag:=false to '
            'skip it). The robot\'s own drivers, camera included, come from '
            'einride_mini_truck_bringup hardware.launch.py, which must '
            'already be running. Drive with `ros2 run teleop_twist_keyboard '
            'teleop_twist_keyboard` in another terminal, SLOWLY and '
            'SMOOTHLY, and capture dock poses with `ros2 run perception '
            'capture_dock_pose` in a third (no sim-time flag on the robot). '
            'Keep the robot still until "gyro bias calibrated", then wait for '
            'slam_toolbox ACTIVE below before driving. Capture dock A (tag 0) '
            'first and sanity-check it against its existing docks.yaml entry. '
            'To watch the map grow, add a Map display on /map in RViz and set '
            'Fixed Frame to `map`.')),

        wheel_odometry,
        fusion,
        mapping,

        # Staggered like the sim launch: slam_toolbox is busy with its
        # CONFIGURE/ACTIVATE window in the first seconds and rectify + detect
        # would compete with it.
        TimerAction(
            period=5.0,
            actions=[apriltag],
            condition=IfCondition(LaunchConfiguration('apriltag'))),
    ])

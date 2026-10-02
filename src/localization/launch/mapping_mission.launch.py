"""Mapping mission: build a map AND capture the arena's real dock poses in one drive.

    ros2 launch localization mapping_mission.launch.py

THIS FILE IS THE SIMULATION ENTRY POINT. On the real robot use
`mapping_mission_hardware.launch.py` instead, which starts the hardware
wheel-odometry node this file does not need (the simulator's DiffDrive plugin
provides `/odom` and the `odom -> base_footprint` TF) and leaves AprilTag
detection out, so it maps with the lidar only. The two files share
`mapping.launch.py`.

WHY THIS EXISTS. `mission/config/docks.yaml`'s 8 dock poses were computed from
`arena.sdf`'s model origins -- from the SIMULATED world file. There is no
arena.sdf on real hardware: the docks are wherever somebody physically put
them, and nothing in this stack can discover that by itself. This launch turns
the one trip somebody already has to make around a new arena (the slam_toolbox
mapping run) into the trip that also records where the docks are, by running
tag detection alongside mapping so each dock's AprilTag can be resolved in the
`map` frame the same run that builds it.

Composing mapping + apriltag and nothing else is the whole point: the capture
tool and the teleop both need a terminal of their own to read keystrokes from,
so neither can be launched from here (a `ros2 launch` child process does not
own the terminal's stdin). They are separate processes by necessity, not by
oversight.

THREE TERMINALS.

  1. This launch -- SLAM in mapping mode + apriltag_ros:

         ros2 launch localization mapping_mission.launch.py

     Wait for "slam_toolbox is ACTIVE" / "slam_toolbox activation CONFIRMED"
     before driving anywhere. mapping.launch.py's watchdog prints it, and a
     mapping run that never reaches `active` silently maps nothing -- see that
     file's own docstring, which is where the lifecycle story lives.

  2. Teleop -- drive the robot:

         ros2 run teleop_twist_keyboard teleop_twist_keyboard

     This terminal must keep focus to steer. Against the simulator add
     `--ros-args -p use_sim_time:=true` if its velocity ramping behaves oddly
     (it publishes /cmd_vel on its own timer, so it mostly does not care).

  3. The capture tool -- record a dock pose per keypress:

         ros2 run perception capture_dock_pose

     Against the simulator, this one DOES care about the clock:

         ros2 run perception capture_dock_pose --ros-args -p use_sim_time:=true

     without which every detection looks ~1.7e9 seconds old and nothing can be
     captured. The node detects that specific mismatch and says so rather than
     reporting a stale tag, but it cannot fix it for you.

WORKFLOW, in the order it actually works.

  a. Start terminal 1, wait for slam_toolbox ACTIVE. Start terminals 2 and 3.
  b. Drive to a dock. Stop. Put its tag clearly in the camera's view, roughly
     square on and roughly 0.6 m out -- the same geometry the robot will use
     when it stages for a real docking, so the capture is taken from the
     viewpoint it is meant to describe.
  c. In terminal 3, type `i` and press Enter. Every tag currently visible is
     captured (two neighbouring docks in one frame is fine and normal), and
     the matching line of `src/mission/config/docks.yaml` and of
     `src/perception/config/docking.yaml` is rewritten in place.
  d. Repeat for all 8 docks, while also covering the arena well enough for the
     map itself -- these are the same drive.
  e. Save the map, exactly as for a plain mapping run:

         ros2 run nav2_map_server map_saver_cli -f src/localization/maps/arena

  f. `git diff` the two YAML files, then `colcon build` so the captures reach
     the installed share copies the rest of the stack reads.

CAPTURE DOCK A (tag id 0) FIRST, and compare the result against its existing
entry before trusting any of the others. Dock A's current numbers are
known-good: they are sim-derived AND they are the one dock whose staging pose
was empirically confirmed by driving to it and seeing tag 0 centred in the
camera (see docks.yaml's header comment). A fresh capture of dock A in the same
arena should land within a few centimetres and a few degrees of it. If it comes
back mirrored, rotated by pi, or somewhere else entirely, the orientation
convention or the TF chain is wrong and EVERY other capture is wrong the same
way -- stop and fix that, do not capture the remaining seven.

WHAT THIS LAUNCH DOES NOT INCLUDE, and why.

  - The simulator. Mapping an arena in sim means bringing the sim up yourself
    (`einride_mini_truck_bringup`'s simulation.launch.py, world:=arena); on
    real hardware there is nothing to launch. Either way it is not this file's
    business, and `full_stack.launch.py` is the launch that owns sim bring-up.
  - EKF / localize.launch.py / Nav2 / docking / mission. Mapping mode and
    localization mode are mutually exclusive (both would publish map->odom),
    and nothing on the autonomy side should be driving while a human is.
  - `oak_points_filter`. It feeds Nav2's collision monitor, which is not
    running here.

`mapping.launch.py` is INCLUDED rather than copied, so its lifecycle watchdog,
its sync-vs-async choice and the reasoning behind both stay in one place; the
same goes for `perception`'s apriltag.launch.py and its tag family/size config.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(package, launch_file):
    """Include another package's launch file by name.

    Same helper shape as `mission/launch/full_stack.launch.py`'s, which is this
    stack's established way of composing launches.
    """
    path = os.path.join(
        get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def generate_launch_description():
    mapping = _include('localization', 'mapping.launch.py')
    apriltag = _include('perception', 'apriltag.launch.py')

    return LaunchDescription([
        LogInfo(msg=(
            'mapping_mission: SLAM (mapping mode) + AprilTag detection. '
            'Drive with `ros2 run teleop_twist_keyboard '
            'teleop_twist_keyboard` in a second terminal and capture dock '
            'poses with `ros2 run perception capture_dock_pose` in a third '
            '(add `--ros-args -p use_sim_time:=true` to the latter in sim). '
            'Wait for slam_toolbox ACTIVE below before driving. Capture dock '
            'A (tag 0) first and sanity-check it against its existing '
            'docks.yaml entry.')),

        mapping,
        # apriltag_ros only has to be up before the first capture, not before
        # the first scan, and starting it a few seconds late keeps it out of
        # slam_toolbox's own CONFIGURE/ACTIVATE window -- which this stack has
        # repeatedly seen fail under startup CPU contention (see
        # mapping.launch.py's lifecycle notes and full_stack.launch.py's
        # staggering rationale).
        TimerAction(period=5.0, actions=[apriltag]),
    ])

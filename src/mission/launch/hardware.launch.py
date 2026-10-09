"""Terminal 1: the robot side of the stack (odometry, SLAM, Nav2).

    ros2 launch mission hardware.launch.py

Run `mission.launch.py` (docking and mission, with readable logs) in a second
terminal. This one is the noisy one:

    terminal 1:  ros2 launch mission hardware.launch.py
    terminal 2:  ros2 launch mission mission.launch.py saga_token:=<token>

All nodes run on wall-clock time (`use_sim_time` false; there is no /clock).
Design: docs/superpowers/specs/2026-10-03-hardware-full-stack-design.md.

Requires the vendor's `einride-mini-truck` systemd service to be running
already. It provides /wheel_encoders, /imu, /scan and the OAK-D topics, and
nothing here starts or restarts it.

Start order (seconds after launch):

     0  hardware_odometry   wheel odometry, gyro bias, EKF owning
                            odom -> base_footprint                (always)
     0  wheel_drive         cmd_vel_ctrl -> T:1 wheel speeds      (always)
     5  apriltag            rectified, on /oak/rgb/image_raw      (apriltag)
     5  oak_points_filter   XYZ cloud from
                            /oak/stereo/image_raw                 (nav)
     7  localize            slam_toolbox localization on `map`    (always)
    18  navigation          Nav2                                  (nav)

Arguments:
    map: Pose graph basename in localization/maps (default arena_real);
        slam_toolbox reads <map>.posegraph and <map>.data.
    map_start_pose: "x, y, yaw" in the map frame (m, rad), default
        "0.0, 0.0, 0.0"; where slam_toolbox seeds the robot.
    apriltag, nav: `true` (default) or `false` to switch a layer off.
        `nav:=false` also drops the point cloud filter, which only Nav2 reads.
        The mission terminal needs both, since docking measures each tag and
        checks the bay on the Nav2 local costmap.

Workflow:
  1. Make sure the `einride-mini-truck` service is running.
  2. Save the arena's pose graph as `arena_real.posegraph` and
     `arena_real.data` in `src/localization/maps/`, then rebuild
     (`colcon build --packages-select localization`) and source
     `install/setup.bash`. Without them the localize launch raises and names
     the missing paths.
  3. The first time, bring layers up one at a time with a hand on the e-stop
     from step b on:
         a. ros2 launch mission hardware.launch.py nav:=false apriltag:=false
            Keep the robot still until "gyro bias calibrated".
         b. drop `nav:=false`; check
            `ros2 param get /local_costmap/local_costmap use_sim_time` is False.
         c. drop `apriltag:=false`
         d. in a second terminal: ros2 launch mission mission.launch.py \
                saga_token:=<token>
  4. Once slam_toolbox is active, set the initial pose. Its scan matcher only
     captures about +-20 degrees around `map_start_pose`. In RViz set Fixed
     Frame to `map`, click "2D Pose Estimate" where the robot is and drag the
     way it faces, then check the scan lies on the walls. Without RViz:
         ros2 topic pub --once /initialpose \
             geometry_msgs/msg/PoseWithCovarianceStamped \
             "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, \
y: 0.0}, orientation: {w: 1.0}}}}"

Cautions: keep a hand on the e-stop. Nav2 tuning and dock poses still need
retuning on the chassis. This launch has not run on the robot; only
in-process launch-description tests cover it.

Includes use `FindPackageShare`, resolved when the include runs, so a
switched-off layer does not need its package's files.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def _include(package, launch_file, arguments=None, switch=None):
    """Includes `package`'s launch file, resolved when it runs.

    Args:
        package: Package that owns the launch file.
        launch_file: File name under the package's `launch` directory.
        arguments: Launch arguments to pass on.
        switch: Name of a launch argument; if set, the include runs only when
            it is true.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=(arguments or {}).items(),
        condition=IfCondition(LaunchConfiguration(switch)) if switch else None)


def generate_launch_description():
    """Returns the staggered hardware stack."""
    odometry = _include('localization', 'hardware_odometry.launch.py')
    apriltag = _include('perception', 'apriltag.launch.py', {
        'image_topic': '/oak/rgb/image_raw', 'rectify': 'true',
        'gated': 'true'},
        switch='apriltag')
    oak_filter = _include('perception', 'oak_points_filter.launch.py',
                          switch='nav')
    localize = _include('localization', 'localize.launch.py',
                        {'map': LaunchConfiguration('map'),
                         'map_start_pose': LaunchConfiguration('map_start_pose')})
    # Nav2 ends in cmd_vel_ctrl, which wheel_drive_node turns into wheel
    # speeds, so the commanded turn rate is the real one (see navigation.launch.py).
    navigation = _include('navigation', 'navigation.launch.py',
                          {'wz_max': '0.75'}, 'nav')
    wheel_drive = _include('localization', 'wheel_drive.launch.py')

    switches = {
        'apriltag': 'AprilTag detection (rectified), used by /dock_robot to measure each dock',
        'nav': 'Nav2 and the point cloud filter it reads',
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='arena_real',
            description='Pose graph basename in localization/maps '
                        '(<map>.posegraph + <map>.data)'),
        DeclareLaunchArgument(
            'map_start_pose', default_value='0.0, 0.0, 0.0',
            description='Seed pose "x, y, yaw" in the map frame (m, rad)'),
        *[DeclareLaunchArgument(
            name, default_value='true', description=f'Run {what}')
          for name, what in switches.items()],
        odometry,
        wheel_drive,
        TimerAction(period=5.0, actions=[apriltag, oak_filter]),
        TimerAction(period=7.0, actions=[localize]),
        TimerAction(period=18.0, actions=[navigation]),
    ])

"""The whole autonomy stack on the REAL robot: the sim stack minus Gazebo.

    ros2 launch mission full_stack_hardware.launch.py saga_token:=<token>

The hardware counterpart to `full_stack.launch.py`: the same layers in the same
order with the same stagger, but the simulator and the sim odometry are
replaced by the robot's real drivers and the real odometry chain. Every node
runs on wall-clock time (`use_sim_time` false): there is no `/clock` on the
robot. Design: docs/superpowers/specs/2026-10-03-hardware-full-stack-design.md.

NEEDS THE ROBOT'S DRIVERS ALREADY RUNNING. The vendor's systemd service
`einride-mini-truck` provides `/wheel_encoders`, `/imu`, `/scan` and the OAK-D
topics, and consumes `/cmd_vel`. Nothing here starts or restarts it.

THE STAGGER (seconds after launch; the offsets of the sim launch, except that
odometry starts at once because there is no simulator to wait for):

     0  hardware_odometry   wheel odometry + gyro bias + EKF owning
                            odom -> base_footprint                (always)
     5  apriltag            rectified, on /oak/rgb/image_raw      (apriltag)
     5  oak_points_filter   voxel + crop box on /oak/points       (nav)
     7  localize            slam_toolbox localization on `map`    (always)
    10  docking             opennav_docking DockRobot server      (docking)
    18  navigation          Nav2                                  (nav)
    25  mission             mission_node (saga_url, saga_token)   (mission)

ARGUMENTS
    map         pose graph basename in localization/maps (default arena_real);
                slam_toolbox reads <map>.posegraph + <map>.data
    saga_url    default http://localhost:8000
    saga_token  default empty; pass it for the mission layer
    apriltag, nav, docking, mission
                `true` (default) or `false`: switch one layer off so layers can
                be brought up one at a time. `nav:=false` also drops the point
                cloud filter, which only Nav2 reads. AprilTag is a verification
                aid only (nothing consumes /detections): start with
                `apriltag:=false` if the Jetson is saturated.

THE ROBOT TERMINAL WORKFLOW
  1. Make sure the `einride-mini-truck` service is running.
  2. Save the arena's pose graph as `arena_real.posegraph` and
     `arena_real.data` into `src/localization/maps/` and rebuild (`colcon
     build --packages-select localization`, then source `install/setup.bash`):
     `maps` is installed by the package. Without them slam_toolbox fails at
     configure and the localize watchdog says so loudly.
  3. Start the stack, one layer at a time the first time (hand on the e-stop
     from step b on):
         a. ros2 launch mission full_stack_hardware.launch.py \
                nav:=false docking:=false mission:=false apriltag:=false
            Keep the robot still until "gyro bias calibrated".
         b. drop `nav:=false`; check
            `ros2 param get /local_costmap/local_costmap use_sim_time` is False
            (if not, Nav2 needs a hardware copy of its parameter file).
         c. drop `docking:=false`
         d. drop `mission:=false` and add `saga_token:=<token>`
  4. Set the initial pose, once slam_toolbox is active. It starts at the pose
     graph's origin and its scan matcher only captures about +-20 degrees. In
     RViz set Fixed Frame to `map`, click "2D Pose Estimate" where the robot
     really is and drag the way it faces; check the scan lies on the map walls
     and re-click if not. Without RViz:
         ros2 topic pub --once /initialpose \
             geometry_msgs/msg/PoseWithCovarianceStamped \
             "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, \
y: 0.0}, orientation: {w: 1.0}}}}"

CAUTIONS. `docking_server` publishes `/cmd_vel` directly, bypassing the
collision monitor (see docking.launch.py), so dock with `nav` on and a hand on
the e-stop. Nav2 and docking tuning, and the dock poses, are the simulation's
until retuned and captured on the chassis. This launch has not been run on the
robot; it is covered by in-process launch-description tests only.

Includes are located with `FindPackageShare`, resolved when the include runs,
so a switched-off layer does not need its package's files.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

HARDWARE = {'use_sim_time': 'false'}  # launch arguments are text


def _include(package, launch_file, arguments=None, switch=None):
    """Include `package`'s launch file, resolved when it runs, optionally gated."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=(arguments or {}).items(),
        condition=IfCondition(LaunchConfiguration(switch)) if switch else None)


def generate_launch_description():
    odometry = _include('localization', 'hardware_odometry.launch.py')
    apriltag = _include('perception', 'apriltag.launch.py', {
        **HARDWARE, 'image_topic': '/oak/rgb/image_raw', 'rectify': 'true'},
        switch='apriltag')
    oak_filter = _include('perception', 'oak_points_filter.launch.py', HARDWARE, 'nav')
    localize = _include('localization', 'localize.launch.py',
                        {**HARDWARE, 'map': LaunchConfiguration('map')})
    docking = _include('perception', 'docking.launch.py', HARDWARE, 'docking')
    navigation = _include('navigation', 'navigation.launch.py', HARDWARE, 'nav')
    mission = _include('mission', 'mission.launch.py', {
        **HARDWARE, 'saga_url': LaunchConfiguration('saga_url'),
        'saga_token': LaunchConfiguration('saga_token')}, 'mission')

    switches = {
        'apriltag': 'AprilTag detection (rectified), a verification aid',
        'nav': 'Nav2 and the point cloud filter it reads',
        'docking': 'the opennav_docking DockRobot server',
        'mission': 'mission_node',
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='arena_real',
            description='Pose graph basename in localization/maps '
                        '(<map>.posegraph + <map>.data)'),
        DeclareLaunchArgument(
            'saga_url', default_value='http://localhost:8000',
            description='Saga server URL for the mission layer'),
        DeclareLaunchArgument(
            'saga_token', default_value='',
            description='Saga API token for the mission layer'),
        *[DeclareLaunchArgument(
            name, default_value='true', description=f'Run {what}')
          for name, what in switches.items()],
        odometry,
        TimerAction(period=5.0, actions=[apriltag, oak_filter]),
        TimerAction(period=7.0, actions=[localize]),
        TimerAction(period=10.0, actions=[docking]),
        TimerAction(period=18.0, actions=[navigation]),
        TimerAction(period=25.0, actions=[mission]),
    ])

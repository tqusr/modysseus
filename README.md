# Modysseus

ROS 2 (Jazzy) software for an autonomous delivery truck built on the Einride
mini truck (a UGV02 6-wheel skid-steer chassis) with a Jetson, a 2D lidar and an
OAK-D camera. The truck picks up and delivers packages between eight AprilTag
marked docks in a small arena, following routes it reads from the Saga AI event
server.

The truck is read-only towards Saga: organizers advance each route from their
admin page, and the truck only polls for its current route and status.

## Packages

| Package | Purpose |
| --- | --- |
| `localization` | Wheel odometry, gyro bias removal, EKF fusion, wheel-speed drive, and slam_toolbox mapping and localization. |
| `navigation` | Nav2 bring-up: costmaps, planner, MPPI controller, velocity smoother and collision monitor. |
| `perception` | AprilTag detection, the tag-measured docking server (`/dock_robot`, `/undock_robot`), dock pose capture, and the OAK-D point cloud filter. |
| `mission` | The behaviour tree that fetches a route from Saga, drives to the source dock, loads, drives to the destination dock and unloads. |

How they fit together:

```text
Saga server <-- poll -- mission_node (py_trees)
                           |  NavigateToPose, DockRobot, UndockRobot
                           v
   tag_dock_server --> Nav2 --> velocity_smoother --> collision_monitor
   (perception)      (navigation)                          |
                                                     cmd_vel_ctrl
                                                           v
   wheel_odometry + gyro --> EKF --> odom       wheel_drive_node --> chassis MCU
   slam_toolbox ---------------------> map          (localization)
```

The EKF owns the `odom -> base_footprint` transform, slam_toolbox owns
`map -> odom`, and `wheel_drive_node` is the only path to the wheels.

## Requirements

- ROS 2 Jazzy on Ubuntu, with `colcon`.
- Nav2, `slam_toolbox`, `robot_localization`, `laser_filters`, `opennav_docking`,
  `pcl_ros`, `py_trees_ros` and `python3-requests`.
- For AprilTag detection: `sudo apt install ros-jazzy-apriltag-ros ros-jazzy-image-proc`.
- On the robot, the vendor's `einride-mini-truck` systemd service must be
  running. It provides `/wheel_encoders`, `/imu`, `/scan` and the OAK-D topics,
  and nothing in this repository starts or restarts it.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Rebuild after editing a config file or a saved map, since both are installed
from the package share directory.

## Run on the robot

Use two terminals. The first is noisy (Nav2, SLAM, EKF); the second shows only
what the mission is doing.

```bash
# Terminal 1: odometry, drive, AprilTag, SLAM localization and Nav2.
ros2 launch mission hardware.launch.py

# Terminal 2: docking servers and the mission.
ros2 launch mission mission.launch.py saga_token:=<token>
```

The first time, bring the layers up one at a time with a hand on the e-stop:

```bash
ros2 launch mission hardware.launch.py nav:=false apriltag:=false
```

Keep the robot still until `gyro bias calibrated` is logged, then drop
`nav:=false`, then `apriltag:=false`. Once slam_toolbox is active, set the
initial pose with RViz's "2D Pose Estimate" or by publishing to `/initialpose`.
The full workflow is in the docstring of `src/mission/launch/hardware.launch.py`.

Useful arguments:

| Launch file | Argument | Meaning |
| --- | --- | --- |
| `hardware.launch.py` | `map` | Pose graph basename in `localization/maps` (default `arena_real`). |
| `hardware.launch.py` | `map_start_pose` | Seed pose `"x, y, yaw"` in the map frame. |
| `hardware.launch.py` | `apriltag`, `nav` | Switch a layer off (`false`). |
| `mission.launch.py` | `saga_url`, `saga_token` | Saga server address and the truck's bearer token. |
| `mission.launch.py` | `mission:=false` | Start only the docking servers, to test docking without Saga. |

## Map the arena

```bash
ros2 launch localization mapping_mission_hardware.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run perception capture_dock_pose
ros2 run nav2_map_server map_saver_cli -f src/localization/maps/arena_real
```

Drive slowly and smoothly, and wait for `slam_toolbox activation CONFIRMED`
before driving. `capture_dock_pose` records dock poses from visible tags into
`mission/config/docks.yaml` and `perception/config/docking.yaml`. Save the
serialized pose graph (`arena_real.posegraph` and `.data`) from the same
mapping run as the occupancy grid; localization needs both. The details are in
`src/localization/launch/mapping_mission_hardware.launch.py`.

## Configuration

| File | Contents |
| --- | --- |
| `src/localization/config/hardware.yaml` | Wheel radius, effective wheel separation (0.4084 m) and the centimetre encoder scale. The one place to change the robot's calibration. |
| `src/mission/config/docks.yaml` | Dock poses (source of truth) and AprilTag ids. |
| `src/perception/config/docking.yaml` | The same docks in opennav_docking's schema, plus the `tag_dock_server` parameters. |
| `src/navigation/config/nav2_params.yaml` | Nav2 and collision monitor parameters. |

`scripts/sync_docking_yaml.py [--check]` regenerates the dock poses in
`docking.yaml` from `docks.yaml`.

## Test

```bash
source install/setup.bash
cd src/<package> && python3 -m pytest test
```

The launch tests evaluate launch descriptions in-process and need the packages
built and sourced. The perception suite takes about a minute and a half.

## Known limitations

- Wheel odometry senses only two of the four driven wheels, and the turn
  calibration comes from one clockwise run; a counter-clockwise run is still to
  be averaged in.
- The AprilTag orientation convention has been derived offline but not confirmed
  against a live tag. Capture dock A first and compare it with its known-good
  entry.
- This environment sometimes drops lifecycle `change_state` responses. The SLAM
  and docking launches use `slam_lifecycle_watchdog.py`; Nav2's lifecycle manager
  has no equivalent yet, so if "Managed nodes are active" is not logged within
  about 30 s, relaunch.
- During docking and undocking the undock reverse is not collision-checked
  (see the `controller:` notes in `docking.yaml`).

## Documentation

Design specs and implementation plans are in `docs/superpowers/`. Code comments
and docstrings follow the Google Python style guide.

## License

See [LICENSE](LICENSE).

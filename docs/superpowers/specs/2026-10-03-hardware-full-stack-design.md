# Full stack on the real robot - design

Date: 2026-10-03. Branch: `hardware-full-stack` (off `apriltag-hardware`). Status: agreed design, not yet implemented.

## Goal

One command brings up the simulated full stack's software on the real robot: `ros2 launch mission full_stack_hardware.launch.py saga_token:=<token>`. "Simple: the simulated version, connected to the hardware." Same staggered structure and order as `full_stack.launch.py`, with Gazebo and the sim odometry swapped for the real drivers and the real odometry chain.

## Design

### New launch: `src/mission/launch/full_stack_hardware.launch.py`

No Gazebo. The vendor systemd service `einride-mini-truck` already provides `/wheel_encoders`, `/imu`, `/scan`, `/oak/...` and consumes `/cmd_vel`. Every node runs with `use_sim_time` false. Stagger (same offsets as the sim launch; odometry starts at 0 because there is no simulator to wait for):

| t | what | switch |
|---|------|--------|
| 0 s | `hardware_odometry.launch.py` (wheel odometry + gyro bias + EKF owning `odom -> base_footprint`, `hardware.yaml`) | always |
| 5 s | `perception/apriltag.launch.py` (`rectify:=true`, `/oak/rgb/image_raw`) | `apriltag` |
| 5 s | `perception/oak_points_filter.launch.py` | `nav` |
| 7 s | `localization/localize.launch.py` (slam_toolbox localization on `map`) | always |
| 10 s | `perception/docking.launch.py` | `docking` |
| 18 s | `navigation/navigation.launch.py` | `nav` |
| 25 s | `mission/mission.launch.py` (`saga_token`, `saga_url`) | `mission` |

Launch arguments: `map` (default `arena_real`), `saga_token` (no default, as in sim), `saga_url`, and boolean switches `apriltag`, `nav`, `docking`, `mission` (all default `true`) so one layer can be tested at a time. Switches are `IfCondition`s on the includes. Includes use `FindPackageShare` (resolved when the include runs), the pattern `mapping_mission_hardware.launch.py` already uses. No new package and no `package.xml` change: `mission` already exec_depends on `localization`, `perception` and `navigation`.

`map` names a serialized pose graph: `localize.launch.py` resolves it to `<localization share>/maps/<map>` and slam_toolbox reads `<map>.posegraph` + `<map>.data`. The user saves `arena_real.posegraph/.data` into `src/localization/maps/` on the robot and rebuilds (`maps` is installed by `CMakeLists.txt`). The `.yaml/.pgm` pair is for nav2/RViz only and is not read by the launch.

### Reuse: `src/localization/launch/hardware_odometry.launch.py`

Extracted from `mapping_mission_hardware.launch.py`: the `wheel_odometry_node` (config `hardware.yaml`, `publish_tf: False`, `use_sim_time: False`) plus the include of `localization.launch.py` (`use_sim_time:=false`, `publish_tf:=true`). Both `mapping_mission_hardware.launch.py` and the new launch include it, in the same order as today, so the graph of the mapping launch does not change. It takes no arguments (hardware only). The extraction is clean: the code is two adjacent actions with no coupling to the rest of the file. One existing test, `test_hardware_launch_still_runs_on_wall_clock_without_the_camera_nodes`, looks for `wheel_odometry_node` among the launch's own top-level nodes; it must be changed to look inside the include (same assertion, one level deeper).

### Sim-hardcoded includes become configurable

Each gains `use_sim_time` (default `true`, so sim callers are unchanged), converted to a typed bool for node parameters (`_as_bool`, as in `perception/launch/apriltag.launch.py`; a string would make rclpy reject the node). Because the body now depends on the argument, each becomes an `OpaqueFunction`.

| file | hardcoded today | change |
|------|-----------------|--------|
| `localization/launch/localize.launch.py` | `scan_filter`, `slam_toolbox` `True`; map `maps/arena` | `use_sim_time`; new `map` (default `arena`) |
| `perception/launch/docking.launch.py` | `docking_server` `True` | `use_sim_time` |
| `navigation/launch/navigation.launch.py` | `common_args` and `lifecycle_manager` `True` | `use_sim_time` |
| `mission/launch/mission.launch.py` | `mission_node` `True` | `use_sim_time`; `saga_*` performed to `str` |
| `perception/launch/oak_points_filter.launch.py` | none inline, but `config/oak_points_filter.yaml` sets `use_sim_time: true` on both filter nodes | inline `{'use_sim_time': ...}` after the config file |

Verified fact that shaped this table: the YAML files also hardcode `true` (`oak_points_filter.yaml`, `docking.yaml`, `nav2_params.yaml`, `localization_params.yaml`, `ekf.yaml`). Launch applies parameter sources in order, so the inline dict after the file wins, and that is the mechanism used. The YAMLs are not edited. `mission.launch.py` currently passes `saga_token` as a bare substitution, which launch parses by YAML rules (a numeric-looking token would become an int); performing it to a string fixes that as a side effect.

### Initial pose

slam_toolbox in localization mode starts at the pose graph's origin; `map_start_pose` is deliberately not set (see `localization_params.yaml`: a wrong seed fails silently). The scan matcher only captures about +-20 deg. Read from the installed `liblocalization_slam_toolbox.so`, the node subscribes to `initialpose` (root namespace, so `/initialpose`, `geometry_msgs/PoseWithCovarianceStamped`), the same topic RViz's "2D Pose Estimate" publishes by default. So: set RViz Fixed Frame to `map`, click "2D Pose Estimate" where the robot really is and drag in the direction it faces, once after slam_toolbox is active. Check the scan overlays the map walls; re-click if not. Fallback: `ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"`.

## Out of scope

Nav2/docking tuning for hardware (still the sim's `nav2_params.yaml`/`docking.yaml`), map saving tooling, Saga server setup, editing the YAML `use_sim_time` entries.

## Risks

- Jetson CPU: AprilTag (rectify + detect at 30 Hz), slam_toolbox, Nav2 and the point-cloud filter together. The switches exist to bring layers up one at a time; start with `apriltag:=false` if the machine is saturated (AprilTag is a verification aid only; nothing consumes `/detections`).
- Nav2 costmap nodes (`/local_costmap/local_costmap`, `/global_costmap/global_costmap`) get `use_sim_time: true` from `nav2_params.yaml` by node name; the inline override targets `controller_server`/`planner_server`. Nav2 normally propagates the parent's value, but this is unverified here. On the robot check `ros2 param get /local_costmap/local_costmap use_sim_time` is `False`; if not, the fix is a hardware copy of the param file.
- `docking_server` publishes `/cmd_vel` directly, bypassing the collision monitor (see its launch docstring). Docking's collision detection reads Nav2's local costmap, so docking is tested with `nav` on. Keep a hand on the e-stop.
- The nav2/docking tuning is the sim's; expect to retune on the real chassis (wheel scrub, `wheel_separation` 0.4084).
- Docks `docks.yaml`/`docking.yaml` carry sim poses until captured on the robot (`capture_dock_pose`).
- The lifecycle activation race (lost `change_state` responses) was seen in sim; the watchdogs are reused unchanged.

## Testing

Automated: in-process launch-description tests with `perception/test/launch_helpers.py` (no process starts, no ROS graph), plus a small `describe_node` extension that exposes inline parameters.

- Each modified launch: default arguments give the sim values (`use_sim_time` is `True` as a bool, `map` resolves to `maps/arena`); `use_sim_time:=false` reaches every node as a bool `False`.
- `hardware_odometry`: one `wheel_odometry_node` (`publish_tf` False, `hardware.yaml`) plus the fusion include with `publish_tf:=true`; `mapping_mission_hardware` tests still pass.
- `full_stack_hardware`: recursively walks includes and timers; no include of `simulation.launch.py` and no Gazebo node; every node's `use_sim_time` is false (including nodes inside includes); `map:=foo` reaches slam_toolbox as `.../maps/foo`; each switch false skips exactly its include(s); `saga_token` reaches `mission_node`.

Manual on-robot staged checklist (service `einride-mini-truck` running, `arena_real` map installed, hand on the e-stop for 2 to 4). Each stage adds one switch; stop and fix before continuing.

1. Localization only: `ros2 launch mission full_stack_hardware.launch.py nav:=false docking:=false mission:=false apriltag:=false`. Keep the robot still until "gyro bias calibrated". In RViz (Fixed Frame `map`) add `/scan_filtered` and `/map`, click 2D Pose Estimate, confirm the scan lies on the walls and stays there while driving by hand. `ros2 run tf2_ros tf2_echo map base_footprint` resolves.
2. Add navigation: drop `nav:=false`. Also `ros2 param get /local_costmap/local_costmap use_sim_time` is `False`. Send a short 2D Goal Pose in open floor; the robot drives, stops short of obstacles, and the e-stop is within reach.
3. Add docking: drop `docking:=false`. One dock from the CLI (server `dock_robot`, `nav2_msgs/action/DockRobot`, dock ids A-H in `docking.yaml`):
   `ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{use_dock_id: true, dock_id: 'H', navigate_to_staging_pose: true}" --feedback`
   Undock: `ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot "{dock_type: 'arena_dock'}"`.
4. Add mission: run the Saga mock, then the full command with `saga_token:=<token>`; confirm `mission_node` leaves `safe_stop` once `map -> base_footprint` is live and `nav2` is active, and runs one order.

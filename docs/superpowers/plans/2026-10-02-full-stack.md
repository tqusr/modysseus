# Full Autonomy Stack Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the Modysseus stack — Localization & Mapping, Perception & Docking, Navigation & Control — and prove it drives the robot through a real, non-stubbed Saga AI delivery loop in the `einride-mini-truck` Gazebo simulation.

**Architecture:** Three new colcon packages (`localization`, `perception`, `navigation`) overlaid the same way `mission` already is, built from mature off-the-shelf ROS 2 packages (`robot_localization`, `slam_toolbox`, `apriltag_ros`, `opennav_docking`, Nav2) configured and wired together via launch files, plus targeted changes to `mission` swapping its stub action servers for the real ones this plan builds.

**Tech Stack:** ROS 2 Jazzy, `robot_localization`, `slam_toolbox`, `apriltag_ros`, `opennav_docking`, full Nav2 (`nav2_bringup`, `nav2_bt_navigator`, `nav2_mppi_controller`, `nav2_rotation_shim_controller`, `nav2_collision_monitor`, `nav2_lifecycle_manager`), the `einride-mini-truck` Gazebo simulation, the local Saga AI mock server.

**Spec:** `docs/superpowers/specs/2026-10-02-full-stack-design.md`

## Global Constraints

- Work only in `/home/albin/modysseus` — never `git push` without explicit instruction.
- Never add a `Co-Authored-By: Claude` (or any AI-attribution) trailer to any commit — standing instruction for this repo.
- Overlay sourcing order before any build/run: `source /opt/ros/jazzy/setup.bash && source /home/albin/einride-mini-truck/install/setup.bash && source /home/albin/modysseus/install/setup.bash` (the last one only after the first build).
- Dock file is the single source of truth: `src/mission/config/docks.yaml`. The 8 real poses are (x, y, yaw radians), tag ids 0-7 for A-H:
  ```
  A (2.992, -1.500, 3.14159)    E (-2.992,  1.500, 0.0)
  B (2.992,  1.500, 3.14159)    F (-2.992, -1.500, 0.0)
  C (1.500,  2.992, -1.5708)   G (-1.500, -2.992, 1.5708)
  D (-1.500, 2.992, -1.5708)   H (1.500,  -2.992, 1.5708)
  ```
- Velocity command order is always: Nav2 controller/docking server → velocity smoother → `nav2_collision_monitor` → `/cmd_vel`. Collision monitor is always last, no exceptions.
- `/oak/points` and `/scan/points` (camera and lidar point clouds) already exist from the simulation — do not build a separate point-cloud-generation pipeline, consume these directly.
- `odom → base_footprint` is already published by Gazebo's diff-drive plugin — localization's job is `map → odom` plus a refined `odom → base_link` via EKF, not reinventing basic odometry.
- Isaac ROS packages (`isaac_ros_*`) are unavailable on this machine (no GPU) — never attempt to install or depend on them. `apriltag_ros` replaces `isaac_ros_apriltag` everywhere.
- No unit-test-first discipline required for config/launch-file tasks in this plan (per the spec's "Testing approach") — verification is: launch it, inspect the real topics/TF/actions with `ros2 topic echo`/`ros2 run tf2_ros tf2_echo`/`ros2 action list`, and fix what's wrong. Each task still ends with a concrete, runnable verification command and its expected output — "write a test" is replaced by "run this command and confirm this is what you see."
- Multi-robot features (occupied-dock-by-another-truck) cannot be tested in this single-robot sim — implement the interface, hardcode the sim-side signal to "always free" (matching `mission`'s existing pattern), and say so in a code comment, not a TODO that gets lost.

---

## Task 1: `localization` package scaffolding + EKF

**Files:**
- Create: `src/localization/package.xml`, `setup.py` or `CMakeLists.txt`+`package.xml` (ament_cmake if config-only, ament_python if any node scripts are added — default to a plain config package: `package.xml` + launch/config dirs only, no Python node needed for EKF since `robot_localization`'s `ekf_node` is a prebuilt executable you just launch with a config file)
- Create: `src/localization/config/ekf.yaml`
- Create: `src/localization/launch/localization.launch.py` (EKF only for this task; mapping/localization launches come in Tasks 2-3)

**Interfaces:**
- Produces: `/odometry/filtered` topic, `odom → base_link` TF, once launched alongside the Gazebo sim.

- [ ] **Step 1: Confirm `robot_localization` is installed, install if not**

```bash
dpkg -l | grep ros-jazzy-robot-localization
```
If missing, this needs a sudo install — same pattern as before in this project: write a small script, ask the user to run it with `! bash <script>`, since the agentic session cannot run sudo. If already installed, proceed.

- [ ] **Step 2: Scaffold the package**

Ament CMake config-only package (no compiled code, just installs config/launch files):
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>localization</name>
  <version>0.0.0</version>
  <description>EKF, SLAM mapping/localization, and AprilTag pose correction for Modysseus.</description>
  <maintainer email="albin@stenboda.se">Albin</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>robot_localization</depend>
  <depend>slam_toolbox</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
`CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(localization)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
```

- [ ] **Step 3: Write `config/ekf.yaml`**

Fuse `/imu` (orientation + angular velocity + linear acceleration) with the existing wheel-odometry source. Check what odometry topic the sim actually publishes (`ros2 topic list | grep odom` after launching the sim — likely `/odom`, matching earlier session testing) and use it as `odom0`. Standard two-source EKF:
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    imu0: imu
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
    publish_tf: true
```
Adjust the boolean config matrices if `ros2 topic echo /imu --once` or `/odom --once` shows a different field layout than expected — the config arrays are [x,y,z, roll,pitch,yaw, vx,vy,vz, vroll,vpitch,vyaw, ax,ay,az]; pick `true` for fields each source actually measures.

- [ ] **Step 4: Write `launch/localization.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory('localization') + '/config/ekf.yaml'
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config],
        ),
    ])
```

- [ ] **Step 5: Build and verify**

```bash
colcon build --packages-select localization
source install/setup.bash
```
In one terminal, launch the einride-mini-truck sim (`ros2 launch einride_mini_truck_bringup simulation.launch.py headless:=true`); in another, `ros2 launch localization localization.launch.py`; drive the robot a bit (`ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'` for a couple seconds, then stop); confirm:
```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
```
Expected: both produce sane, non-zero output tracking the robot's actual motion (not frozen at zero, not wildly diverging).

- [ ] **Step 6: Commit**

```bash
git add src/localization
git commit -m "Add EKF localization package"
```

---

## Task 2: Arena mapping (slam_toolbox, mapping mode)

**Files:**
- Create: `src/localization/launch/mapping.launch.py`
- Create: `src/localization/config/mapper_params_online_sync.yaml` (or whatever filename slam_toolbox's launch convention expects for its params file — check `ros2 pkg prefix slam_toolbox`'s share directory for the example config shipped with the package and adapt it rather than writing one from scratch blind)
- Create (output, not hand-written): a saved map under `src/localization/maps/arena.yaml` + `arena.pgm` — generated by actually running the mapper, not authored.

**Interfaces:**
- Produces: `src/localization/maps/arena.yaml`/`.pgm`, consumed by Task 3's localization launch.

- [ ] **Step 1: Find and adapt slam_toolbox's shipped example config**

```bash
find /opt/ros/jazzy/share/slam_toolbox -name "*.yaml" | xargs -I{} basename {}
cat /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml
```
Copy the online-async (or online-sync, either is fine for this single-robot offline mapping use case) example into `src/localization/config/`, and adjust only what's needed: `scan_topic` to match the sim's actual lidar topic (`/scan`), `base_frame` to `base_link` (or `base_footprint`, whichever the TF tree from Task 1 actually resolves through — check with `ros2 run tf2_ros tf2_echo map base_link` once running), `odom_frame` to `odom`, `resolution` reasonable for a small indoor arena (e.g. `0.02`-`0.05`), `max_laser_range` matching the LD19P's real range (0.02-12m per einride's docs).

- [ ] **Step 2: Write `launch/mapping.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory('localization') + '/config/mapper_params_online_async.yaml'
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',  # or sync_slam_toolbox_node, matching whichever config you adapted
            name='slam_toolbox',
            output='screen',
            parameters=[config],
        ),
    ])
```

- [ ] **Step 3: Run it, drive the robot around the whole arena, save the map**

```bash
mkdir -p src/localization/maps
```
Launch the sim + Task 1's localization (EKF) + this mapping launch together. Drive the robot around the full arena perimeter and through the middle (teleop via repeated `ros2 topic pub /cmd_vel ...` calls, or write a short throwaway script that publishes a simple patrol pattern — your call, it doesn't need to be elegant, it's a one-time data-collection pass) so the lidar sees every wall and dock. Then:
```bash
ros2 run nav2_map_server map_saver_cli -f src/localization/maps/arena --ros-args -p save_map_timeout:=10000.0
```
Confirm `src/localization/maps/arena.yaml` and `.pgm` were created and the `.pgm` visually resembles a plausible room/arena outline when opened (`file src/localization/maps/arena.pgm` at minimum to confirm it's not empty/corrupt; if you can inspect pixel content some other way — e.g. a quick Python script checking occupied-vs-free pixel counts are non-trivial — do that too).

- [ ] **Step 4: Commit**

```bash
git add src/localization/launch/mapping.launch.py src/localization/config/mapper_params_online_async.yaml src/localization/maps
git commit -m "Add arena mapping launch and a saved map of the simulated arena"
```

---

## Task 3: Arena localization (slam_toolbox, localization mode)

**Files:**
- Create: `src/localization/launch/localize.launch.py`
- Create: `src/localization/config/localization_params.yaml` (adapted from slam_toolbox's shipped localization example, same approach as Task 2)

**Interfaces:**
- Consumes: `src/localization/maps/arena.yaml` (Task 2).
- Produces: `map → odom` TF, available to Perception (Task 5) and Navigation (Task 7).

- [ ] **Step 1: Find and adapt slam_toolbox's localization example config**

```bash
find /opt/ros/jazzy/share/slam_toolbox -iname "*localization*"
```
Adapt it the same way as Task 2's mapping config, pointing `map_file_name` (or whatever the exact param is called in the installed version — check the example file) at `src/localization/maps/arena` (slam_toolbox appends `.yaml`/`.posegraph` itself — follow its actual convention, don't guess the extension).

- [ ] **Step 2: Write `launch/localize.launch.py`** (same shape as `mapping.launch.py` but using `localization_slam_toolbox_node` and the localization config)

- [ ] **Step 3: Run it and verify**

Launch sim + EKF + this localization launch (NOT the mapping launch — they're mutually exclusive uses of slam_toolbox). Drive the robot a short distance and confirm:
```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```
Expected: both resolve (no "waiting for transform" hang) and `map → base_link` tracks the robot's actual position as it drives (compare against the sim's ground-truth pose if easily available, e.g. via Gazebo's own GUI if not running headless, or just sanity-check the direction/magnitude of movement matches what you commanded).

- [ ] **Step 4: Commit**

```bash
git add src/localization/launch/localize.launch.py src/localization/config/localization_params.yaml
git commit -m "Add arena localization launch against the saved map"
```

---

## Task 4: `perception` package + AprilTag detection

**Files:**
- Create: `src/perception/package.xml`, `CMakeLists.txt`
- Create: `src/perception/config/apriltag.yaml`
- Create: `src/perception/launch/apriltag.launch.py`

**Interfaces:**
- Produces: `/tag_detections` (or whatever topic `apriltag_ros` actually names it — confirm, don't assume) with tag poses in the camera frame; TF for each detected tag once a `tag_frames`-style broadcaster is enabled (check `apriltag_ros`'s own config options for this — it usually can publish TF directly).

- [ ] **Step 1: Install and inspect `apriltag_ros`**

```bash
dpkg -l | grep ros-jazzy-apriltag-ros
```
If missing, same sudo-script pattern as Task 1. Once installed:
```bash
find /opt/ros/jazzy/share/apriltag_ros -name "*.yaml"
```
Read the shipped example config to get the exact parameter names right (tag family, size, topic remapping) rather than guessing.

- [ ] **Step 2: Scaffold the package** (same ament_cmake config-only shape as `localization`'s `package.xml`/`CMakeLists.txt`, depend on `apriltag_ros`)

- [ ] **Step 3: Write `config/apriltag.yaml`**

Tag family `tag36h11`, size `0.100` (meters, per the dock model's own built-in comment), tag ids 0-7 all valid (no need to list individual tag sizes if they're uniform — check whether the installed version requires per-tag-id entries or accepts a single default size).

- [ ] **Step 4: Write `launch/apriltag.launch.py`**, remapping `apriltag_ros`'s expected image/camera_info topics to `/oak/rgb/image_raw` and `/oak/rgb/camera_info`.

- [ ] **Step 5: Run it and verify**

Launch sim (robot positioned facing a dock — may need to drive it there first using the navigation/localization already working, or just manually teleport/drive close to one for this isolated test) + this launch:
```bash
ros2 topic echo /tag_detections --once
```
Expected: a detection message with a non-empty `detections` array when the camera can see a dock's tag, containing a plausible pose. If nothing appears, check the robot is actually within camera range/view of a dock (docks are ~3m from arena center per the pose table — drive close to one) before treating it as a bug.

- [ ] **Step 6: Commit**

```bash
git add src/perception
git commit -m "Add AprilTag detection"
```

---

## Task 5: Real dock poses + docking server

**Files:**
- Modify: `src/mission/config/docks.yaml` (replace placeholder `0,0,0` poses with the real ones)
- Create: `src/perception/config/docking.yaml`
- Create: `src/perception/launch/docking.launch.py`

**Interfaces:**
- Produces: a real `DockRobot` action server (package/executable from `opennav_docking`), to replace `mission`'s stub.

- [ ] **Step 1: Update `src/mission/config/docks.yaml` with the real poses**

Using the table in this plan's Global Constraints section for `dock_pose`. For `staging_pose`: a point ~0.5-0.7m back from the dock along its approach line, with the robot facing the dock. The exact sign convention depends on which way the dock model's tag actually faces — don't guess blind: launch the sim, drive the robot to approximately where you think the staging area should be for one dock (e.g. dock A), confirm via the camera feed or AprilTag detection (Task 4) that the tag is visible and roughly centered facing the robot, then compute the other 7 staging poses by the same geometric relationship to their own `dock_pose`/yaw. Document the final formula you used in a code comment in `docks.yaml` (as a YAML comment) so it's clear how `staging_pose` relates to `dock_pose`, not just a table of magic numbers.

- [ ] **Step 2: Install and inspect `opennav_docking`**

```bash
dpkg -l | grep ros-jazzy-opennav-docking
```
Same sudo-script pattern if missing.
```bash
find /opt/ros/jazzy/share/opennav_docking -name "*.yaml"
```
Read its shipped example config/docs for the exact structure of a dock database (likely a list of docks with pose + a "dock plugin" type — there may be a built-in AprilTag-based dock plugin, or you may need `opennav_docking`'s simple-charging-dock plugin pointed at the detected tag pose rather than a hardcoded world pose, so the final approach is vision-corrected rather than purely odometry-based, per the spec's intent).

- [ ] **Step 3: Write `config/docking.yaml`** populating the dock database from `mission`'s `docks.yaml` values (either by hand-transcribing for now, or writing a tiny one-off script that reads `docks.yaml` and emits `docking.yaml` — your call; if you hand-transcribe, add a comment noting `docks.yaml` is the source of truth and this file must be kept in sync).

- [ ] **Step 4: Write `launch/docking.launch.py`** bringing up `opennav_docking`'s server node with this config, plus its own lifecycle manager if it needs one separate from Nav2's (check the package's launch examples).

- [ ] **Step 5: Run it and verify**

```bash
ros2 action list | grep dock
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{dock_id: 'A', use_dock_id: true}" --feedback
```
(Substitute the actual action name/type if `opennav_docking` names it differently — confirm via `ros2 action list` rather than assuming `/dock_robot`.) Expected: with the robot positioned somewhere reasonable in the arena with localization (Task 3) and AprilTag detection (Task 4) both running, the action drives the robot to dock A and reports success — this is the first point in this plan where something should visibly, actually move and dock, not just publish a topic.

- [ ] **Step 6: Commit**

```bash
git add src/mission/config/docks.yaml src/perception/config/docking.yaml src/perception/launch/docking.launch.py
git commit -m "Add real dock poses and the opennav_docking server"
```

---

## Task 6: `navigation` package — costmaps, planner, controller, safety chain

**Files:**
- Create: `src/navigation/package.xml`, `CMakeLists.txt`
- Create: `src/navigation/config/nav2_params.yaml`
- Create: `src/navigation/launch/navigation.launch.py`

**Interfaces:**
- Produces: the real `NavigateToPose` action (via `nav2_bt_navigator`), to replace `mission`'s stub.

- [ ] **Step 1: Confirm Nav2 packages installed**

```bash
dpkg -l | grep -E "ros-jazzy-navigation2|ros-jazzy-nav2-bringup|ros-jazzy-nav2-mppi-controller|ros-jazzy-nav2-rotation-shim-controller|ros-jazzy-nav2-collision-monitor"
```
Same sudo-script pattern for anything missing — bundle all the missing ones into one install script rather than asking the user to run several separate scripts.

- [ ] **Step 2: Scaffold the package** (ament_cmake config-only, depend on `nav2_bringup`, `nav2_mppi_controller`, `nav2_rotation_shim_controller`, `nav2_collision_monitor`, `nav2_lifecycle_manager`, `nav2_costmap_2d`)

- [ ] **Step 3: Write `config/nav2_params.yaml`**

Start from `nav2_bringup`'s own shipped example params file (`find /opt/ros/jazzy/share/nav2_bringup -name "nav2_params.yaml"`) rather than writing one from scratch — it already has reasonable defaults for every Nav2 node. Adapt:
- Robot footprint/radius from `einride_mini_truck_description` (check the URDF/SDF for the actual chassis dimensions — track 0.17452m, wheelbase 0.171m per earlier session notes; derive a reasonable footprint box or radius from those).
- Costmap obstacle layers: lidar layer on `/scan` (or `/scan/points`), plus an added layer consuming `/oak/points` for the camera obstacles — check `nav2_costmap_2d`'s `PointCloud2`/`Obstacle` layer plugin options for the right plugin name/params to subscribe to a `PointCloud2` topic directly.
- Controller plugin: `nav2_mppi_controller::MPPIController`, with `nav2_rotation_shim_controller::RotationShimController` wrapping it per the spec.
- Planner plugin: `nav2_navfn_planner::NavfnPlanner`.
- Velocity smoother + `nav2_collision_monitor` configured as the last stage before `/cmd_vel`, with stop/slowdown zones sized to the footprint above.

- [ ] **Step 4: Write `launch/navigation.launch.py`**, bringing up the Nav2 node set with `nav2_lifecycle_manager` managing them, pointed at this package's params file. Base this structurally on `nav2_bringup`'s own `navigation_launch.py` (check `/opt/ros/jazzy/share/nav2_bringup/launch/`) rather than writing the whole thing blind — copy and trim to what's actually needed (no AMCL — slam_toolbox already provides `map→odom`, don't duplicate localization inside Nav2's own launch).

- [ ] **Step 5: Run it and verify**

Launch sim + localization (Tasks 1-3) + this navigation launch. Confirm:
```bash
ros2 action list | grep navigate_to_pose
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```
Expected: the robot actually drives toward that pose in the arena and the action reports success. This is the big one — iterate on costmap/controller params if it gets stuck, oscillates, or can't find a path, per the spec's stated testing approach (fix until it works, not until it's tuned for speed).

- [ ] **Step 6: Commit**

```bash
git add src/navigation
git commit -m "Add Nav2 navigation stack"
```

---

## Task 7: Mission package — swap stubs for real servers, real health checks

**Files:**
- Modify: `src/mission/mission/mission_node.py` (no action-name changes needed if the stub and real servers happen to share action names `navigate_to_pose`/`dock_robot` — check Tasks 5 and 6's actual action names against what `mission_tree.py`'s `CallAction` calls already use; only change what's actually different)
- Modify: `src/mission/mission/mission_tree.py` (`VehicleStatusOK`, `Localize` — replace stub `Success` behaviours with real checks)
- Modify: `src/mission/launch/mission.launch.py` (stop launching `stub_action_servers` once the real servers from Tasks 5-6 are confirmed working; keep the stub code in the repo, just unreferenced, rather than deleting it — in case something regresses and stubs are needed again for isolated mission-tree testing)

**Interfaces:**
- Consumes: the real `DockRobot` (Task 5) and `NavigateToPose` (Task 6) actions.

- [ ] **Step 1: Confirm action name/type compatibility**

```bash
ros2 action list
ros2 action info /navigate_to_pose
ros2 action info /dock_robot   # or whatever Task 5's actual action name is
```
Compare against `mission_tree.py`'s `CallAction(..., action_name="navigate_to_pose", action_type=NavigateToPose, ...)` / `action_name="dock_robot"` call sites. If the names/types already match (likely, since both used `nav2_msgs` action types from the start), no code change is needed here beyond removing `stub_action_servers` from the launch file.

- [ ] **Step 2: Implement real `VehicleStatusOK`**

Replace the `py_trees.behaviours.Success(name="VehicleStatusOK (stub)")` in `mission_tree.py` with a real `py_trees.behaviour.Behaviour` checking, at minimum: Nav2's lifecycle manager reports all managed nodes active (check `nav2_lifecycle_manager`'s own status topic/service — `ros2 service list | grep lifecycle` once Task 6 is running, to find the right interface), and slam_toolbox is publishing `map→odom` recently (TF lookup with a short allowed age, e.g. reject if the last update is more than 2 seconds old). Return `SUCCESS` if all pass, `FAILURE` otherwise — this feeds Mission's existing safe-stop branch, already built in the earlier `mission` package work.

- [ ] **Step 3: Implement real `Localize`**

Replace the `Localize` stub similarly — a TF lookup confirming `map→base_link` resolves and is recent. Keep this one simpler/narrower than `VehicleStatusOK` (it's specifically about "are we localized," not general health).

- [ ] **Step 4: Update `launch/mission.launch.py`** to stop launching `stub_action_servers` (comment out or remove that `Node(...)` entry, with a comment explaining it's superseded by Tasks 5-6's real servers, not removed because it's broken).

- [ ] **Step 5: Verify the existing mission test suite still passes for what's unaffected**

```bash
colcon build --packages-select mission
source install/setup.bash
python3 -m pytest src/mission/test/ -v
```
Expected: `test_stub_action_servers.py` still passes in isolation (the stub code itself still works, it's just not launched by `mission.launch.py` anymore) — if you deleted rather than kept the stub code, this task's Files section instruction wasn't followed; go back and keep it. `test_mission_tree.py`'s structural test (which builds its own `StubActionServers` directly, not through the launch file) should be unaffected either way.

- [ ] **Step 6: Commit**

```bash
git add src/mission
git commit -m "Point mission at the real NavigateToPose/DockRobot servers, implement real health checks"
```

---

## Task 8: Full-stack integration launch + first live test

**Files:**
- Create: `src/mission/launch/full_stack.launch.py` (or wherever makes sense — a top-level launch bringing up: the einride-mini-truck sim, localization (EKF + slam_toolbox localize-mode), perception (AprilTag + docking), navigation, and mission, all together)

**Interfaces:**
- This is the integration point — no new code interfaces, just wiring everything built in Tasks 1-7 into one launch and testing the real thing end to end.

- [ ] **Step 1: Write the combined launch file**

Use `IncludeLaunchDescription` to pull in: `einride_mini_truck_bringup`'s `simulation.launch.py`, `localization`'s EKF + `localize.launch.py` (not `mapping.launch.py` — mapping was a one-time data-collection step, already done in Task 2), `perception`'s `apriltag.launch.py` + `docking.launch.py`, `navigation`'s `navigation.launch.py`, and `mission`'s `mission.launch.py`.

- [ ] **Step 2: Start the local Saga AI mock server and start an event**

```bash
cd /home/albin/ai-powered-autonomous-challenge-server
export PATH="$HOME/.local/bin:$PATH"
uv run python -m app
```
`curl -u admin:<password> -X POST http://localhost:8000/admin/event/start` (same approach as the mission package's own end-to-end verification).

- [ ] **Step 3: Launch the full stack and watch it try to run a real route**

```bash
ros2 launch mission full_stack.launch.py saga_token:=<truck 1's token>
```
Watch logs and (if not headless) RViz/Gazebo's own view. Expected failure modes to fix, in roughly the order they'll likely bite: TF tree not fully connected (missing static transforms, frame name mismatches between packages — check `ros2 run tf2_tools view_frames` or `ros2 run tf2_ros tf2_echo map base_link`), Nav2 rejecting goals (costmap/footprint misconfiguration from Task 6), docking failing to converge (staging pose sign error from Task 5), AprilTag not detecting at the actual approach distance/angle (camera FOV or tag size mismatch from Task 4). Fix root causes, not symptoms — if a quick parameter tweak doesn't obviously fix something, trace it back through TF/topic echoes to find out what's actually wrong before changing more parameters.

- [ ] **Step 4: Iterate until a full route completes for real**

Using the same admin-action curl sequence the `mission` package's own end-to-end verification established (`arrived_at_source`, `loading_complete`, `arrived_at_destination`, `unloading_complete` — though now the robot should actually be physically arriving, not just a stub reporting success instantly), drive at least one complete dock→load→dock→unload cycle with the robot genuinely moving and docking under its own navigation. Capture real log excerpts as evidence, the same way the `mission` package's own Task 8 did.

- [ ] **Step 5: Clean up all processes**

```bash
pgrep -fa "gz sim|ros_gz_bridge|mission_node|stub_action_servers|slam_toolbox|ekf_node|apriltag|docking_server|bt_navigator|controller_server|planner_server|collision_monitor|lifecycle_manager|uv run python -m app"
```
Kill everything found, confirm with a second `pgrep` that nothing remains.

- [ ] **Step 6: Commit**

```bash
git add src/mission/launch/full_stack.launch.py
git commit -m "Add full-stack integration launch, verified end-to-end in Gazebo"
```

---

## Self-Review Notes

- **Spec coverage:** every package in the spec (localization, perception/docking, navigation, mission updates) has a task; the "deferred to real hardware" items are explicitly excluded from scope (no tasks attempt them) and called out in Global Constraints so an implementer doesn't accidentally try to install Isaac ROS.
- **Placeholder scan:** no bare TODOs; every task ends in a concrete runnable verification command with an expected observable result.
- **Ambiguity acknowledged, not hidden:** staging-pose sign convention (Task 5), exact slam_toolbox param file naming per installed version (Tasks 2-3), and planner choice (navfn vs smac, Task 6) are explicitly left to implementer judgment with a clear fallback/first-choice stated, rather than asserting exact values I can't verify without the package actually running.
- **Known risk:** this plan leans on implementers reading each package's own shipped example configs rather than me hand-writing every YAML key, since getting Nav2/slam_toolbox parameter specifics exactly right from memory is less reliable than adapting the package's own maintained examples — this is intentional, not a gap, per the spec's stated testing approach of iterating against real behavior rather than a priori correctness.

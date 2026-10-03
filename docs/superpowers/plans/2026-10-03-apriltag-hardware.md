# AprilTag Detection on the Real Robot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run AprilTag detection (rectified, wall-clock) during a hardware mapping run so `ros2 run perception capture_dock_pose` works on the real robot.

**Architecture:** `perception/launch/apriltag.launch.py` gains arguments (`use_sim_time`, `image_topic`, `rectify`) whose defaults reproduce the simulator graph; with `rectify:=true` it also starts `image_proc`'s `rectify_node` and the detector reads the rectified topic. `localization/launch/mapping_mission_hardware.launch.py` includes it behind an `apriltag` argument (default true) inside a 5 s `TimerAction`, found by share-directory lookup at runtime (no `localization -> perception` package dependency: that would be a cycle).

**Tech Stack:** ROS 2 Jazzy, `launch` / `launch_ros` (Python launch files), `apriltag_ros` 3.4.0, `image_proc` 5.0.13, pytest, colcon (ament_python).

**Spec:** `docs/superpowers/specs/2026-10-03-apriltag-hardware-design.md`

## Starting state (read this first)

Branch `apriltag-hardware`, HEAD `64c6c3f` (the spec). The working tree holds UNCOMMITTED prototype work that is the deliberate starting point. Do NOT `git checkout`, `git stash`, `git restore` or `git clean` anything.

| file | state |
| --- | --- |
| `src/perception/launch/apriltag.launch.py` | modified (prototype of spec section 1) |
| `src/perception/config/apriltag.yaml` | modified (`use_sim_time` line removed) |
| `src/perception/test/launch_helpers.py` | untracked (in-process launch evaluation helpers) |
| `src/perception/test/test_apriltag_launch.py` | untracked (9 test cases) |
| `src/localization/test/test_mapping_mission_hardware_launch.py` | untracked (5 tests; 4 fail until Task 2) |

## Global Constraints

- Tags: family `36h11`, `size: 0.100` (10.0 cm edge), ids 0-7 = docks A-H (unchanged from sim).
- `apriltag.launch.py` arguments and defaults, verbatim: `use_sim_time` = `true`, `image_topic` = `/oak/rgb/image_raw`, `rectify` = `false`. No `camera_info` argument: image_transport always uses `<image_topic's namespace>/camera_info`. Sim callers (no arguments) must get exactly today's graph.
- `rectify:=true`: node `rectify_node` (package `image_proc`, executable `rectify_node`), `image` -> `image_topic`, `image_rect` -> `<image_topic's namespace>/image_rect`; apriltag node's `image_rect` -> that rectified topic, no `camera_info` remapping.
- `use_sim_time` reaches nodes as a typed bool, never the string (rclpy declares it as bool; a string stops the node starting).
- `apriltag.yaml` must not contain `use_sim_time`; `image_transport: raw`, `qos_profile: default`, `detector.threads: 1`, `detector.decimate: 1.0` are NOT changed in this work.
- Hardware launch: argument `apriltag` default `true`; include perception's `apriltag.launch.py` with `use_sim_time:=false`, `image_topic:=/oak/rgb/image_raw`, `rectify:=true`, in a `TimerAction` of 5 s; `apriltag:=false` is the lidar-only run. This launch does not start the camera (the vendor's `hardware.launch.py` owns it).
- No `localization/package.xml` change and no `localization -> perception` dependency (cycle: `perception` exec_depends on `localization`). `perception/package.xml` gains `<exec_depend>image_proc</exec_depend>`.
- Robot install line: `sudo apt install ros-jazzy-apriltag-ros ros-jazzy-image-proc`.
- `perception/test/test_dock_capture.py` must pass unchanged. No process is launched, no Gazebo, no `ros2 launch` in any automated step (project rule: the user runs the simulation and robot, not agents).
- Commits: Conventional Commits, no `Co-Authored-By` or other AI attribution trailer.
- Run all commands from `/home/albin/modysseus`. Every shell is fresh, so each command line below sources ROS itself.

## Review Focus

Failure modes the spec implies but a casual test set would miss; each is pinned by a test named in the owning task.

1. `rectify` / `use_sim_time` given in another casing (`True`, `TRUE`, `False`, `FALSE`, `1`, `0`): must behave like the lowercase form, and `use_sim_time` must still reach the node as a bool. Task 1 (`test_rectify_accepts_any_true_spelling`, `test_use_sim_time_false_spellings_reach_nodes_as_bool`).
2. An `image_topic` at the root namespace (`/image_raw`): the rectified topic must be `/image_rect`, not `//image_rect` or `image_rect`. Task 1 (`test_root_namespace_topic_rectifies_to_root_image_rect`).
3. The sim caller (`mapping_mission.launch.py`) passes no arguments and must keep today's graph (one node, raw topics, `use_sim_time` true, no rectify node). Task 1 (`test_sim_defaults_...`, already present); the hardware change must not alter it.
4. `apriltag:=False` (capital F) on the hardware launch must skip the include just like `false`. Task 2 (`test_apriltag_false_spelling_variants_skip_the_include`).
5. The hardware launch must still start exactly `wheel_odometry_node` itself on wall clock with the camera untouched (no camera or other node sneaks in; the apriltag nodes live only inside the include). Task 2 (`test_hardware_launch_still_runs_on_wall_clock_without_the_camera_nodes`, already present).

Known and accepted, not tested: any text that is not `true`/`1` is read as false by `_as_bool` (a typo such as `ture` silently means "false"). The spec is silent; the sim default is `true`, so this only bites someone typing the argument by hand. Left as is (YAGNI).

---

## File Structure

| file | responsibility |
| --- | --- |
| `src/perception/launch/apriltag.launch.py` | detector (+ optional rectify) launch; arguments |
| `src/perception/config/apriltag.yaml` | detector parameters (no clock setting) |
| `src/perception/test/launch_helpers.py` | in-process launch evaluation helpers (shared by both packages' tests by path) |
| `src/perception/test/test_apriltag_launch.py` | tests of `apriltag.launch.py` |
| `src/localization/launch/mapping_mission_hardware.launch.py` | real-robot mapping launch, gains the `apriltag` include |
| `src/localization/test/test_mapping_mission_hardware_launch.py` | tests of that launch |
| `src/perception/package.xml` | declares `image_proc` |
| `src/localization/launch/mapping_mission.launch.py`, `src/perception/perception/capture_dock_pose.py` | wording only |

How the launch tests work (so nobody has to rediscover it): `launch_helpers.load_launch_module` imports a `*.launch.py` by path; `make_context` builds a `LaunchContext` holding each `DeclareLaunchArgument`'s default then the test's overrides; `expand` runs every `OpaqueFunction` against that context; `describe_node` reads a `Node`'s package, executable, name, remappings and `use_sim_time` from its private (name-mangled) attributes, because `Node.node_name` and friends are only valid after execution. `TimerAction.period`, `.actions`, `.condition.evaluate(ctx)` and `IncludeLaunchDescription.launch_description_source.location` / `.launch_arguments` are public. The localization test loads perception's `launch_helpers.py` by relative path, because `localization` must not depend on `perception`.

Launch files resolve their config and each other through the INSTALLED share directory, so `colcon build` must precede the tests and be re-run after any launch/config change. Baseline before this plan: 175 tests pass with `/opt/ros/jazzy` and the workspace install sourced.

---

### Task 1: Finalize the rectifying `apriltag.launch.py` (prototype review, config, dependency)

**Files:**
- Modify (already modified, uncommitted): `src/perception/launch/apriltag.launch.py`, `src/perception/config/apriltag.yaml`
- Create (already present, untracked): `src/perception/test/launch_helpers.py`, `src/perception/test/test_apriltag_launch.py`
- Modify: `src/perception/package.xml`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces (Task 2 relies on these): `apriltag.launch.py` accepts launch arguments `use_sim_time`, `image_topic`, `rectify` (all text); `perception/test/launch_helpers.py` exports `load_launch_module(path)`, `make_context(description, **overrides)`, `expand(entities, context)`, `nodes_of(entities)`, `describe_node(node, context) -> {'package','executable','name','remappings','use_sim_time'}`, `describe_include(include, context) -> {'file','args'}`. The installed `perception` share copy of `apriltag.launch.py` (needs a build).

- [ ] **Step 1: Read the prototype against the spec**

Run: `git diff src/perception/launch/apriltag.launch.py src/perception/config/apriltag.yaml`
Check each against spec section 1 and Global Constraints: (a) three arguments with the exact defaults; (b) `rectify` false graph is today's (apriltag node only, `image_rect` -> raw topic); (c) rectify node params carry only a bool `use_sim_time`; (d) `use_sim_time` converted to bool via `_as_bool`; (e) `apriltag.yaml` lost only the `use_sim_time` line (other values byte-identical); (f) the "nothing consumes the detections" docstring is kept. The prototype was reviewed once against these and matches. The only off-spec findings are two comments in `apriltag.yaml` that are now wrong ("sim's OAK-D camera", "sim publishes raw"): fixed in Step 3. If anything else differs from the list above, fix it in the same step.

- [ ] **Step 2: Add the three review-focus tests (red)**

Append to `src/perception/test/test_apriltag_launch.py` (after `test_rectify_accepts_any_true_spelling`, before `test_params_file_no_longer_hardcodes_use_sim_time`):

```python
@pytest.mark.parametrize('spelling', ['false', 'False', 'FALSE', '0'])
def test_use_sim_time_false_spellings_reach_nodes_as_bool(spelling):
    nodes = _nodes(use_sim_time=spelling, rectify='true')

    assert nodes['apriltag_node']['use_sim_time'] is False
    assert nodes['rectify_node']['use_sim_time'] is False


def test_root_namespace_topic_rectifies_to_root_image_rect():
    nodes = _nodes(image_topic='/image_raw', rectify='true')

    assert nodes['rectify_node']['remappings']['image_rect'] == '/image_rect'
    assert nodes['apriltag_node']['remappings']['image_rect'] == '/image_rect'
```

- [ ] **Step 3: Fix the two stale comments in `apriltag.yaml`**

In `src/perception/config/apriltag.yaml` replace

```
# model's own built-in comment) from the sim's OAK-D RGB camera.
```
with
```
# model's own built-in comment) from the OAK-D RGB camera, simulated or real.
```
and replace
```
        image_transport: raw     # sim publishes raw (uncompressed) images
```
with
```
        image_transport: raw     # sim and the real driver publish raw images
```
Change nothing else in the file.

- [ ] **Step 4: Declare the dependency**

In `src/perception/package.xml`, after the line `<exec_depend>localization</exec_depend>` add:

```xml
  <!-- apriltag.launch.py starts image_proc's rectify_node on the real robot
       (rectify:=true). apriltag_ros normally pulls image_proc in already; it
       is declared here because this package's own launch file names it. -->
  <exec_depend>image_proc</exec_depend>
```

- [ ] **Step 5: Build, then run the launch tests**

Run: `source /opt/ros/jazzy/setup.bash && colcon build --packages-select perception && source install/setup.bash && python3 -m pytest -q src/perception/test/test_apriltag_launch.py`
Expected: `14 passed` (the prototype's 9 cases plus the 5 added in Step 2: 4 `use_sim_time` spellings and 1 root-namespace). `0 failed`. If a test fails, the launch file (not the test) is wrong unless the test contradicts a Global Constraint.

- [ ] **Step 6: Run the whole perception suite**

Run: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 -m pytest -q src/perception/test`
Expected: `107 passed` (the 102 from the prototype run plus the 5 added cases), `test_dock_capture.py` unchanged and green.

- [ ] **Step 7: Commit**

```bash
git add src/perception/launch/apriltag.launch.py src/perception/config/apriltag.yaml \
        src/perception/test/launch_helpers.py src/perception/test/test_apriltag_launch.py \
        src/perception/package.xml
git commit -m "feat(perception): rectify then detect, with clock and topics as launch arguments"
```
(Stage by explicit path; never `git add -A`: `src/localization/test/test_mapping_mission_hardware_launch.py` belongs to Task 2.)

---

### Task 2: Hardware mapping launch runs AprilTag detection

**Files:**
- Modify: `src/localization/launch/mapping_mission_hardware.launch.py`
- Test (already present, untracked; add one test): `src/localization/test/test_mapping_mission_hardware_launch.py`

**Interfaces:**
- Consumes: Task 1's `apriltag.launch.py` arguments and the INSTALLED perception share copy (the test evaluates `include['file']`, which is `share/perception/launch/apriltag.launch.py`), and `perception/test/launch_helpers.py` (loaded by relative path from localization's test).
- Produces: launch argument `apriltag` (default `true`) on `mapping_mission_hardware.launch.py`.

The existing test file already matches the spec exactly (period 5.0, `IfCondition`, the four include arguments, rectified nodes on wall clock, own nodes only `wheel_odometry_node`), so no test needs changing; it is committed with this task and gains the one extra review-focus test below.

- [ ] **Step 1: Add the review-focus test**

Append to `src/localization/test/test_mapping_mission_hardware_launch.py` (after `test_apriltag_false_skips_the_include`):

First add `import pytest` to the file's import block (below `import os`, above the `from launch.actions ...` line), then append:

```python
@pytest.mark.parametrize('spelling', ['False', 'FALSE', '0'])
def test_apriltag_false_spelling_variants_skip_the_include(spelling):
    description, context = _description_and_context(apriltag=spelling)

    (timer,) = _apriltag_timers(description)

    assert timer.condition.evaluate(context) is False
```
- [ ] **Step 2: Run to verify red**

Run: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 -m pytest -q src/localization/test/test_mapping_mission_hardware_launch.py`
Expected: FAIL. `test_hardware_launch_still_runs_on_wall_clock_without_the_camera_nodes` passes (pins existing behaviour); the others fail with `ValueError: not enough values to unpack (expected 1, got 0)` from `(timer,) = _apriltag_timers(description)`, because there is no apriltag timer yet. Expected summary: `7 failed, 1 passed` (the 4 original failing tests plus the 3 spelling cases fail; the one existing-behaviour test passes).

- [ ] **Step 3: Implement the launch**

Edit `src/localization/launch/mapping_mission_hardware.launch.py`.

3a. Imports. Replace
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
```
with
```python
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
```

3b. Body. Replace the block from `    return LaunchDescription([` to the end of the file with:

```python
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
            PathJoinSubstitution([FindPackageShare('perception'),
                                  'launch', 'apriltag.launch.py'])),
        launch_arguments={
            'use_sim_time': USE_SIM_TIME_ARG,
            'image_topic': '/oak/rgb/image_raw',
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
```

3c. Docstring. Apply these exact replacements in the module docstring.

(i) Title line: replace
```
"""Mapping mission on the REAL robot: wheel + gyro fusion + SLAM in mapping mode.
```
with
```
"""Mapping mission on the REAL robot: wheel + gyro fusion + SLAM + AprilTag capture.
```

(ii) Replace the whole section from `LIDAR ONLY. NO APRILTAG, DELIBERATELY.` through `so it is left out rather than half-wired.` (the paragraph ending "Mapping does not depend on it, so it is left out rather than half-wired.") with:
```
APRILTAG DETECTION, FOR DOCK CAPTURE. `mapping_mission.launch.py` runs
apriltag_ros alongside mapping so one drive both builds the map and captures
the arena's real dock poses; so does this file, 5 s after SLAM starts (so it
does not compete with slam_toolbox's CONFIGURE/ACTIVATE window). It includes
`perception`'s `apriltag.launch.py` with `use_sim_time:=false`,
`image_topic:=/oak/rgb/image_raw` and `rectify:=true`: the real OAK-D RGB image is distorted and apriltag_ros
ignores distortion, so `image_proc`'s rectify_node runs first and the detector
reads `/oak/rgb/image_rect`. Pass `apriltag:=false` for the lidar-only run
this launch used to be (e.g. if the Jetson cannot keep up). The camera driver
itself is the vendor's, started by `hardware.launch.py`.

  Needs `sudo apt install ros-jazzy-apriltag-ros ros-jazzy-image-proc` on the
  robot. The tag-orientation convention in `perception/perception/
  dock_capture.py` has never been checked against a live tag; the first
  capture on the robot is that check (see "STILL UNVERIFIED AGAINST A LIVE
  TAG" there and the dock-A advice in `mapping_mission.launch.py`).
```

(iii) Replace
```
NO SIM BRING-UP, AND NO DEVICE DRIVERS. This launch starts odometry and SLAM,
nothing else. The robot's own drivers - the serial link that produces
`/wheel_encoders` and accepts `/cmd_vel`, the lidar that produces `/scan`, and
RViz - come from the vendor's `hardware.launch.py`, which you are already
```
with
```
NO SIM BRING-UP, AND NO DEVICE DRIVERS. This launch starts odometry, fusion,
SLAM and (by default) AprilTag detection, nothing else. The robot's own
drivers - the serial link that produces `/wheel_encoders` and accepts
`/cmd_vel`, the lidar that produces `/scan`, the OAK-D camera that produces
`/oak/rgb/image_raw`, and RViz - come from the vendor's `hardware.launch.py`,
which you are already
```

(iv) Replace
```
  1. This launch - wheel odometry, gyro fusion and SLAM in mapping mode:
```
with
```
  1. This launch - wheel odometry, gyro fusion, SLAM in mapping mode and
     AprilTag detection:
```

(v) Replace
```
  3. Save the map when the drive is done, exactly as for a sim mapping run:

         ros2 run nav2_map_server map_saver_cli -f src/localization/maps/arena
```
with
```
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
```

- [ ] **Step 4: Rebuild and run to verify green**

Run: `source /opt/ros/jazzy/setup.bash && colcon build --packages-select perception localization && source install/setup.bash && python3 -m pytest -q src/localization/test/test_mapping_mission_hardware_launch.py`
Expected: `8 passed` (5 original + 3 spelling cases). The build is needed because the test evaluates perception's INSTALLED launch file and `ros2 launch` runs localization's installed copy.

- [ ] **Step 5: Run the whole localization suite**

Run: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 -m pytest -q src/localization/test`
Expected: all pass, no failures in `test_gyro_bias.py`, `test_scan_filter_config.py`, `test_wheel_odometry.py`.

- [ ] **Step 6: Commit**

```bash
git add src/localization/launch/mapping_mission_hardware.launch.py \
        src/localization/test/test_mapping_mission_hardware_launch.py
git commit -m "feat(localization): run rectified AprilTag detection in the hardware mapping launch"
```

---

### Task 3: Wording fixes, full verification, push

**Files:**
- Modify: `src/localization/launch/mapping_mission.launch.py` (docstring lines 6-9)
- Modify: `src/perception/perception/capture_dock_pose.py` (docstring line 4-6, clock-mismatch message ~line 186-191)

**Interfaces:**
- Consumes: Tasks 1 and 2 (the hardware launch now runs AprilTag).
- Produces: nothing code-facing; wording only.

No new test: these are docstring and error-message text with no logic. `test_dock_capture.py` (which does not assert on the changed message; grep confirms no match for the clock-mismatch wording) and the full run below guard against accidents.

- [ ] **Step 1: Fix the stale sim docstring**

In `src/localization/launch/mapping_mission.launch.py` replace
```
THIS FILE IS THE SIMULATION ENTRY POINT. On the real robot use
`mapping_mission_hardware.launch.py` instead, which starts the hardware
wheel-odometry node this file does not need (the simulator's DiffDrive plugin
provides `/odom` and the `odom -> base_footprint` TF) and leaves AprilTag
detection out, so it maps with the lidar only. The two files share
`mapping.launch.py`.
```
with
```
THIS FILE IS THE SIMULATION ENTRY POINT. On the real robot use
`mapping_mission_hardware.launch.py` instead, which starts the hardware
wheel-odometry and fusion nodes this file does not need (the simulator's
DiffDrive plugin provides `/odom` and the `odom -> base_footprint` TF) and
runs AprilTag detection too, rectifying the real camera's distorted image
first (its `apriltag` argument, default true). The two files share
`mapping.launch.py` and perception's `apriltag.launch.py`.
```

- [ ] **Step 2: Fix the capture tool wording**

In `src/perception/perception/capture_dock_pose.py` replace
```
`ros2 run perception capture_dock_pose` -- terminal 3 of the mapping-mission
workflow described at length in
`localization/launch/mapping_mission.launch.py`'s docstring (read that first;
it is the primary documentation for this feature).
```
with
```
`ros2 run perception capture_dock_pose` -- terminal 3 of the mapping-mission
workflow described at length in
`localization/launch/mapping_mission.launch.py`'s docstring (read that first;
it is the primary documentation for this feature). It runs the same way on the
real robot after `localization/launch/mapping_mission_hardware.launch.py`,
except that no `use_sim_time` flag is passed there: there is no /clock on
hardware and the default (false) is already right.
```
and replace
```
                f'apriltag_ros disagree about use_sim_time. Restart this node '
                f'with "--ros-args -p use_sim_time:=true" when running '
                f'against the simulator.')
```
with
```
                f'apriltag_ros disagree about use_sim_time. Restart this node '
                f'with "--ros-args -p use_sim_time:=true" when running '
                f'against the simulator, and with no such flag on the real '
                f'robot (mapping_mission_hardware.launch.py runs apriltag_ros '
                f'on wall-clock time).')
```

- [ ] **Step 3: Full build and full test run**

Run: `source /opt/ros/jazzy/setup.bash && colcon build --packages-select perception localization && source install/setup.bash && python3 -m pytest -q src/perception/test src/localization/test`
Expected: all pass, 0 failed; 107 perception tests (Task 1) plus the localization suite including the 8 hardware-launch tests; the criterion is zero failures, not the exact total. Build prints `Summary: 2 packages finished`.

- [ ] **Step 4: Confirm only intended files changed**

Run: `git status --short`
Expected: only the two wording files modified (everything else from Tasks 1-2 is committed; `build/`, `install/`, `log/` are ignored).

- [ ] **Step 5: Commit**

```bash
git add src/localization/launch/mapping_mission.launch.py src/perception/perception/capture_dock_pose.py
git commit -m "docs(perception): say the hardware mapping launch runs AprilTag and capture needs no sim flag there"
```

- [ ] **Step 6: Push (last step)**

Run: `git push -u origin apriltag-hardware`
Expected: branch published to `origin`; the robot then does `git pull` on `apriltag-hardware`.

---

## On-robot verification (manual, user runs; not part of any automated step)

Prerequisites on the robot: `git pull` on `apriltag-hardware`; `sudo apt install ros-jazzy-apriltag-ros ros-jazzy-image-proc`; `colcon build --packages-select perception localization`; `source install/setup.bash`; the vendor `hardware.launch.py` is already running.

1. `ros2 launch localization mapping_mission_hardware.launch.py`. Wait for "gyro bias calibrated" and slam_toolbox ACTIVE. `ros2 topic hz /oak/rgb/image_rect` should be near the camera rate (30 Hz). With a tag in view `ros2 topic hz /detections` is steady, and `ros2 topic echo --once /detections --field header.stamp` is within about 0.5 s of wall-clock now.
   - `/oak/rgb/image_rect` alive but `/detections` silent with a tag in view means a camera_info QoS mismatch: apriltag_node's `camera_info` subscription is reliable (`qos_profile: default`) and goes straight to the driver. Run `ros2 topic info -v /oak/rgb/camera_info`; if the publisher is best-effort, set `qos_profile: sensor_data` in `src/perception/config/apriltag.yaml`, rebuild and relaunch. (rectify_node adapts to the publisher's QoS and publishes reliable, so the image QoS is not the problem.)
   - Rates far below 30 Hz or slam_toolbox falling behind (Jetson CPU): raise `detector.decimate` (2.0) or lower `detector.threads` in `apriltag.yaml` as measured, or run with `apriltag:=false`.
2. `ros2 topic echo --once /oak/rgb/camera_info --field header.frame_id` prints `oak_rgb_camera_optical_frame`; `--field distortion_model` prints a model OpenCV accepts (`plumb_bob` or `rational_polynomial`). apriltag_ros reads its intrinsics from `P` (not `K`), so rectify-then-detect is already consistent and no relay is needed; just check that `P` in `/oak/rgb/camera_info` is non-zero and sane.
3. RViz, Fixed Frame `map`: a TF display shows `tag36h11:<id>` on the physical tag. Check its axes on the tag (z axis is the outward normal per `dock_capture.py`); this is the first live check of that convention.
4. Capture dock A first from about 0.6 m, square-on: `ros2 run perception capture_dock_pose` (no sim-time flag), type `i` + Enter. Then `git diff` in `~/modysseus`: a plausible pose on the arena wall, staging pose about 0.6 m out along the outward normal; compare with dock A's existing sim-derived entry in `docks.yaml`. Mirrored, rotated by pi or elsewhere entirely means the convention or TF chain is wrong: stop, do not capture the rest.
5. Capture the other docks while driving the map, save the map, then commit the captures on the robot (`git add` + `git commit` in `~/modysseus`) before any later `git pull`. After step 4 passes, the "STILL UNVERIFIED AGAINST A LIVE TAG" note in `dock_capture.py` can be updated (separate follow-up).

---

## Self-review (against the spec)

- Spec 1 (`apriltag.launch.py` arguments, rectify graph, bool `use_sim_time`, yaml drop, docstring kept): Task 1, with the extra spelling/root-namespace tests.
- Spec 2 (`apriltag` argument, include args, 5 s stagger, docstring section replaced, terminal workflow with capture / save / git diff + commit on robot, LogInfo, camera not started): Task 2.
- Spec 3 (capture tool wording only, "UNVERIFIED" note left until the robot check): Task 3 plus the verification section's last line.
- Spec 4 (`exec_depend image_proc`, apt line, no localization change): Task 1 Step 4, Task 2 docstring, Global Constraints.
- Spec Testing (sim defaults, hardware arguments, custom topics, hardware launch on/off, `use_sim_time` false on nodes, yaml has no `use_sim_time`, `test_dock_capture.py` unchanged): all present in the two test files; Task 3 runs the full suite.
- Spec Verification and Risks (QoS, rate, P sanity, camera_info QoS, distortion model, tag orientation, pull conflicts): the On-robot section, including the conditional fixes.
- Out of scope respected: no detections wiring, no vendor changes, no default tuning.
- Placeholder scan: none; every code step shows the code, every replace shows exact old and new text.
- Type consistency: argument names, `describe_node` / `describe_include` keys and `USE_SIM_TIME_ARG` match across tasks and the existing tests.
- Test-count caveat: counts in the "Expected" lines are derived by reading the files; zero failures is the real criterion.

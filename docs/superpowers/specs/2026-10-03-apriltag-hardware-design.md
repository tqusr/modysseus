# AprilTag detection and dock capture on the real robot - design

Date: 2026-10-03. Branch: `apriltag-hardware` (off `full-stack`). Status: agreed design, not yet implemented.

## Goal

Run AprilTag detection on the real robot during a hardware mapping run, and make the existing interactive capture tool (`ros2 run perception capture_dock_pose`) usable on the robot, so one drive both builds the map and records where the docks physically are.

- Output of a capture is unchanged: the dock's line in `mission/config/docks.yaml` and in `perception/config/docking.yaml` is rewritten in the robot's own source tree, `~/modysseus/src`.
- Captures stay local on the robot. The user commits them there. The laptop never receives them automatically.
- Tags are the same as in simulation: family 36h11, 10.0 cm edge, ids 0-7 are docks A-H.
- Delivery: this work lives on `apriltag-hardware`; the robot pulls that branch.

## Context and verified facts

Everything below was read from the laptop repo, the vendor repo (`/home/albin/einride-mini-truck`), or the Jazzy packages installed in `/opt/ros/jazzy`, unless marked "check on the robot".

### Current laptop state

- `perception/launch/apriltag.launch.py` starts one `apriltag_ros` `apriltag_node` (name `apriltag_node`) with `apriltag.yaml` plus a hardcoded `{'use_sim_time': True}`, and remaps `image_rect` to `/oak/rgb/image_raw` and `camera_info` to `/oak/rgb/camera_info`. The sim image is fed straight in, unrectified, because the sim camera has no distortion.
- `perception/config/apriltag.yaml` also hardcodes `use_sim_time: true`. Other settings: `image_transport: raw`, `family: 36h11`, `size: 0.100`, `max_hamming: 0`, `qos_profile: default`, `detector.threads: 1`, `detector.decimate: 1.0`, `detector.refine: true`, `pose_estimation_method: pnp`, no per-tag overrides.
- `mapping_mission.launch.py` (sim) includes `mapping.launch.py` and, after a 5 s `TimerAction`, `perception`'s `apriltag.launch.py`. Its LogInfo tells the user to add `--ros-args -p use_sim_time:=true` to the capture tool in sim.
- `mapping_mission_hardware.launch.py` currently includes `mapping.launch.py` (use_sim_time false), starts `wheel_odometry_node` (TF off), and includes `localization.launch.py` (gyro bias node plus EKF, which owns `odom -> base_footprint`). It deliberately has no AprilTag; its docstring says so and calls a hardware AprilTag variant "its own task". That is this task.
- `localization/package.xml` does NOT depend on `perception` and must not: `perception` exec_depends on `localization`, so the reverse edge would be a cycle colcon refuses to order. The existing sim launch already includes perception's launch file by share-directory lookup at runtime, with no declared dependency. The hardware launch does the same; no `localization/package.xml` change.
- `perception/package.xml` already has `<depend>apriltag_ros</depend>`, `apriltag_msgs`, `tf2_ros`, `rclpy`, `opennav_docking`, `pcl_ros`, and `<exec_depend>localization</exec_depend>`. It has no `image_proc`.

### Capture tool on the robot

- `capture_dock_pose` does not set `use_sim_time`, so rclpy's default (false, wall clock) applies. On the robot nothing needs to be passed. It prints the effective value in its setup banner (`describe_setup`). The 1000 s clock-mismatch diagnosis exists for the opposite mistake.
- Workspace source lookup, `find_workspace_source_file(share_dir, relative_path)`: starts from `get_package_share_directory('perception')`, i.e. `~/modysseus/install/perception/share/perception`, and walks up at most 8 levels, testing `<dir>/src/<relative_path>` at each. The fourth level up is `~/modysseus`, so `~/modysseus/src/mission/config/docks.yaml` and `~/modysseus/src/perception/config/docking.yaml` are found. This works for the isolated colcon layout and for `--symlink-install`. If the lookup fails the node raises at startup with the `docks_file` / `docking_file` override parameters named. No change needed.
- Tag TF frames: `tag_tf_frame` builds `<detection.family>:<id>`, which is `tag36h11:0` ... `tag36h11:7`, taken from the message's own `family` field. This matches apriltag_ros's default naming (we set no `tag.frames`), and it is camera-independent.
- The tag TF's parent is the `frame_id` of the image apriltag_ros consumed. The tool looks up `map <- tag36h11:<id>` at the latest time, so it only needs the whole chain `map -> odom -> base_footprint -> ... -> camera optical frame` to exist. It never names a camera frame itself.
- Nothing in the tool assumes simulation. The remaining sim-flavoured text is in docstrings and LogInfo strings, not logic. The "STILL UNVERIFIED AGAINST A LIVE TAG" note in `dock_capture.py` stays until the on-robot check below passes.
- Default max detection age is 0.5 s, which at 30 Hz is fine; detection stamps are the image stamps, so a hardware driver whose stamps lag the ROS clock would show up as a stale-detection warning (see Verification step 1).

### Real camera (vendor repo)

- `oak_d_lite.yaml` (default `camera_config:=rgbd`): RGB 1280x720 at 30 Hz, `i_publish_tf_from_calibration: true`, `i_tf_base_frame: oak`, `i_tf_parent_frame: base_link`. Camera node name is `oak`.
- `docs/camera.md` and `docs/topics.md`: colour stream is `/oak/rgb/image_raw` with `/oak/rgb/camera_info`, `bgr8` on hardware (`rgb8` in sim), unrectified, with its own factory calibration, in frame `oak_rgb_camera_optical_frame`.
- Frame names match sim. Sim's `model.sdf` uses `optical_frame_id oak_rgb_camera_optical_frame`; on hardware, images are stamped with `<i_tf_base_frame>_rgb_camera_optical_frame`, and because the base frame is `oak` (same as the node name) the TF frame and the image `frame_id` agree. Hardware TF comes from the device EEPROM via the driver, not from `oak_calibration_tf`. `camera.md` documents that this agreement breaks if the base frame is renamed; we are not renaming it. Check on the robot: `ros2 topic echo --once /oak/rgb/camera_info --field header.frame_id`.
- Camera streams are NOT stated anywhere in the vendor docs as best-effort or reliable (only `/imu`, `/mag`, `/wheel_encoders` are documented best-effort). The depthai driver's image publisher QoS cannot be determined from the installed binaries. Check on the robot: `ros2 topic info -v /oak/rgb/image_raw`. See Risks.

### apriltag_ros 3.4.0 (installed at /opt/ros/jazzy, Jazzy apt)

- Package `apriltag_ros`, executable `apriltag_node` (also a composable `AprilTagNode` in `libAprilTagNode.so`). Both are present locally, so the existing launch file already matches reality.
- Topics (names read from the binary): subscribes `image_rect` and `camera_info`; publishes `detections` (`apriltag_msgs/AprilTagDetectionArray`). Remapping is by those relative names, as the launch file does today.
- Parameters present in the binary: `image_transport`, `family`, `size`, `max_hamming`, `qos_profile` (`default`, `sensor_data`, `system_default`), `detector.{threads,decimate,blur,refine,sharpening,debug}`, `pose_estimation_method`, and the optional `tag.ids` / `tag.frames` / `tag.sizes`.
- It uses only the camera matrix from `camera_info`. The binary contains no reference to distortion coefficients, and upstream's own example launch (`camera_36h11.launch.yml`) puts an `image_proc::RectifyNode` in front of the detector. So pose accuracy on a distorted image is wrong by an uncorrected amount, which is why rectification is needed for the real camera; sim has no distortion so it needed none.
- `apriltag_ros` 3.4.0's `package.xml` declares `image_proc` as an `exec_depend` for Jazzy, so apt installing apriltag_ros normally pulls image_proc already. We still declare it directly because our launch file names it.

### image_proc 5.0.13 (installed at /opt/ros/jazzy, Jazzy apt)

- Package `image_proc` (apt `ros-jazzy-image-proc`). Both forms exist: standalone executable `rectify_node` (`/opt/ros/jazzy/lib/image_proc/rectify_node`) and composable plugin `image_proc::RectifyNode`.
- Topics: subscribes `image` and `camera_info`, publishes `image_rect` (the same names the upstream `camera_36h11.launch.yml` uses). Encoding and header (including `frame_id`) are carried through, so the tag TF parent stays `oak_rgb_camera_optical_frame` and bgr8 is fine for apriltag_ros (cv_bridge).
- The OAK RGB distortion model, per `camera.md` for the mono pair, is `rational_polynomial`. `image_proc` rectifies through `image_geometry::PinholeCameraModel::rectifyImage`, which uses OpenCV and supports it. Whether the colour `camera_info` carries a distortion model OpenCV accepts is checked on the robot (Verification step 2).

## Design (approach A: rectify, then detect)

### 1. `apriltag.launch.py`

New launch arguments:

| argument | default | meaning |
| --- | --- | --- |
| `use_sim_time` | `true` | applied to the apriltag node and (if started) the rectify node. Default true keeps sim callers unchanged. |
| `image_topic` | `/oak/rgb/image_raw` | the raw image topic. |
| `camera_info_topic` | `/oak/rgb/camera_info` | the matching camera info. |
| `rectify` | `false` | when `true`, also start `image_proc`'s `rectify_node` and detect on its output. |

Behaviour:
- `rectify:=false`: exactly today's graph. `image_rect` is remapped to `image_topic`, `camera_info` to `camera_info_topic`.
- `rectify:=true`: a `rectify_node` (package `image_proc`, executable `rectify_node`, name `rectify_node`) with `image` remapped to `image_topic`, `camera_info` to `camera_info_topic`, and `image_rect` remapped to `<image_topic's namespace>/image_rect` (`/oak/rgb/image_rect` with the defaults). The apriltag node's `image_rect` is remapped to that rectified topic and its `camera_info` stays on `camera_info_topic`, so the detector reads the same `K` the rectify node was given (see Risks, "Rectified intrinsics").
- `use_sim_time` is a launch argument wired into both nodes as a typed bool via the `LaunchConfiguration` substitution (same bool-vs-string caution that `mapping_mission_hardware.launch.py` documents).
- `apriltag.yaml` drops its `use_sim_time: true` line; the launch argument is the single source. Tag family, size, decimation and the remaining detector settings are unchanged. `image_transport: raw` stays: the driver publishes raw topics.
- The big existing docstring about "nothing consumes the detections" stays true and is kept.

### 2. `mapping_mission_hardware.launch.py`

- New launch argument `apriltag` (default `true`). When `true`, the file includes `perception`'s `apriltag.launch.py` with `use_sim_time:=false`, `image_topic:=/oak/rgb/image_raw`, `camera_info_topic:=/oak/rgb/camera_info`, `rectify:=true`. When `false`, the run is lidar-only as today.
- Same stagger as the sim file: the include sits in a `TimerAction` (5 s), so it does not compete with slam_toolbox's CONFIGURE/ACTIVATE window.
- The module docstring's "LIDAR ONLY. NO APRILTAG" section is replaced. The terminal workflow becomes: (0) vendor `hardware.launch.py`, (1) this launch (wait for gyro bias calibrated and slam_toolbox ACTIVE), (2) teleop, (3) `ros2 run perception capture_dock_pose` on the robot, no sim-time flag, (4) save the map, (5) `git diff` and commit the captures on the robot. The LogInfo string is updated to match.
- The vendor's `hardware.launch.py` must already be running; it owns the camera driver, so this launch does not start the camera.

### 3. `capture_dock_pose`

No logic change expected, per the verified facts above. Docstring/LogInfo wording that implies sim-only is touched up where it would mislead on the robot. The module docstrings' "STILL UNVERIFIED AGAINST A LIVE TAG" note is updated after the on-robot check, not before.

### 4. Dependencies

- `perception/package.xml`: add `<exec_depend>image_proc</exec_depend>`. `apriltag_ros` is already declared.
- The robot needs: `sudo apt install ros-jazzy-apriltag-ros ros-jazzy-image-proc`. The laptop already has both.
- No `localization/package.xml` change (see Context: the cycle).

## Verification (manual, user runs on the robot)

Prerequisite: `git pull` on `apriltag-hardware`, the apt install above, `colcon build --packages-select perception localization`, then `source install/setup.bash`. The robot's own `hardware.launch.py` is already running.

1. `ros2 launch localization mapping_mission_hardware.launch.py`, wait for gyro bias calibrated and slam_toolbox ACTIVE. `ros2 topic hz /oak/rgb/image_rect` should show roughly the camera rate (see Risks if much lower). With a tag in view, `ros2 topic hz /detections` shows a steady rate; `ros2 topic echo --once /detections --field header.stamp` should be within about 0.5 s of the current wall-clock time (otherwise the capture tool will report stale detections).
2. `ros2 topic echo --once /oak/rgb/camera_info --field header.frame_id` should print `oak_rgb_camera_optical_frame`, and `--field distortion_model` should print a model OpenCV accepts (`plumb_bob` or `rational_polynomial`). `ros2 topic info -v /oak/rgb/image_raw` shows the publisher's reliability; compare with the subscribers (see Risks).
3. In RViz (Fixed Frame `map`), a TF display shows `tag36h11:<id>` for the visible tag. Check its axes on the physical tag. The pose-from-tag convention in `dock_capture.py` (z axis of the tag frame is the outward normal) has been derived offline and never compared with a live tag; this is the first such check.
4. Capture one dock from about 0.6 m square-on: `ros2 run perception capture_dock_pose`, type `i` + Enter. Then `git diff` in `~/modysseus`: the dock line should be a plausible pose, on the arena wall, with the staging pose about 0.6 m out along the outward normal. Capture dock A first and compare it with its existing sim-derived entry in `docks.yaml`.
5. Commit the captures on the robot (`git add` + `git commit` in `~/modysseus`), so a later `git pull` of this branch does not conflict.

## Testing (laptop, automated)

- Launch-description tests (plain pytest, no process launched, no Gazebo) that import `apriltag.launch.py` and generate its description:
  - sim defaults (no arguments): one `apriltag_node`, `use_sim_time` true, remaps `image_rect -> /oak/rgb/image_raw` and `camera_info -> /oak/rgb/camera_info`, no rectify node.
  - hardware arguments (`use_sim_time:=false`, `rectify:=true`): a `rectify_node` with the right remaps and a `false` `use_sim_time`, and an `apriltag_node` whose `image_rect` remap points at the rectified topic, with `use_sim_time` false.
  - `rectify:=false` with custom topics: remaps follow the arguments, no rectify node.
- A test of `mapping_mission_hardware.launch.py`: with the default `apriltag:=true` the description includes the perception launch with the hardware arguments; with `apriltag:=false` it does not. A third assertion checks `use_sim_time` is false on the apriltag nodes.
- The existing `perception/test/test_dock_capture.py` must still pass unchanged.
- Also checks that `apriltag.yaml` no longer contains `use_sim_time`.
- No live run is possible on the laptop for the hardware path; the Verification checklist above covers it.

## Out of scope

- Wiring detections into docking, localization or the mission (still nothing consumes `/detections`).
- Changing the vendor repo or the camera driver configuration (read-only here).
- Approach C: on-device OAK AprilTag, which needs vendor driver changes and has no apriltag_ros TF output.
- Camera calibration beyond what the OAK EEPROM provides.
- Pushing captures from the robot to the laptop, or any merge flow for them.

## Rejected alternatives

- B: detect on the raw, distorted image with no rectify node. Simpler, but apriltag_ros ignores distortion, so tag poses carry a systematic error that the real lens makes non-trivial. Kept only as the `rectify:=false` fallback, and as what sim runs.
- C: on-device (OAK) AprilTag. Requires vendor driver changes and gives no apriltag_ros `tag36h11:<id>` TF, which the capture tool consumes.

## Risks

- Jetson CPU cost. Rectify plus detect on 1280x720 at 30 Hz, on top of slam_toolbox, EKF and the driver, may not keep up; rectify's full-frame remap and the apriltag quad detector at `decimate: 1.0` with one thread are the main costs. Symptoms: `/oak/rgb/image_rect` or `/detections` well below 30 Hz, or slam_toolbox falling behind. Detection of a ~10 cm tag at ~0.6 m does not need 30 Hz. Mitigation knobs, to be applied in `apriltag.yaml` only if the measured rate on the robot demands it: `detector.decimate` (2.0 is upstream's example default) and `detector.threads`. Defaults are NOT changed in this work, because nothing here justifies it yet; `apriltag:=false` is the lidar-only escape hatch.
- Camera QoS mismatch. If `ros2 topic info -v /oak/rgb/image_raw` shows a best-effort publisher, a reliable subscriber (apriltag's `qos_profile: default`, and the rectify node's defaults) receives nothing. Symptom: `/detections` and `/oak/rgb/image_rect` silent while the camera topic is alive. The fix is to set `qos_profile: sensor_data` on the apriltag node via `apriltag.yaml`; the rectify node's subscription QoS is checked at the same time. This is deliberately a robot-side finding, not a guess made here.
- Rectified intrinsics. After rectification the effective pinhole intrinsics are the `P` matrix of the rectified camera, while apriltag_ros reads `K` from the `camera_info` it is given. The rectify node does not republish a modified camera_info, so if `P` differs noticeably from `K` for this camera, tag range will be slightly off. Check on the robot by comparing `K` and `P` in `/oak/rgb/camera_info`; if they differ materially, the fix is a `camera_info` relay that publishes `P` as `K`, in a follow-up rather than speculatively now.
- First live check of the tag-orientation convention. Everything in `dock_capture.py` about which tag axis is the outward normal is derived offline; a wrong convention would write wrong dock poses that still look numerically fine. Verification steps 3 and 4 exist to catch exactly that, and dock A (whose sim-derived entry exists) is the cross-check.
- Pull conflicts. Captures edit tracked files on the robot; committing them there before pulling avoids conflicts in `docks.yaml` and `docking.yaml`.

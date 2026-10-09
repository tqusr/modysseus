# OAK-D obstacle cloud investigation: handoff brief

For a Claude agent running on a separate, more powerful machine. It has SSH
access to the robot (a Jetson, user `jetson`, hostname/IP: ask the user).
Work happens on both machines:

- **Heavy work (your machine):** offline analysis of recorded data, prototyping
  filters, numpy/open3d/matplotlib plots, writing the candidate node.
- **Robot (via SSH):** anything that needs the live camera or the live ROS graph:
  recording bags, `ros2 param set`, `ros2 topic hz`, running the candidate
  node against live data, publishing to Foxglove. Run these yourself over SSH.
  Do not ask the user to run them unless SSH fails.

## 1. Problem

The OAK-D Lite is the only sensor that sees obstacles below the lidar plane
(about 12 cm). Its cloud is too sparse and too slow. It does not cover a whole
object, so the robot clips the object's edges while driving past or around it.

Test setup: one object about **30 cm in front of the camera, placed in a corner**.
The robot is **stationary** during all tests.

Goal: find the setup (camera parameters, our own processing, or both) that gives
a **complete, stable cloud of the object** at an acceptable rate. Deliver a
ranked recommendation with evidence. Do not land changes without the user's say.

## 2. Hard constraints

1. **Do not edit `~/einride-mini-truck`** (the camera and driver repo, on the
   robot at `/home/jetson/einride-mini-truck`). Its working tree has uncommitted
   edits by the user; leave them alone. Read it freely for context.
2. Prefer changes in `~/modysseus` (our repo). Changes there must be "standard
   and limited": parameter changes, or a small node in `src/perception`.
3. The camera is on **USB 2 (480 Mbit/s)**, confirmed with `lsusb -t`. This cannot
   be fixed in hardware. Treat bandwidth as a fixed budget.
4. Runtime camera parameter changes with `ros2 param set` are **approved**, but
   they are temporary. Record the original values first and restore them when
   done. If a change wedges the camera, `sudo systemctl restart einride-mini-truck`
   restores the baseline (the service also owns the lidar and drive base, so
   only do this while the robot is stationary).
5. Keep the robot stationary. Do not publish `/cmd_vel` or start navigation.
6. Put recordings and scratch files in a new directory, e.g.
   `~/oak_investigation/` on the robot. Do not write into either repo, except
   the final candidate node, and only after the user agrees.

## 3. What is already known (verified from the repos and the live system)

**Camera driver** (`einride_mini_truck_bringup/config/oak_d_lite.yaml`, node `/oak`,
RGBD pipeline). Changed 2026-10-08 to cope with USB 2:

- Depth is **unaligned**, 320x240 `16UC1` mm, in `oak_right_camera_optical_frame`.
  It is the 640x480 mono pair halved by the ROBOTICS preset's decimation.
  Topics: `/oak/stereo/image_raw`, `/oak/stereo/camera_info`.
- Colour is cut to 640x360. The device streams colour even when nothing subscribes.
- Stereo: `i_resolution: 480P`, `i_subpixel: true`, `i_depth_preset: ROBOTICS`,
  `i_depth_filter_size: 7` (7x7 median), `i_stereo_conf_threshold: 245`,
  `i_fps: 30`, `i_align_depth: false`.
- ROBOTICS enables the spatial filter (delta 20, hole-fill 2) and the speckle
  filter (range 200), and clamps depth to 0 to 10 m. These are applied below the
  ROS parameter layer and do not show in `ros2 param get`. `camera.i_pipeline_dump:=true`
  writes the exact StereoDepth config to `/tmp/<device-id>_pipeline.json`.
- The nominal minimum depth is 0.2 m. An object at 30 cm is near the stereo
  minimum range, so close-range blind spots or holes are likely. Check whether
  extended disparity is available in the driver (`i_extended_disp` or similar,
  check the installed driver's params) and what it costs in range and rate.
- `docs/camera.md` in that repo records the measurements. Reported on USB 2:
  aligned 1280x720 depth ran at about 5 Hz with 200 ms latency, and unaligned
  320x240 depth plus 640x360 colour runs at about 30 Hz with about 60 ms latency.
  Re-measure; do not trust it.

**Our pipeline** (`~/modysseus/src/perception/launch/oak_points_filter.launch.py`,
`config/oak_points_filter.yaml`): `depth_image_proc::PointCloudXyzNode` turns
`/oak/stereo/image_raw` into `/oak/points_filtered` (XYZ, 76,800 points, NaN
for empty pixels, `queue_size: 1`). Wall-clock time throughout.

**Consumers** (`src/navigation/config/nav2_params.yaml`):

- Costmap `camera_layer` (local and global): ObstacleLayer on `/oak/points_filtered`,
  `min_obstacle_height: 0.05`, `max_obstacle_height: 0.35`, `obstacle_max_range: 2.5`,
  `obstacle_min_range: 0.0`, `clearing/marking: true`.
- `collision_monitor` source `oak_cloud`: same height limits, `source_timeout: 2.0`
  (older clouds are re-projected through TF with `base_shift_correction`).
- The 0.05 m minimum height exists because floor noise at lower thresholds made
  the planner unstable. Do not lower it without checking that.
- Costmap `robot_radius: 0.18`, `resolution: 0.05`. A hole smaller than a few
  cells matters little, but a missing object edge several cells wide matters.

**ROS environment on the robot:** `source /opt/ros/jazzy/setup.bash`,
`ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Shared memory does not work there (silent
failure), so large messages cost CPU. Export `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` in
every shell you use. The camera, lidar and base run as the systemd service
`einride-mini-truck`, and `foxglove_bridge` runs as `einride-mini-truck-foxglove`
on **ws://<robot>:8765**. Its config is `/etc/default/einride-mini-truck`. Check
both are active before starting. A stale `ros2 daemon` can show an empty topic
list, so try `ros2 daemon stop; ros2 daemon start`.

## 4. Hypotheses to test

Rank by evidence, not intuition. Your first job is to find out which of these is
actually responsible.

- **H1: Holes from on-device filtering.** The 7x7 median, confidence 245,
  speckle and spatial filters drop pixels on low-texture surfaces. The cloud
  then covers only the textured part of the object.
- **H2: Close range.** At 30 cm the disparity is at or beyond the search range
  of the 480P stereo pair. The object is partly invalid or clipped.
- **H3: Frame rate or jitter.** The real rate or its spread is worse than 30 Hz
  (USB 2, CPU load in the camera container, DDS UDP fragmentation). Measure the
  rate, the gaps and the latency (`now - header.stamp`).
- **H4: Our height gate or range gate** trims the object. Compare the object's
  extent in the raw cloud against what survives 0.05 to 0.35 m, in the
  `base_footprint` frame. Check the camera's mount pitch and height.
- **H5: Temporal sparsity.** Single frames are incomplete, but the union over
  the last N frames is complete (valid because the robot is stationary now).
  Moving later needs TF motion compensation.

## 5. Plan

### Step 0: Baseline (robot, SSH)

1. Confirm the services are active and record the original camera parameters:
   `ros2 param dump /oak > ~/oak_investigation/oak_params_original.yaml`.
2. Measure `ros2 topic hz /oak/stereo/image_raw`, plus the same for
   `/oak/points_filtered` and `/oak/rgb/image_raw`. Measure the latency
   distribution over 30 s (write a tiny subscriber, or use `ros2 topic delay`).
3. Check `top` for the `oak_container` CPU share and check the USB speed
   (`lsusb -t` shows 480M for the device).
4. Record a baseline bag, 15 s, with the object in place:
   `ros2 bag record -o ~/oak_investigation/baseline /oak/stereo/image_raw /oak/stereo/camera_info /tf /tf_static /oak/rgb/image_raw`.
   Include the colour image so you can see what the object looks like.
5. Copy the bag to your machine (`scp -r`).

### Step 1: Offline analysis (your machine)

- Load the depth frames. Report per frame: valid-pixel ratio in the object
  region, fill ratio over time, and which pixels flicker. A min / median / max
  of the valid count across frames shows whether the holes are random (frame
  union helps) or fixed (filters or range must change).
- Reconstruct the XYZ cloud with the `camera_info` intrinsics, transform to
  `base_footprint` with the bag's TF, and apply the Nav2 height gates. Show
  what is lost at each stage (raw, then valid, then height-gated, then range-gated).
- Prototype and compare candidate processing on the recorded frames:
  - temporal union or max-hold over N = 2..8 frames, with the object's coverage
    and the floor-noise leakage measured for each N;
  - depth-image hole fill and dilation (small kernel);
  - morphological closing on the valid mask;
  - an optional statistical or radius outlier filter, to counter the extra noise.
  Metric: object coverage (fraction of the object's bounding-box area that has
  points) versus false points above 0.05 m on the floor and walls, and added
  latency.

### Step 2: Live camera parameter sweep (robot, SSH)

Change one parameter at a time with `ros2 param set /oak ...` (or the node name
the driver actually uses; check `ros2 node list`). **Read each value back, and
check that the topic keeps flowing.** Some `i_*` parameters are startup-only and
will be rejected or silently ignored. Record which ones actually take effect.
Candidates:

- confidence threshold lower than 245 (more pixels pass, more noise);
- median filter size (7x7 to 5x5 to 3x3 to off);
- speckle and spatial filter enable or range;
- extended disparity, if exposed (helps close range);
- subpixel on or off;
- depth range (min / max threshold) if exposed.

For each setting: record 10 s, and report the rate, the latency and the
object's valid-pixel coverage. Restore the originals after the sweep (from
`oak_params_original.yaml`, and verify with `ros2 param get`).

If a needed setting is startup-only, the fallback is to restart the driver
against a modified copy of the params file kept **outside** the repo (for
example in `~/oak_investigation/`). Ask the user before stopping the service,
and always restore it with `sudo systemctl restart einride-mini-truck`.

### Step 3: Foxglove visualisation (robot to the user's screen)

The user wants to see the cloud. Publish each variant as its own PointCloud2
topic with a clear name (`/oak_test/raw`, `/oak_test/accum_n3`, ...), in a
frame that is in TF (use `oak_right_camera_optical_frame` or transform to
`base_footprint`) and stamped with the current time. `foxglove_bridge` already
runs on port 8765 and exposes every topic, so no new service is needed. The
user opens Foxglove at `ws://<robot>:8765`, adds a 3D panel and selects the topics.

Tell the user which topics to turn on, and wait for them to confirm what they
see before concluding. Put a 3D panel with the object and the robot footprint
in the same frame. Publish at the camera rate and keep each cloud small
(76,800 points XYZ is 1.2 MB). Do not publish colour clouds over UDP.

### Step 4: Candidate node, live (robot)

Write the best candidate as a small ROS 2 node (Python or C++) in a scratch
location first. It takes `/oak/stereo/image_raw`, produces the improved cloud
on a **separate test topic**, never on `/oak/points_filtered`, so Nav2 is not
touched during testing. Run it on the robot and check CPU, rate and latency
with the robot idle. Then re-run the Foxglove comparison.

### Step 5: Report

One page for the user:

- the cause (which hypothesis held, with numbers);
- a ranked table of setups: coverage, rate, latency, false points, CPU;
- the recommended change, as a precise diff (camera param values for the user
  to apply in the driver repo themselves, and any change to our repo);
- what is not solved (for example motion compensation when driving, the 30 cm
  minimum range, USB 2 limits);
- the commands to reproduce each measurement.

## 6. Do not

- Edit `~/einride-mini-truck` or commit anything in `~/modysseus` without being asked.
- Leave camera parameters changed. Verify the restore.
- Drive the robot or start Nav2.
- Publish test clouds on `/oak/points_filtered`.
- Claim a cause without numbers from recorded data.

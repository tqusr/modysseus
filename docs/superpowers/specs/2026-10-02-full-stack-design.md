# Full autonomy stack — design

Date: 2026-10-02
Status: approved for implementation planning

## Goal

Complete the Modysseus ROS 2 stack beyond the `mission` package (already
built, merged): add Localization & Mapping, Perception & Docking, and
Navigation & Control, then prove the whole thing actually drives the
robot autonomously through a full Saga AI delivery loop in the
`einride-mini-truck` Gazebo simulation — launch it, watch it fail, fix it,
repeat until a route completes for real (not stubbed).

Hardware-only pieces (GPU-dependent Isaac ROS packages, real-robot
calibration, multi-robot interaction) are stubbed with simple, clearly
documented stand-ins — enumerated under "Deferred to real hardware" below —
so a teammate can swap them in later without re-deriving what's missing.

## Facts established before writing this plan (not assumptions)

Investigated directly in `/home/albin/einride-mini-truck`:

- **8 real docks exist in the sim world**, `einride_mini_truck_gazebo/worlds/arena.sdf`,
  with AprilTag markers (family `tag36h11`, 0.100 m, sized per the dock
  template's own comment for `apriltag_ros` — not an Isaac-specific tag
  size). World poses (x, y, yaw radians):
  ```
  A (2.992, -1.500, π)      E (-2.992,  1.500, 0)
  B (2.992,  1.500, π)      F (-2.992, -1.500, 0)
  C (1.500,  2.992, -π/2)   G (-1.500, -2.992, π/2)
  D (-1.500, 2.992, -π/2)   H (1.500,  -2.992, π/2)
  ```
  Tag IDs 0-7 for A-H (matches `docs/saga-ai.md` and `mission`'s existing
  `docks.yaml` letter→tag_id mapping already).
- **No GPU / Isaac ROS environment exists on this machine.** `isaac_ros_apriltag`,
  `isaac_ros_visual_slam`, `isaac_ros_dnn_stereo_depth`, `isaac_ros_object_detection`
  all require Jetson/CUDA and are out of reach here. `apriltag_ros` (CPU,
  standard ROS 2 package) replaces `isaac_ros_apriltag` — this is not a
  downgrade-only-for-sim choice, it's what the dock model's own tag size was
  built against, and it runs unchanged on the real robot too.
- **`/oak/points` (camera depth point cloud) and `/scan/points` (lidar point
  cloud) are already published by the simulation** — the "turn stereo depth
  into a point cloud" perception requirement is already satisfied upstream;
  our job is consuming it, not producing it.
- **`odom → base_footprint` is already published** by Gazebo's diff-drive
  plugin. Localization's real value-add is `map → odom` (slam_toolbox) and
  AprilTag-based correction, not reinventing basic odometry. `robot_localization`
  still earns its place fusing `/imu` for a cleaner `odom → base_link`, per
  the original spec.
- **Zero existing `robot_localization`/`slam_toolbox`/Nav2 config** anywhere
  in the vendor repo — all of it is new, ours to write.
- **The sim is single-robot only** — no namespacing/multi-spawn support.
  "Detect the other robots" / occupied-dock-by-another-truck logic has
  nothing to test against here.

## Package boundaries

Four new/changed ROS 2 packages, each its own colcon package under
`modysseus/src/`, overlaid the same way `mission` already is:

- `localization` — EKF, SLAM mapping+localization, AprilTag pose correction.
- `perception` — AprilTag detection, camera obstacle cloud relay/filter,
  `opennav_docking` server configuration.
- `navigation` — Nav2 bringup: costmaps, planner, controller, velocity
  smoother, collision monitor, `nav2_bt_navigator` (the real `NavigateToPose`).
- `mission` (existing, modified) — point the stub action names at the real
  servers once they exist; implement `VehicleStatusOK`/`Localize` for real.

## Shared interfaces (binding across packages, from the original hackathon plan)

- Frames: `map → odom → base_link`, plus sensor frames with measured/sim
  mounting positions (already in the URDF from `einride_mini_truck_description`).
- Topics: `/odometry/filtered` (EKF output), `/scan` (lidar), camera
  image/depth topics (already named above), `/oak/points` as the camera
  obstacle source into the costmap, `/cmd_vel` to the chassis.
- Velocity command order: Nav2 controller or docking server → velocity
  smoother → `nav2_collision_monitor` → `/cmd_vel`. Collision monitor is
  always last.
- Dock file: `modysseus/src/mission/config/docks.yaml` — already exists,
  gets its placeholder `0,0,0` poses replaced with the real `arena.sdf`
  values above as part of this work (letter → tag_id → dock_pose →
  staging_pose). Localization, perception/docking, and mission all read
  this same file; nobody else gets to maintain a second copy.
- Mission talks to the rest of the stack **only** through `NavigateToPose`
  and `DockRobot` actions — unchanged contract, just backed by real servers
  now instead of `mission`'s stubs.

## Localization & Mapping package

- **EKF** (`robot_localization`'s `ekf_node`): fuses `/imu` and the existing
  wheel-odometry-derived `odom→base_footprint` transform/topic, publishing
  a refined `odom→base_link` and `/odometry/filtered`.
- **Mapping**: `slam_toolbox` in mapping mode against `/scan`, run once
  against the Gazebo arena to produce a saved map (`.yaml`/`.pgm` pair) —
  this stands in for the real competition's "map the arena that morning"
  step, which is finals-day-only and out of scope here.
- **Localization**: `slam_toolbox` in localization mode against the saved
  map, publishing `map→odom`.
- **AprilTag correction**: uses perception's tag detections plus the known
  dock tag world poses (table above) as a localization sanity signal —
  exact mechanism (a correction service call vs. just cross-checking) is an
  implementation-time judgment call; perception must publish tag poses in
  `map` frame (via TF) for this to be usable regardless of the exact
  mechanism chosen.

## Perception & Docking package

- **AprilTag detection**: `apriltag_ros` subscribing to `/oak/rgb/image_raw`
  + its `camera_info`, tag family `tag36h11`, size `0.100` m (from the dock
  model). Publishes tag detections with pose, which TF makes available in
  `map` frame once localization is up.
- **Docking**: `opennav_docking`'s docking server, dock poses sourced from
  `docks.yaml` (now populated with real poses) cross-checked against live
  AprilTag detections for the final approach. This **is** the real
  `DockRobot` action server — `mission`'s stub retires once this exists.
- **Camera obstacles**: relay/pass through `/oak/points` into the
  costmap-ready frame/topic navigation's config expects (likely no
  transformation needed beyond what's already published — confirm against
  navigation's actual costmap layer config at implementation time).
- **Occupied-dock check**: implement the real logic (query whether another
  robot occupies the destination dock before entering) wired to a real topic
  interface, but since the sim has no second robot to ever report "occupied,"
  this is untestable here — document clearly, same hardcoded-"always free"
  behavior `mission`'s current stub already has for this exact reason.

## Navigation & Control package

- **Costmaps**: local + global, lidar obstacle layer (`/scan` or
  `/scan/points`) + camera obstacle layer (`/oak/points`), inflation layer,
  footprint sized for the mini-truck (dimensions in
  `einride_mini_truck_description`).
- **Planner**: `nav2_navfn_planner` (simplest, matches the plan's own
  documented fallback) or `nav2_smac_planner` if navfn proves insufficient
  for the arena's dock geometry — implementer's call, navfn first.
- **Controller**: `nav2_mppi_controller` + `nav2_rotation_shim_controller`
  (turn in place before driving off, per the original spec).
- **Safety chain**: velocity smoother → `nav2_collision_monitor` → `/cmd_vel`,
  stop/slowdown zones sized for the mini-truck's footprint.
- **`nav2_bt_navigator`**: provides the real `NavigateToPose` action —
  `mission`'s stub retires once this exists. Nav2's own default behavior
  tree is used as-is (explicitly out of scope to customize, per the
  original plan).
- **Lifecycle**: `nav2_lifecycle_manager` bringing up/monitoring the Nav2
  node set.

## Mission package changes

- Swap the stub `navigate_to_pose`/`dock_robot` action *names* (and only the
  names/targets — the tree structure itself doesn't change) to point at the
  real Nav2/docking servers instead of `mission`'s own stub action servers.
- `VehicleStatusOK`: replace the hardcoded-`Success` stub with a real check
  now that real nodes exist to check — e.g. Nav2's lifecycle manager
  reporting all nodes active, slam_toolbox publishing a recent pose with
  bounded covariance. Exact signal set is an implementation-time call;
  "soft" (slows down) vs. "hard" (trips safe-stop) classification from the
  original behavior-tree decode should still roughly hold.
- `Localize`: replace the hardcoded-`Success` stub similarly — check
  `map→odom` TF is being published and recent.
- Stub `stub_action_servers.py` and its tests can be deleted once nothing
  references them, or left in place disabled — implementer's call; don't
  delete working, still-referenced code.

## Deferred to real hardware (explicitly out of scope here, documented so the swap-in is easy later)

- `isaac_ros_visual_slam` as backup odometry (Jetson-only, explicitly a
  stretch goal even in the original plan).
- `isaac_ros_object_detection` for classifying obstacles (explicitly
  "add later only if needed" in the original plan).
- Real camera calibration (sim's `camera_info` is clean/synthetic; a real
  OAK-D Lite needs an actual calibration pass — not applicable here).
- Multi-robot occupied-dock detection (sim is single-robot; logic is wired
  but the "another robot is here" signal can never fire in this environment).
- Isaac-specific GPU-accelerated AprilTag/stereo depth (replaced by
  `apriltag_ros` and the sim's already-published `/oak/points`, both of
  which also work on the real robot, so no rework is needed later here —
  only the Isaac-specific *optional* GPU paths are the actual gap).

## Testing approach

No unit-test-first discipline for most of this — these are mostly
off-the-shelf ROS 2 packages being configured and wired together, not novel
logic. Verification is integration-shaped: launch it, check the right
topics/TF/actions appear and behave, then launch the *whole* stack
(localization + perception + navigation + mission) against the Gazebo sim
and a route from the local Saga AI mock server, and iterate — fix whatever
breaks — until a full dock→load→dock→unload cycle completes for real, with
the robot actually driving and docking (not a stub reporting instant
success). This mirrors the original plan's own stated development
approach ("first a thin version runs end-to-end, then each owner replaces
their stub with the real part").

## Out of scope for this pass

- Finals-day rehearsed quick-mapping procedure (this builds the map once,
  manually, for development — the quick/rehearsed version is a
  separate, later concern).
- Everything listed under "Deferred to real hardware" above.
- Tuning Nav2/MPPI parameters beyond "good enough to complete a route
  reliably in this arena" — not a tuned-for-competition-speed pass.

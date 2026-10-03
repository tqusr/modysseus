# IMU + Wheel Odometry Fusion Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Combine wheel odometry and the chassis IMU gyro into one estimate of how the robot moves, on both the real robot and in simulation. Each sensor covers the other's weak spot: the wheels measure distance well but turning badly, and the gyro measures turning well but drifts over time. The fused result should feed both Nav2 and slam_toolbox.

**Architecture:** Finish the `robot_localization` EKF that already exists (`src/localization/config/ekf.yaml`) instead of writing a new filter. Fuse velocities only: forward speed from the wheels, turn rate mainly from the gyro. The EKF integrates those into a pose and becomes the single owner of the `odom -> base_footprint` TF. A small new ROS-free module (with a thin node around it) removes gyro bias, using the encoders to detect when the robot is standing still.

**Tech Stack:** ROS 2 Jazzy, `robot_localization` (`ekf_node`), `rclpy`, `sensor_msgs/Imu`, `nav_msgs/Odometry`, `pytest`, `ros2 bag` for validation.

**Related code:** `src/localization/localization/wheel_odometry.py`, `src/localization/scripts/wheel_odometry_node.py`, `src/localization/config/ekf.yaml`, `src/localization/launch/localization.launch.py`, `src/localization/launch/mapping_mission_hardware.launch.py`; vendor: `einride-mini-truck/docs/sim-vs-hardware.md`, `einride_mini_truck_hardware/ugv02_serial_node.py`, `einride_mini_truck_description/models/einride_mini_truck/model.sdf`.

---

## Background: what each sensor is good and bad at

| | Wheel odometry (`/wheel_encoders` -> `/odom`) | IMU gyro (ICM-20948 -> `/imu`) |
|---|---|---|
| **Forward distance** | Good: exact encoder counts, ~1 cm resolution | Poor: you'd have to integrate the accelerometer twice, and the error grows quickly |
| **Rotation** | Poor: this skid-steer robot has to drag its wheels sideways to turn. The scrub factor is a guess from simulation, and only the front axle has encoders | Good: it measures turn rate directly and doesn't care about friction |
| **Long-term error** | Slip and scrub add up mostly when turning | A small gyro offset (bias) slowly rotates the heading, even when the robot is standing still |
| **Absolute heading** | No | No: on hardware the MCU sends raw rates only (`orientation_covariance[0] = -1`) |

**The split:** distance and forward speed come from the wheels, turning comes mainly from the gyro, and the wheels catch the gyro's drift. When the encoders say the robot is standing still, any non-zero gyro reading is pure offset, and that offset can be measured and subtracted.

**Syncing is mostly done already.** On hardware, the IMU and encoder values arrive in the same `T:1001` serial line and get the same back-dated timestamp (`ugv02_serial_node.py`, publish block for `imu` / `mag` / `wheel_encoders`). In simulation, both run on `/clock`. No extra time alignment is needed beyond correct `use_sim_time` handling.

## Problems in the current setup

1. **`ekf.yaml` fuses `/odom`'s absolute pose (x, y, yaw) *and* its velocities.** That gives the wheel data two votes, and the wheels' wrong idea of the heading fights the gyro.
2. **`ekf.yaml` fuses IMU orientation (roll, pitch, yaw).** In Gazebo that is close to a perfect true heading, so simulation looks better than it will on the robot. On hardware the EKF ignores the orientation because its covariance is -1. Sim tuning therefore won't carry over.
3. **It fuses accelerometer x/y/z with `imu0_remove_gravitational_acceleration: true`.** Removing gravity needs the robot's tilt, which hardware doesn't provide, so any tilt leaks gravity into the forward acceleration.
4. **`use_sim_time: true` is hardcoded in `ekf.yaml`.** On the real robot (no `/clock`) the EKF would sit at time zero forever.
5. **`publish_tf: false`.** slam_toolbox reads the `odom -> base_footprint` TF, not `/odometry/filtered`. On hardware, `wheel_odometry_node` owns that TF, so SLAM never sees the fused estimate.
6. **No gyro bias handling.** `robot_localization` doesn't estimate gyro bias.
7. **The wheel covariances are placeholders** (`wheel_odometry_node.py`, `position_stddev` / `yaw_stddev` / `linear_velocity_stddev` / `yaw_rate_stddev` = 0.05). They are constant, even though wheel turning error is much worse during turns.
8. **Wheel separation disagrees with itself.** The code constant `WHEEL_SEPARATION_M` is `0.4084` (`SCRUB_FACTOR = 2.340`), but the `mapping_mission_hardware.launch.py` default is `0.21701` (scrub factor 1.2435). The docstrings mention both numbers.

## Global Constraints

- **Never `git push`** without explicit instruction (standing rule for this repo).
- Follow the package's existing split: maths in ROS-free modules under `src/localization/localization/` with pytest tests, and thin ROS shells in `src/localization/scripts/`.
- Exactly **one** publisher per TF edge. Two unsynchronised publishers of `odom -> base_footprint` already caused a TF_OLD_DATA storm once (see `ekf.yaml`'s header comment).
- `/imu` and `/wheel_encoders` are best-effort. Every new subscriber must use `qos_profile_sensor_data`, or it silently receives nothing.
- Don't declare `use_sim_time` inside nodes; set it from the launch file (see `wheel_odometry_node.py`'s docstring).
- Changes to the vendor repo (`einride-mini-truck`) need explicit sign-off first. Task 4 is the only task that touches it.

---

## File Structure

```
modysseus/src/localization/
  config/
    ekf.yaml                      # Task 1: velocity-only fusion; header updated
    ekf_hardware.yaml             # Task 2: (or launch-arg override) use_sim_time false, publish_tf true
  localization/
    gyro_bias.py                  # Task 5: ROS-free bias estimator
  scripts/
    gyro_bias_node.py             # Task 5: /imu -> /imu/corrected
    wheel_odometry_node.py        # Task 3: publish_tf param used false under EKF; Task 6: covariances
  launch/
    localization.launch.py        # Task 2: use_sim_time / config-file arguments
    mapping_mission_hardware.launch.py  # Task 3: start bias node + EKF, hand TF to EKF
  test/
    test_gyro_bias.py             # Task 5
  CMakeLists.txt                  # Task 5: install gyro_bias_node.py, add test
```

---

## Task 1: Velocity-only fusion in `ekf.yaml`

**Why:** Fixes problems 1-3. Each sensor contributes only what it really measures.

- [ ] Set `odom0_config` to fuse only `vx`, `vy`, `vyaw` (no x, y, yaw). `vy` is always 0 for a skid-steer, which tells the filter there is no sideways motion.
- [ ] Set `imu0_config` to fuse only `vyaw`. Turn off roll/pitch/yaw orientation and ax/ay/az.
- [ ] Remove `imu0_remove_gravitational_acceleration` (it no longer applies).
- [ ] Rewrite the file's header comment to explain the velocity-only choice and the sim-orientation trap.
- [ ] **Verify (sim):** `ros2 launch mission full_stack.launch.py`, drive a loop, and check that `/odometry/filtered` tracks Gazebo's true pose without the jumps that orientation fusion caused.

## Task 2: Make the EKF runnable on hardware

**Why:** Fixes problem 4.

- [ ] Add a `use_sim_time` launch argument to `localization.launch.py` (default `true`, so sim callers keep working) and pass it as a bool parameter. See the string-vs-bool note in `mapping_mission_hardware.launch.py`.
- [ ] Add a `publish_tf` override path: either an `ekf_hardware.yaml` layered after `ekf.yaml`, or a launch argument.
- [ ] **Verify (hardware):** with `hardware.launch.py` and `wheel_odometry_node` running, `ros2 topic hz /odometry/filtered` shows ~30 Hz, and `print_diagnostics` reports no stale inputs.

## Task 3: EKF owns `odom -> base_footprint` on hardware

**Why:** Fixes problem 5 on the real robot, so SLAM gets the fused pose.

- [ ] In `mapping_mission_hardware.launch.py` (and any future hardware localize launch), start `wheel_odometry_node` with `publish_tf:=false` and the EKF with `publish_tf: true`.
- [ ] Start the EKF before SLAM, so the TF edge exists before slam_toolbox's first scan lookup (same reasoning as the current wheel-odometry ordering).
- [ ] Update that launch file's docstring ("WHAT THIS LAUNCH DOES NOT INCLUDE" currently says the EKF is excluded).
- [ ] **Verify:** `ros2 run tf2_ros tf2_monitor odom base_footprint` shows exactly one authority (`ekf_filter_node`), and slam_toolbox reaches ACTIVE.

## Task 4: EKF owns `odom -> base_footprint` in simulation (needs vendor sign-off)

**Why:** Fixes problem 5 in simulation. Today Gazebo's DiffDrive plugin broadcasts this edge, which is why `publish_tf` is false.

- [ ] Get sign-off for the vendor change.
- [ ] Stop the DiffDrive plugin from broadcasting the TF (in `model.sdf`, or by remapping its tf topic in the bridge config) while keeping its `/odom` topic.
- [ ] Set `publish_tf: true` in `ekf.yaml`.
- [ ] **Verify:** no TF_OLD_DATA warnings, and Nav2 + SLAM localization still work end to end in `full_stack.launch.py`.
- [ ] **If sign-off is refused:** sim keeps fusion for Nav2 only (via `/odometry/filtered`), and document that limitation in `ekf.yaml`.

## Task 5: Gyro bias estimator

**Why:** Fixes problem 6. Uncorrected bias turns directly into heading drift.

- [ ] `localization/gyro_bias.py` (ROS-free):
  - Startup calibration: average gyro z over the first N seconds (default 2.0) while the robot is stationary. That average is the bias.
  - Zero-velocity update: while the encoders report no motion (both sides' deltas below a threshold) for at least 0.5 s, update the bias slowly (exponential moving average).
  - Ignore samples during motion, and never update the bias from a single sample.
- [ ] `scripts/gyro_bias_node.py`: subscribe to `/imu` and `/wheel_encoders` (sensor-data QoS), and publish `/imu/corrected` with the bias subtracted from `angular_velocity.z`. Pass through everything else (including the `-1` orientation covariance) unchanged.
- [ ] Point `imu0` in `ekf.yaml` at `imu/corrected`.
- [ ] `test/test_gyro_bias.py`: synthetic constant bias + noise gets recovered; motion samples don't corrupt the estimate; a sign error in the bias makes a test fail.
- [ ] Register the script and test in `CMakeLists.txt`.
- [ ] **Verify (hardware):** with the robot standing still for 5 minutes, the fused yaw changes by < 1°.

## Task 6: Measured covariances

**Why:** Fixes problem 7. The EKF weighs sensors by their covariance, so placeholder values mean placeholder fusion.

- [ ] **Gyro:** log `/imu` stationary for 60 s; check that the stddev of `angular_velocity.z` (after bias removal) is close to the datasheet value of 2.62e-3 rad/s.
- [ ] **Wheel speed:** repeat the 2 m straight-drive calibration ~5 times and set `linear_velocity_stddev` from the spread.
- [ ] **Wheel turn rate:** set `yaw_rate_stddev` deliberately large (~5-10x the linear value) so the gyro dominates rotation. Optional: scale it with |turn rate| inside `wheel_odometry_node`.
- [ ] Update the "PLACEHOLDERS" comment in `wheel_odometry_node.py` with where the numbers came from.

## Task 7: Calibrate wheel separation with the gyro

**Why:** Fixes problem 8, and improves wheel odometry on its own too.

- [ ] Drive slow on-the-spot turns (e.g. 5 x 360°) and record `/imu/corrected` and `/wheel_encoders`.
- [ ] Integrate the gyro angle and the wheel angle; the ratio (wheel ÷ gyro) multiplied by the current `wheel_separation` gives the corrected effective separation.
- [ ] Repeat in both turn directions and on the arena floor.
- [ ] Fix the single source of truth: make `WHEEL_SEPARATION_M` / `SCRUB_FACTOR` and the launch default agree, and update every docstring that cites 1.2435 or 2.340.

## Task 8: Validation

**Why:** Show that the fused estimate is actually better than either sensor alone.

- [ ] **Closed-loop test:** drive a ~2 m square back to a taped start mark, recording `/odom`, `/imu/corrected` and `/odometry/filtered` with `ros2 bag`. Compare the end-point error for wheels only, gyro-integrated heading, and fused.
- [ ] **Spin test:** 10 x 360° turns. The fused heading should come back within a few degrees; record how far off wheels alone are.
- [ ] **Map quality:** map the arena with and without the EKF. Compare corner angles and whether the square room closes properly.
- [ ] **Simulation regression:** compare `/odometry/filtered` against Gazebo's true pose over a full mission.
- [ ] Write the results (numbers, not impressions) into this plan or a short report next to it.

---

## Open Decisions

- **TF ownership in simulation (Task 4):** this is the biggest change, because it touches the vendor's `model.sdf`. Without it, the fusion only reaches Nav2 in simulation, never SLAM.
- **Second IMU:** the camera's BMI270 (`/oak/imu/data`) is about 2x less noisy than the chassis gyro. It could be added later as `imu1`, but its frame transform and timing (a separate USB device, not the MCU line) would need checking first.
- **Magnetometer:** `/mag` (AK09916) could give an absolute heading, but motors and metal indoors usually make it unreliable. Out of scope unless Task 8 shows heading drift is still the main error.
- **Accelerometer:** left out in Task 1. Revisit only if Task 8 shows forward-speed errors from wheel slip (for example on hard acceleration), and only for `ax`.

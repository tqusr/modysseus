"""Pure skid-steer wheel-odometry maths behind `wheel_odometry_node.py`.

Imports no ROS, so the whole conversion (two accumulated per-side wheel
angles to a pose in `odom` plus a body twist) is unit-testable without a graph
or a robot (see `localization/test/test_wheel_odometry.py`).
`scripts/wheel_odometry_node.py` is a thin shell: subscribe, call in here,
publish. `perception/dock_capture.py` and `capture_dock_pose.py` split the
same way.

Nothing else published `odom -> base_footprint`, which slam_toolbox looks up
for every scan (`mapper_params_hardware.yaml`, `odom_frame: odom`). This
module provides `/odom`; the EKF (`ekf.yaml`) fuses it with the gyro and owns
the TF edge, so the node's `publish_tf` is off.

Input: `/wheel_encoders` is a `sensor_msgs/JointState` from the vendor driver
(`einride_mini_truck_hardware`'s `ugv02_serial_node.py`) with exactly two
entries, aggregated per side:

  - `name`: `['left_up_wheel_link_joint', 'right_up_wheel_link_joint']`, left
    first, the order the MCU sends odl/odr in (see `config/hardware.yaml`).
  - `position`: accumulated angle per side in radians, i.e. metres divided by
    the wheel radius.
  - `velocity`: the same two sides in rad/s.

Position deltas are integrated, not the published velocity. A position
difference is the exact distance travelled whatever the sample rate, dropped
messages or clock. The driver differentiates `velocity` on its own monotonic
clock while the header stamp is a back-dated ROS time, so combining them with
a dt mixes two clocks. It also reports (0.0, 0.0) for its first sample and
computes the value before any `feedback_publish_rate` decimation. So dt is
used only to turn an exact displacement into a reported velocity.

Kinematic constants: `WHEEL_RADIUS_M` is from the vendor's `model.sdf`.
`WHEEL_SEPARATION_M` was measured on the real robot and is the code default
(and what the tests use). On the robot the authoritative value is
`wheel_separation` in `config/hardware.yaml`, loaded by
`mapping_mission_hardware.launch.py`. Keep the two in step.

`WHEEL_SEPARATION_M` is not the geometric track width (0.17452 m). A
skid-steer scrubs its wheels sideways to turn, so the ideal differential model
over-predicts yaw rate. Do not change it back to the geometric value.

Measurement (real robot, arena floor): spin N full turns in place and read
the position deltas:

    effective_separation = |dR - dL| / (2 * pi * N)      [dR, dL in metres]

Position is radians = MCU centimetres / wheel_radius, i.e. 25 units per cm
before `encoder_scale` converts to metres. Five clockwise turns gave dL =
+16025 and dR = -16050 units, so |dR - dL| = 12.830 m over 31.416 rad, an
effective separation of 0.4084 m (scrub factor 2.340). This is a single
clockwise run; a counter-clockwise run should be averaged in.

Only two of the four driven wheels are sensed: the chassis is 6-wheel 4WD
(front and rear driven, middle idlers) and only the front axle has encoders.
This odometry cannot see a rear wheel slipping against the front one.

Sign conventions follow REP-103 (x forward, y left, yaw positive
counter-clockwise), so the right side running faster turns the robot left,
hence `arc_step`'s `(d_right - d_left)`. Not verified offline: that the MCU's
odl/odr increase when driving forward (the firmware is not in the workspace).
If wrong, the symptom is unmistakable. The left-then-right order is
documented.

Encoder units: the vendor driver documents `position` as metres, but the MCU
counts whole centimetres. Driving a measured ~2.00 m gave a 201-count average
delta, ~0.00995 m per count (0.5% off 1 cm). That rules out the ~1/29 "tick
count" scale assumed earlier, which predicted ~58 counts.
`ENCODER_SCALE_FACTOR` and the `encoder_scale` parameter apply the correction.
"""

import math
from dataclasses import dataclass


#: Wheel radius in metres, from model.sdf and the vendor driver's
#: `wheel_radius` parameter, which must agree or `position` is not radians of
#: the same wheel.
WHEEL_RADIUS_M = 0.040

#: Geometric track width in metres. Only the reference for the scrub factor;
#: never used for kinematics.
GEOMETRIC_TRACK_M = 0.17452

#: Measured ratio of effective to geometric track (single clockwise run; see
#: the module docstring).
SCRUB_FACTOR = 2.340

#: Effective wheel separation in metres (`GEOMETRIC_TRACK_M * SCRUB_FACTOR`,
#: rounded to the measured value). Code default; `config/hardware.yaml` is
#: authoritative on the robot.
WHEEL_SEPARATION_M = 0.4084

#: Centimetres to metres: the vendor's `position` is mislabelled as metres
#: (see the module docstring). Code default; `config/hardware.yaml` is
#: authoritative on the robot.
ENCODER_SCALE_FACTOR = 1.0 / 100.0

#: Longest gap (s) between samples still integrated as one constant-curvature
#: arc. The link runs near 80 Hz, so this is ~40 missed samples. A longer gap
#: is re-anchored and skipped (see `WheelOdometry.update`).
MAX_SAMPLE_GAP_SEC = 0.5

#: Per-side speed (m/s) above which a sample is garbage, not motion. Top speed
#: is ~1.39 m/s and quantization noise can imply up to ~1.65 m/s per sample, so
#: 2.0 leaves headroom while still catching an accumulator reset or jump.
MAX_WHEEL_SPEED_MPS = 2.0

#: Yaw change (rad) below which an arc is integrated as a straight line.
#: Not a tuning knob: it only avoids dividing by a vanishing `dtheta`.
STRAIGHT_LINE_EPSILON_RAD = 1e-9

#: `update()` outcomes, as strings so logs and test failures name the case.
UPDATE_OK = 'ok'
UPDATE_FIRST_SAMPLE = 'first_sample'
UPDATE_BACKWARD_TIME = 'backward_time'
UPDATE_GAP = 'gap'
UPDATE_IMPLAUSIBLE = 'implausible'


class InvalidGeometry(ValueError):
    """A wheel radius or separation that cannot describe a real robot.

    Raised at construction rather than per sample, so a mistyped parameter is
    easy to trace.
    """


@dataclass(frozen=True)
class Pose2D:
    """A planar pose in the `odom` frame.

    Attributes:
        x: Metres.
        y: Metres.
        theta: Heading in radians.
    """

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass(frozen=True)
class Update:
    """The result of one encoder sample.

    Attributes:
        status: One of the `UPDATE_*` constants. Anything but `UPDATE_OK`
            means `pose` was not advanced.
        pose: The current pose, advanced or not. Always safe to publish.
        linear_velocity: Body-forward velocity in m/s; 0.0 unless `UPDATE_OK`.
        angular_velocity: Yaw rate in rad/s; 0.0 unless `UPDATE_OK`.
    """

    status: str
    pose: Pose2D
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


def normalize_angle(angle):
    """Wraps an angle into (-pi, pi]."""
    wrapped = math.fmod(angle, 2.0 * math.pi)
    if wrapped > math.pi:
        wrapped -= 2.0 * math.pi
    elif wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def yaw_to_quaternion(theta):
    """Returns the `(x, y, z, w)` quaternion for a rotation of `theta` about +z."""
    half = 0.5 * theta
    return (0.0, 0.0, math.sin(half), math.cos(half))


def side_displacements(previous_angles, current_angles, wheel_radius=WHEEL_RADIUS_M):
    """Returns the metres travelled by each side between two readings.

    Args:
        previous_angles: `(left, right)` accumulated angles in radians.
        current_angles: The same, from the newer sample.
        wheel_radius: Wheel radius in metres.

    Returns:
        `(d_left, d_right)` in metres, signed.
    """
    previous_left, previous_right = previous_angles
    current_left, current_right = current_angles
    return ((current_left - previous_left) * wheel_radius,
            (current_right - previous_right) * wheel_radius)


def arc_step(d_left, d_right, wheel_separation=WHEEL_SEPARATION_M):
    """Returns the body arc length and heading change for one step.

    Uses `(d_right - d_left)` because REP-103 yaw is counter-clockwise: the
    right side travelling further is a left turn.

    Args:
        d_left: Left displacement in metres.
        d_right: Right displacement in metres.
        wheel_separation: Effective separation in metres.

    Returns:
        `(ds, dtheta)`: metres along the arc, radians of yaw.
    """
    return (0.5 * (d_left + d_right), (d_right - d_left) / wheel_separation)


def advance(pose, ds, dtheta):
    """Moves `pose` along a constant-curvature arc.

    Uses the closed form, not an Euler step, which errs on every turn; the
    mapping drive is mostly near-in-place turns. Below
    `STRAIGHT_LINE_EPSILON_RAD` a straight line is used, which is the same
    curve without the singularity.

    Args:
        pose: Starting pose; not modified.
        ds: Arc length in metres.
        dtheta: Heading change in radians.

    Returns:
        A new `Pose2D`.
    """
    theta = pose.theta
    if abs(dtheta) < STRAIGHT_LINE_EPSILON_RAD:
        return Pose2D(pose.x + ds * math.cos(theta),
                      pose.y + ds * math.sin(theta),
                      normalize_angle(theta + dtheta))
    radius = ds / dtheta
    heading = theta + dtheta
    return Pose2D(pose.x + radius * (math.sin(heading) - math.sin(theta)),
                  pose.y + radius * (math.cos(theta) - math.cos(heading)),
                  normalize_angle(heading))


class WheelOdometry:
    """Integrates `/wheel_encoders` samples into a pose in the `odom` frame.

    Stateful but ROS-free: the node passes a stamp and two accumulated angles
    and gets an `Update`. The pose starts at the origin, which defines `odom`.

    Args:
        wheel_radius: Wheel radius in metres.
        wheel_separation: Effective, scrub-corrected separation in metres.
            See the module docstring before changing it.
        max_sample_gap_sec: See `MAX_SAMPLE_GAP_SEC`.
        max_wheel_speed: See `MAX_WHEEL_SPEED_MPS`.
        encoder_scale: Multiplier on each side's displacement, to correct the
            centimetre encoder units (see `ENCODER_SCALE_FACTOR`). Defaults to
            1.0 (off); `wheel_odometry_node.py` defaults it to
            `ENCODER_SCALE_FACTOR`.

    Raises:
        InvalidGeometry: If the radius or separation is not strictly positive.
    """

    def __init__(self,
                 wheel_radius=WHEEL_RADIUS_M,
                 wheel_separation=WHEEL_SEPARATION_M,
                 max_sample_gap_sec=MAX_SAMPLE_GAP_SEC,
                 max_wheel_speed=MAX_WHEEL_SPEED_MPS,
                 encoder_scale=1.0):
        if wheel_radius <= 0.0:
            raise InvalidGeometry(
                'wheel_radius must be > 0, got {}'.format(wheel_radius))
        if wheel_separation <= 0.0:
            raise InvalidGeometry(
                'wheel_separation must be > 0, got {}'.format(wheel_separation))
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.max_sample_gap_sec = max_sample_gap_sec
        self.max_wheel_speed = max_wheel_speed
        self.encoder_scale = encoder_scale
        self._pose = Pose2D()
        # (stamp_sec, left_angle, right_angle) of the sample the next one is
        # measured against; None until the first sample.
        self._anchor = None

    @property
    def pose(self):
        """Returns the current pose; only an `UPDATE_OK` sample advances it."""
        return self._pose

    @property
    def has_anchor(self):
        """Returns True once a first sample has been seen."""
        return self._anchor is not None

    def update(self, stamp_sec, left_angle, right_angle):
        """Adds one encoder sample.

        Args:
            stamp_sec: The sample's header stamp in seconds (never the wall
                clock, since the published pose and TF carry this stamp).
            left_angle: Accumulated left-side angle, radians.
            right_angle: Accumulated right-side angle, radians.

        Returns:
            An `Update`. Four cases hold the pose instead of publishing a jump:

              - `UPDATE_FIRST_SAMPLE`: nothing to difference against. The
                origin is still published, putting `odom -> base_footprint`
                into TF before slam_toolbox's first scan lookup.
              - `UPDATE_BACKWARD_TIME`: stamp at or before the anchor's. The
                sample is dropped and the anchor kept, so no distance is lost.
              - `UPDATE_GAP`: more than `max_sample_gap_sec` since the anchor,
                too long for a single arc. Re-anchored and skipped; expect
                this once after a serial reconnect.
              - `UPDATE_IMPLAUSIBLE`: a side moved faster than
                `max_wheel_speed`, so the accumulator jumped. Re-anchored
                and skipped.
        """
        sample = (stamp_sec, left_angle, right_angle)
        anchor = self._anchor
        if anchor is None:
            self._anchor = sample
            return Update(UPDATE_FIRST_SAMPLE, self._pose)

        anchor_stamp, anchor_left, anchor_right = anchor
        dt = stamp_sec - anchor_stamp
        if dt <= 0.0:
            # Anchor kept on purpose (see the docstring).
            return Update(UPDATE_BACKWARD_TIME, self._pose)

        self._anchor = sample
        if dt > self.max_sample_gap_sec:
            return Update(UPDATE_GAP, self._pose)

        d_left, d_right = side_displacements(
            (anchor_left, anchor_right), (left_angle, right_angle),
            self.wheel_radius)
        # Centimetre correction, applied before the plausibility guard so both
        # it and the kinematics see metres.
        d_left *= self.encoder_scale
        d_right *= self.encoder_scale
        if max(abs(d_left), abs(d_right)) > self.max_wheel_speed * dt:
            return Update(UPDATE_IMPLAUSIBLE, self._pose)

        ds, dtheta = arc_step(d_left, d_right, self.wheel_separation)
        self._pose = advance(self._pose, ds, dtheta)
        return Update(UPDATE_OK, self._pose, ds / dt, dtheta / dt)


__all__ = [
    'ENCODER_SCALE_FACTOR', 'GEOMETRIC_TRACK_M', 'InvalidGeometry',
    'MAX_SAMPLE_GAP_SEC', 'MAX_WHEEL_SPEED_MPS', 'Pose2D', 'SCRUB_FACTOR',
    'STRAIGHT_LINE_EPSILON_RAD', 'UPDATE_BACKWARD_TIME', 'UPDATE_FIRST_SAMPLE',
    'UPDATE_GAP', 'UPDATE_IMPLAUSIBLE', 'UPDATE_OK', 'Update',
    'WHEEL_RADIUS_M', 'WHEEL_SEPARATION_M', 'WheelOdometry', 'advance',
    'arc_step', 'normalize_angle', 'side_displacements', 'yaw_to_quaternion',
]

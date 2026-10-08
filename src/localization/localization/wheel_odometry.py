"""Pure skid-steer wheel-odometry maths behind `wheel_odometry_node.py`.

NOTHING HERE IMPORTS rclpy, tf2_ros OR ANY ROS MESSAGE, on purpose. The whole
conversion -- two accumulated per-side wheel angles -> a pose in `odom` and a
body twist -- is plain Python, so it is unit-testable without a ROS graph, a
robot (see `localization/test/test_wheel_odometry.py`).
`scripts/wheel_odometry_node.py` is then a thin ROS shell: subscribe, call in
here, publish. Same split, and for the same reason, as
`perception/perception/dock_capture.py` versus `capture_dock_pose.py`.

WHY THIS EXISTS. The robot had NO odometry source at all: nothing published a
dynamic `odom -> base_footprint` transform, which `mapper_params_hardware.yaml`
(`odom_frame: odom`) makes slam_toolbox look up for every scan. This module
provides `/odom`; the EKF (`ekf.yaml`) fuses it with the gyro and owns the TF
edge, so the node's own `publish_tf` is off.

THE INPUT, and what it actually guarantees.

`/wheel_encoders` is a `sensor_msgs/JointState` published by the vendor
driver, `einride_mini_truck_hardware`'s `ugv02_serial_node.py`. Read against
that source and `serial_link.py`, it carries EXACTLY TWO entries, aggregated
per side, not per wheel:

  - `name`     `['left_up_wheel_link_joint', 'right_up_wheel_link_joint']`,
               left first, then right - `config/hardware.yaml` says outright
               that this is "the order the MCU sends odl/odr in".
  - `position` accumulated angle per SIDE in radians, converted from the
               wire's accumulated metres by `distance_to_angle(d, radius)`,
               i.e. simply `metres / wheel_radius`.
  - `velocity` the same two sides in rad/s.

The module is written against joint NAMES and not against array positions
alone, so a reordered message cannot swap the robot's sides.

POSITION DELTAS, NOT THE PUBLISHED VELOCITY FIELD. `update()` integrates
differences of `position`, and derives the twist from those same differences.
That is a deliberate choice over reading `velocity`:

  - `position` is an accumulated quantity, so the difference between two
    samples is the exact distance travelled in between, whatever happened to
    the sample rate, however many messages were dropped, and regardless of
    which clock anybody used.
  - `velocity` is differentiated by the driver on its own MONOTONIC clock
    (see `_wheel_velocities`), while the header stamp is a back-dated ROS
    time. Multiplying one by a dt computed from the other mixes two clocks.
    The driver also reports `(0.0, 0.0)` for its own first sample and
    computes the value before any `feedback_publish_rate` decimation, so a
    decimated stream would report an instantaneous rate over an interval
    shorter than the one between published messages.

So dt is used only to turn an exact displacement into a reported velocity, and
never to produce the displacement itself.

THE KINEMATIC CONSTANTS. `WHEEL_RADIUS_M` comes from the vendor's chassis
description, `einride_mini_truck_description`'s `model.sdf`. `WHEEL_SEPARATION_M` is MEASURED ON THE REAL ROBOT and is the
the node's code default (and what the tests use); on the robot the
authoritative value is `wheel_separation` in `config/hardware.yaml`, which
`mapping_mission_hardware.launch.py` loads over it. Keep the two in step.

`WHEEL_SEPARATION_M` is NOT the geometric track width (0.17452 m). A
skid-steer has to scrub its wheels sideways to rotate, so the ideal two-wheel
differential model over-predicts yaw rate; the effective separation is the
geometric track times a scrub factor. Do not "fix" it back to the geometric
value.

HOW IT WAS MEASURED (real robot, arena floor). Spin N full turns in place and
read the `/wheel_encoders` position deltas over the spin:

    effective_separation = |dR - dL| / (2 * pi * N)      [dR, dL in metres]

where, remember, `/wheel_encoders` position is RADIANS = MCU centimetres /
wheel_radius, i.e. 25 position units per cm (0.4 mm per unit) before the
`encoder_scale` correction below turns it into metres. Measured: 5 clockwise
turns gave dL = +16025 and dR = -16050 units, so |dR - dL| = 32075 units =
12.830 m over 5 * 2 * pi = 31.416 rad -> effective separation 0.4084 m, scrub
factor 0.4084 / 0.17452 = 2.340. THIS IS A SINGLE CLOCKWISE RUN; a
counter-clockwise run is still to come and should be averaged in.

(An earlier value, 1.2435 / 0.21701 m, was far too small for the real tyres
and floor, which under-rotates the map.)

ONLY TWO OF THE FOUR DRIVEN WHEELS ARE SENSED. The chassis is 6-wheel, 4WD
(front and rear pairs driven, middle pair free-rolling idlers), but only the
front axle is instrumented - see `hardware.launch.py`'s `joint_state_relay`
comment. This is therefore real odometry, not perfect odometry: it cannot see
a rear wheel slipping differently from the front one on the same side. It is
still vastly better than no motion prior at all, which is what slam_toolbox
had on hardware before this existed.

SIGN CONVENTIONS, and which of them are verified.

REP-103: `x` forward, `y` left, `z` up, yaw positive counter-clockwise seen
from above. So the right side running faster than the left turns the robot
left, which is positive yaw - hence `arc_step`'s `(d_right - d_left)`.

What is NOT verified offline: that the MCU's `odl`/`odr` actually INCREASE
when the robot drives forward. `serial_link.py` documents them only as
"accumulated distance travelled by the left/right side, metres" and the
firmware is not in this workspace, so forward-positive is an assumption
inherited from the driver's intent (positive joint angle is forward
rotation). If it is wrong, the symptom
is unmistakable and is listed in this feature's report. The index order (left
then right) IS documented, in both the node and `hardware.yaml`.

ENCODER UNIT CALIBRATION: THE REAL ROBOT'S `position` FIELD IS CENTIMETRES, NOT
METRES. The vendor driver (`serial_link.py`) documents `/wheel_encoders`
`position` as metres, but the underlying MCU counts are actually whole
CENTIMETRES - the vendor's label is simply wrong. Confirmed by a physical
calibration test: driving the robot a measured ~2.00 m produced a 201-count
average delta across the left/right wheels, i.e. ~0.00995 m per count (~1 cm,
a 0.5% match against the centimetre hypothesis). This also cleanly rejects the
~1/29 "tick count" scale assumed before this calibration: it predicted only
~58 counts for the same run, off by 3.47x.

`ENCODER_SCALE_FACTOR` below (and the `encoder_scale` parameter it feeds)
applies this confirmed centimetre-to-metre correction - see its own comment.
"""

import math
from dataclasses import dataclass


#: Wheel radius in metres, from model.sdf ("measured from the wheel mesh,
#: circumradius 0.040") and from the vendor driver's own
#: `wheel_radius` parameter, which must agree with it or `position` would not
#: be radians of the same wheel.
WHEEL_RADIUS_M = 0.040

#: Geometric track width in metres, 2 x 0.08726. Recorded only so the scrub
#: factor below has something to be a factor OF; never used for kinematics.
GEOMETRIC_TRACK_M = 0.17452

#: Measured ratio of effective to geometric track for this chassis, on the REAL
#: robot (5 clockwise in-place turns; see the module docstring). Single CW run
#: so far - average in a CCW run when it is available.
SCRUB_FACTOR = 2.340

#: Effective wheel separation in metres: `GEOMETRIC_TRACK_M * SCRUB_FACTOR`,
#: rounded to the measured 0.4084. Code default only: on the robot the
#: authoritative value comes from `config/hardware.yaml`.
WHEEL_SEPARATION_M = 0.4084

#: Code default; on the robot `config/hardware.yaml` is authoritative.
#: Confirmed unit-conversion factor for the real robot's encoder data: the
#: vendor's `position` field is mislabelled as metres but is actually whole
#: centimetres - see the module docstring's "ENCODER UNIT CALIBRATION" note
#: for the calibration test (~2.00 m drive, 201 counts, ~1 cm/count) that
#: confirmed this and superseded the earlier ~1/29 assumption.
ENCODER_SCALE_FACTOR = 1.0 / 100.0

#: Longest gap between two encoder samples that is still integrated as a
#: single constant-curvature arc, in seconds. The link runs near 80 Hz
#: (~12.5 ms), so this is ~40 missed samples: long enough never to trip on
#: ordinary scheduling jitter, short enough that the arc assumption still
#: holds. A longer gap is re-anchored and skipped - see `WheelOdometry.update`.
MAX_SAMPLE_GAP_SEC = 0.5

#: Per-side speed above which a sample is treated as garbage rather than
#: motion, in m/s. The chassis's real top speed is ~1.39 m/s (~5 km/h); with
#: the confirmed centimetre encoder scale (see `ENCODER_SCALE_FACTOR`), a
#: single sample at that speed can quantize up to roughly 1.4-1.65 m/s of
#: implied speed from timing/quantization noise alone, so the guard needs
#: headroom above the true ceiling rather than sitting right on it. 2.0 is a
#: deliberately round number with generous margin above that - not the
#: tighter ~1.6-1.8 that would be the minimal-headroom alternative - while
#: still catching something genuinely impossible, such as the MCU's
#: accumulator resetting or jumping across a multi-sample gap, which would be
#: many multiples of the true ceiling.
MAX_WHEEL_SPEED_MPS = 2.0

#: Below this much yaw change in one step, the arc is integrated as a straight
#: line. Not a tuning knob: it only avoids dividing by a vanishing `dtheta`
#: in `advance`, and at 1e-9 rad the two formulations agree far below any
#: physically meaningful resolution.
STRAIGHT_LINE_EPSILON_RAD = 1e-9

#: `update()` outcomes. Strings, so a log line or a failing test names the
#: case instead of an opaque integer.
UPDATE_OK = 'ok'
UPDATE_FIRST_SAMPLE = 'first_sample'
UPDATE_BACKWARD_TIME = 'backward_time'
UPDATE_GAP = 'gap'
UPDATE_IMPLAUSIBLE = 'implausible'


class InvalidGeometry(ValueError):
    """A wheel radius or separation that cannot describe a real robot.

    Raised at construction, not per sample: a zero or negative value would
    otherwise divide by zero (or silently mirror the robot's steering) on
    every message, which is far harder to trace back to a mistyped parameter.
    """


@dataclass(frozen=True)
class Pose2D:
    """A planar pose in the `odom` frame: metres, metres, radians."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass(frozen=True)
class Update:
    """What one encoder sample did.

    Attributes:
        status: one of the ``UPDATE_*`` constants. Anything other than
            `UPDATE_OK` means `pose` was NOT advanced by this sample.
        pose: the current pose, advanced or not. Always safe to publish:
            holding the previous pose still is the honest answer to a sample
            that could not be integrated.
        linear_velocity: body-forward velocity in m/s, 0.0 unless `UPDATE_OK`.
        angular_velocity: yaw rate in rad/s, 0.0 unless `UPDATE_OK`.
    """

    status: str
    pose: Pose2D
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


def normalize_angle(angle):
    """Wrap an angle into (-pi, pi].

    Keeps the integrated heading from growing without bound over a long run,
    which matters because the yaw-only quaternion below is built from it.
    """
    wrapped = math.fmod(angle, 2.0 * math.pi)
    if wrapped > math.pi:
        wrapped -= 2.0 * math.pi
    elif wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def yaw_to_quaternion(theta):
    """`(x, y, z, w)` for a rotation of `theta` about +z.

    Written out rather than taken from tf_transformations: this module imports
    no ROS, and a planar rotation is two trigonometric calls.
    """
    half = 0.5 * theta
    return (0.0, 0.0, math.sin(half), math.cos(half))


def side_displacements(previous_angles, current_angles, wheel_radius=WHEEL_RADIUS_M):
    """Metres travelled by each side between two `position` readings.

    Args:
        previous_angles: `(left, right)` accumulated angles in radians.
        current_angles: the same, from the newer sample.
        wheel_radius: metres.

    Returns:
        `(d_left, d_right)` in metres, signed.
    """
    previous_left, previous_right = previous_angles
    current_left, current_right = current_angles
    return ((current_left - previous_left) * wheel_radius,
            (current_right - previous_right) * wheel_radius)


def arc_step(d_left, d_right, wheel_separation=WHEEL_SEPARATION_M):
    """Body arc length and heading change for one pair of side displacements.

    The standard differential-drive relation. `(d_right - d_left)` and not the
    other way round because REP-103 yaw is positive counter-clockwise, so the
    right side travelling further is a left turn - see the module docstring.

    Returns:
        `(ds, dtheta)`: metres along the arc, radians of yaw.
    """
    return (0.5 * (d_left + d_right), (d_right - d_left) / wheel_separation)


def advance(pose, ds, dtheta):
    """Move `pose` along a constant-curvature arc of length `ds` turning `dtheta`.

    The closed form, not an Euler step. Euler integration (move `ds` along the
    OLD heading, then rotate) leaves a systematic error on every turn, and
    this robot's mapping runs are mostly turns: the arena is small and
    `mapper_params_hardware.yaml` had to tighten `minimum_travel_heading`
    specifically because the drive does several near-in-place rotations of 90
    degrees or more. Getting those right costs four trigonometric calls.

    As `dtheta` vanishes the turn radius `ds / dtheta` diverges, so a straight
    line is substituted below `STRAIGHT_LINE_EPSILON_RAD` - which is the same
    curve, just without the removable singularity.

    Returns:
        A new `Pose2D`. `pose` itself is frozen and untouched.
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

    Stateful but ROS-free: the node hands it a stamp and two accumulated
    angles and gets back an `Update`. The pose starts at the origin, which is
    the definition of the `odom` frame rather than a guess about where the
    robot is.

    Args:
        wheel_radius: metres.
        wheel_separation: EFFECTIVE separation in metres, scrub-corrected.
            See the module docstring before changing it.
        max_sample_gap_sec: see `MAX_SAMPLE_GAP_SEC`.
        max_wheel_speed: see `MAX_WHEEL_SPEED_MPS`.
        encoder_scale: multiplier applied to each side's displacement before
            any kinematics, to correct the real encoder data's confirmed
            centimetre units - see the module docstring's "ENCODER UNIT
            CALIBRATION" note and `ENCODER_SCALE_FACTOR`. Defaults to `1.0`,
            i.e. OFF: this class never applies a hidden correction unless a
            caller explicitly asks for one. `wheel_odometry_node.py` is the
            caller that defaults it to `ENCODER_SCALE_FACTOR` instead.

    Raises:
        InvalidGeometry: either geometric value is not strictly positive.
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
        #: `(stamp_sec, left_angle, right_angle)` of the sample the next one is
        #: measured against, or None until the first sample arrives.
        self._anchor = None

    @property
    def pose(self):
        """The current pose. Advanced only by an `UPDATE_OK` sample."""
        return self._pose

    @property
    def has_anchor(self):
        """True once a first sample has been seen."""
        return self._anchor is not None

    def update(self, stamp_sec, left_angle, right_angle):
        """Fold one encoder sample in.

        Args:
            stamp_sec: the sample's own header stamp in seconds. The header
                stamp, never the wall clock: the published pose and TF are
                stamped with it, so integrating against anything else would
                date the result differently from the data it describes.
            left_angle: accumulated left-side angle, radians.
            right_angle: accumulated right-side angle, radians.

        Returns:
            An `Update`. Four cases do NOT advance the pose, and each is a
            deliberate choice to hold still rather than publish a jump:

              - `UPDATE_FIRST_SAMPLE`: nothing to difference against yet. The
                pose is the origin, and publishing it is the point - it is
                what puts the `odom -> base_footprint` edge into TF before
                slam_toolbox's first scan lookup.
              - `UPDATE_BACKWARD_TIME`: a stamp at or before the anchor's.
                The sample is discarded and THE ANCHOR IS KEPT, so the next
                in-order sample still differences against a real earlier
                reading and loses no distance.
              - `UPDATE_GAP`: more than `max_sample_gap_sec` since the anchor.
                The displacement is still exact, but a single arc is no longer
                a fair description of what happened across that long a gap -
                the robot could have turned around and come back. Re-anchored
                and skipped, so the pose under-reports instead of inventing a
                path. Expect this once after a serial reconnect.
              - `UPDATE_IMPLAUSIBLE`: a side moved faster than
                `max_wheel_speed`. Physically impossible for this chassis, so
                it is the accumulator having jumped rather than the robot
                having moved. Re-anchored and skipped.
        """
        sample = (stamp_sec, left_angle, right_angle)
        anchor = self._anchor
        if anchor is None:
            self._anchor = sample
            return Update(UPDATE_FIRST_SAMPLE, self._pose)

        anchor_stamp, anchor_left, anchor_right = anchor
        dt = stamp_sec - anchor_stamp
        if dt <= 0.0:
            # Anchor deliberately not replaced - see the docstring.
            return Update(UPDATE_BACKWARD_TIME, self._pose)

        self._anchor = sample
        if dt > self.max_sample_gap_sec:
            return Update(UPDATE_GAP, self._pose)

        d_left, d_right = side_displacements(
            (anchor_left, anchor_right), (left_angle, right_angle),
            self.wheel_radius)
        # Confirmed centimetre-unit correction - see ENCODER_SCALE_FACTOR and
        # the module docstring's "ENCODER UNIT CALIBRATION" note. A no-op at
        # the class default of 1.0, applied here (before the plausibility
        # guard and the kinematics) so both see the corrected,
        # physically-meaningful displacement.
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

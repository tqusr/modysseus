"""Unit tests for localization.wheel_odometry -- the pure maths behind
`ros2 run localization wheel_odometry_node.py`.

No rclpy, no tf2, no ROS graph, no robot, no Gazebo: every test here is plain
Python against plain functions, which is the reason that module exists
separately from the node at all.

WHAT THESE TESTS CANNOT DO, and it matters. They pin down the kinematics and
every guard, but they cannot confirm the two things that only the real robot
can answer:

  - that the MCU's `odl`/`odr` increase when the robot drives FORWARD (the
    sign assumption; see the module docstring), and
  - that `wheel_separation = 0.21701` is right for real tyres on a real
    floor, rather than only for the simulation it was measured in.

Both are listed, with the symptoms to watch for, in this feature's report.
What the tests do instead is make sure that IF those two inputs are right, the
pose that comes out is right - so a bad map can be blamed on calibration
rather than on arithmetic.
"""

import math

import pytest

from localization.wheel_odometry import (GEOMETRIC_TRACK_M, InvalidGeometry,
                                         MAX_SAMPLE_GAP_SEC,
                                         MAX_WHEEL_SPEED_MPS, Pose2D,
                                         SCRUB_FACTOR, UPDATE_BACKWARD_TIME,
                                         UPDATE_FIRST_SAMPLE, UPDATE_GAP,
                                         UPDATE_IMPLAUSIBLE, UPDATE_OK,
                                         WHEEL_RADIUS_M, WHEEL_SEPARATION_M,
                                         WheelOdometry, advance, arc_step,
                                         normalize_angle, side_displacements,
                                         yaw_to_quaternion)


def test_localization_package_importable():
    import localization  # noqa: F401


# --------------------------------------------------------------------------
# The constants, and that they still describe the same robot
# --------------------------------------------------------------------------

def test_effective_separation_is_the_geometric_track_times_the_scrub_factor():
    """Guards the number most likely to be 'corrected' back to the track width.

    model.sdf derives 0.21701 as 0.17452 x 1.2435. If somebody edits one of
    the three without the others, this fails rather than quietly changing
    every turn the robot records.
    """
    assert GEOMETRIC_TRACK_M * SCRUB_FACTOR == pytest.approx(
        WHEEL_SEPARATION_M, abs=1e-5)


def test_effective_separation_is_wider_than_the_geometric_track():
    """Scrub always makes the EFFECTIVE track wider, never narrower."""
    assert WHEEL_SEPARATION_M > GEOMETRIC_TRACK_M


# --------------------------------------------------------------------------
# normalize_angle
# --------------------------------------------------------------------------

@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0),
    (1.0, 1.0),
    (math.pi, math.pi),
    (-math.pi, math.pi),
    (math.pi + 0.5, -math.pi + 0.5),
    (3.0 * math.pi, math.pi),
    (-3.0 * math.pi, math.pi),
    (100.0 * math.pi + 0.25, 0.25),
])
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected, abs=1e-9)


def test_normalize_angle_keeps_a_long_run_bounded():
    """A full mapping run turns many times; the heading must not grow forever."""
    theta = 0.0
    for _ in range(1000):
        theta = normalize_angle(theta + 0.5)
    assert -math.pi < theta <= math.pi


# --------------------------------------------------------------------------
# yaw_to_quaternion
# --------------------------------------------------------------------------

@pytest.mark.parametrize('theta', [0.0, 0.5, -0.5, math.pi / 2.0, math.pi - 1e-6])
def test_yaw_to_quaternion_is_a_unit_quaternion_about_z(theta):
    x, y, z, w = yaw_to_quaternion(theta)
    assert (x, y) == (0.0, 0.0)
    assert x * x + y * y + z * z + w * w == pytest.approx(1.0)
    # Recovering the yaw the way every consumer does.
    assert 2.0 * math.atan2(z, w) == pytest.approx(theta)


# --------------------------------------------------------------------------
# side_displacements
# --------------------------------------------------------------------------

def test_side_displacements_converts_radians_to_metres():
    d_left, d_right = side_displacements((0.0, 0.0), (1.0, 2.0), wheel_radius=0.04)
    assert d_left == pytest.approx(0.04)
    assert d_right == pytest.approx(0.08)


def test_side_displacements_is_a_difference_not_an_absolute_reading():
    """Accumulated angles are integrated as deltas, so an offset cancels."""
    assert side_displacements((10.0, 10.0), (11.0, 12.0), 0.04) == pytest.approx(
        side_displacements((0.0, 0.0), (1.0, 2.0), 0.04))


def test_side_displacements_signs_reverse_motion():
    d_left, d_right = side_displacements((1.0, 1.0), (0.0, 0.0), 0.04)
    assert d_left < 0.0 and d_right < 0.0


# --------------------------------------------------------------------------
# arc_step: the sign convention
# --------------------------------------------------------------------------

def test_arc_step_equal_displacements_is_pure_translation():
    ds, dtheta = arc_step(0.1, 0.1)
    assert ds == pytest.approx(0.1)
    assert dtheta == pytest.approx(0.0)


def test_arc_step_right_side_faster_is_positive_yaw():
    """REP-103: positive yaw is counter-clockwise, i.e. a LEFT turn.

    The right side travelling further than the left IS a left turn, so this
    is the test that catches the sides being swapped in `arc_step`.
    """
    _, dtheta = arc_step(0.0, 0.1)
    assert dtheta > 0.0


def test_arc_step_left_side_faster_is_negative_yaw():
    _, dtheta = arc_step(0.1, 0.0)
    assert dtheta < 0.0


def test_arc_step_opposite_displacements_is_pure_rotation():
    ds, dtheta = arc_step(-0.05, 0.05, wheel_separation=0.2)
    assert ds == pytest.approx(0.0)
    assert dtheta == pytest.approx(0.5)


def test_arc_step_uses_the_separation_it_is_given():
    """A wider effective track means the same wheels produce LESS yaw.

    Which is the whole point of the scrub correction: the uncorrected
    geometric track would over-predict the turn.
    """
    _, narrow = arc_step(0.0, 0.1, wheel_separation=GEOMETRIC_TRACK_M)
    _, wide = arc_step(0.0, 0.1, wheel_separation=WHEEL_SEPARATION_M)
    assert wide < narrow


# --------------------------------------------------------------------------
# advance: the arc integration
# --------------------------------------------------------------------------

def test_advance_straight_ahead_from_the_origin():
    pose = advance(Pose2D(), 0.25, 0.0)
    assert (pose.x, pose.y, pose.theta) == pytest.approx((0.25, 0.0, 0.0))


def test_advance_straight_along_the_current_heading():
    pose = advance(Pose2D(1.0, 2.0, math.pi / 2.0), 0.5, 0.0)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.5)
    assert pose.theta == pytest.approx(math.pi / 2.0)


def test_advance_rotation_in_place_does_not_translate():
    pose = advance(Pose2D(1.0, 2.0, 0.0), 0.0, 0.75)
    assert (pose.x, pose.y) == pytest.approx((1.0, 2.0))
    assert pose.theta == pytest.approx(0.75)


def test_advance_quarter_circle_to_the_left_closes_exactly():
    """One unit-radius quarter turn from the origin must land at (1, 1, pi/2).

    The closed form gets this in a single step; an Euler step would land at
    (pi/2, 0) - which is the systematic turn error this test exists to keep
    out.
    """
    pose = advance(Pose2D(), math.pi / 2.0, math.pi / 2.0)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(1.0)
    assert pose.theta == pytest.approx(math.pi / 2.0)


def test_advance_quarter_circle_to_the_right_mirrors_it():
    pose = advance(Pose2D(), math.pi / 2.0, -math.pi / 2.0)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(-1.0)
    assert pose.theta == pytest.approx(-math.pi / 2.0)


def test_advance_one_arc_equals_many_small_ones():
    """Subdividing an arc must not change where it ends.

    The independent check on the closed form: 500 small steps of the same
    total arc and turn have to agree with one big step.
    """
    ds, dtheta = 0.8, 1.1
    whole = advance(Pose2D(), ds, dtheta)

    steps = 500
    piecewise = Pose2D()
    for _ in range(steps):
        piecewise = advance(piecewise, ds / steps, dtheta / steps)

    assert piecewise.x == pytest.approx(whole.x, abs=1e-9)
    assert piecewise.y == pytest.approx(whole.y, abs=1e-9)
    assert piecewise.theta == pytest.approx(whole.theta, abs=1e-9)


def test_advance_straight_branch_matches_the_arc_branch_at_the_threshold():
    """The epsilon must not introduce a step in the output.

    Just above and just below `STRAIGHT_LINE_EPSILON_RAD` take different code
    paths, and they have to agree to well within anything measurable.
    """
    just_curved = advance(Pose2D(), 0.005, 1e-8)
    as_straight = advance(Pose2D(), 0.005, 0.0)
    assert just_curved.x == pytest.approx(as_straight.x, abs=1e-12)
    assert just_curved.y == pytest.approx(as_straight.y, abs=1e-10)


def test_advance_does_not_mutate_the_pose_it_was_given():
    start = Pose2D(1.0, 2.0, 0.5)
    advance(start, 1.0, 1.0)
    assert (start.x, start.y, start.theta) == (1.0, 2.0, 0.5)


def test_advance_normalizes_the_heading():
    pose = advance(Pose2D(0.0, 0.0, math.pi - 0.1), 0.1, 0.5)
    assert -math.pi < pose.theta <= math.pi


# --------------------------------------------------------------------------
# WheelOdometry: construction
# --------------------------------------------------------------------------

def test_wheel_odometry_starts_at_the_origin_with_no_anchor():
    odometry = WheelOdometry()
    assert (odometry.pose.x, odometry.pose.y, odometry.pose.theta) == (0.0, 0.0, 0.0)
    assert odometry.has_anchor is False


def test_wheel_odometry_defaults_to_the_robot_the_module_documents():
    odometry = WheelOdometry()
    assert odometry.wheel_radius == WHEEL_RADIUS_M
    assert odometry.wheel_separation == WHEEL_SEPARATION_M
    assert odometry.max_sample_gap_sec == MAX_SAMPLE_GAP_SEC
    assert odometry.max_wheel_speed == MAX_WHEEL_SPEED_MPS


@pytest.mark.parametrize('radius, separation', [
    (0.0, 0.2), (-0.04, 0.2), (0.04, 0.0), (0.04, -0.2),
])
def test_wheel_odometry_rejects_impossible_geometry(radius, separation):
    """Caught at construction, not silently per sample.

    A zero separation divides by zero on every message and a negative one
    mirrors the robot's steering - both far easier to diagnose as a refusal to
    start than as a map that comes out wrong.
    """
    with pytest.raises(InvalidGeometry):
        WheelOdometry(wheel_radius=radius, wheel_separation=separation)


# --------------------------------------------------------------------------
# WheelOdometry.update: the first sample
# --------------------------------------------------------------------------

def test_first_sample_has_nothing_to_integrate_but_still_yields_a_pose():
    """The origin pose IS the useful output of the first sample.

    Publishing it is what puts `odom -> base_footprint` into TF before
    slam_toolbox's first scan lookup, which is the entire reason this node
    exists.
    """
    odometry = WheelOdometry()
    update = odometry.update(100.0, 12.3, 45.6)
    assert update.status == UPDATE_FIRST_SAMPLE
    assert (update.pose.x, update.pose.y, update.pose.theta) == (0.0, 0.0, 0.0)
    assert (update.linear_velocity, update.angular_velocity) == (0.0, 0.0)
    assert odometry.has_anchor is True


def test_first_sample_ignores_however_large_the_accumulated_angles_are():
    """The robot does not teleport because the MCU has been counting a while."""
    odometry = WheelOdometry()
    odometry.update(100.0, 1e6, 1e6)
    update = odometry.update(100.1, 1e6, 1e6)
    assert update.status == UPDATE_OK
    assert (update.pose.x, update.pose.y) == pytest.approx((0.0, 0.0))


# --------------------------------------------------------------------------
# WheelOdometry.update: straight, rotating, turning
# --------------------------------------------------------------------------

def _angle_for(distance_m, wheel_radius=WHEEL_RADIUS_M):
    """The accumulated-angle delta the driver would report for a distance.

    `ugv02_serial_node` publishes `distance_to_angle(metres, radius)`, which
    is just `metres / radius`; this is that, inverted, so the tests can be
    written in metres.
    """
    return distance_m / wheel_radius


def test_straight_line_motion_is_pure_translation():
    """Both sides equal: forward only, and no yaw at all."""
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)
    travelled = 0.02
    update = odometry.update(
        0.1, _angle_for(travelled), _angle_for(travelled))

    assert update.status == UPDATE_OK
    assert update.pose.x == pytest.approx(travelled)
    assert update.pose.y == pytest.approx(0.0)
    assert update.pose.theta == pytest.approx(0.0)
    assert update.linear_velocity == pytest.approx(travelled / 0.1)
    assert update.angular_velocity == pytest.approx(0.0)


def test_straight_line_motion_accumulates_over_many_samples():
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)
    step = 0.0025  # ~0.2 m/s at 80 Hz
    angle = 0.0
    for i in range(1, 81):
        angle += _angle_for(step)
        update = odometry.update(i * 0.0125, angle, angle)
        assert update.status == UPDATE_OK

    assert update.pose.x == pytest.approx(80 * step)
    assert update.pose.y == pytest.approx(0.0)
    assert update.linear_velocity == pytest.approx(step / 0.0125)


def test_reverse_motion_goes_backwards():
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)
    update = odometry.update(0.1, _angle_for(-0.02), _angle_for(-0.02))
    assert update.pose.x == pytest.approx(-0.02)
    assert update.linear_velocity < 0.0


def test_in_place_rotation_to_the_left_does_not_translate():
    """Right side forward, left side back by the same amount: turn left, stay put."""
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)
    half_arc = 0.01
    update = odometry.update(
        0.1, _angle_for(-half_arc), _angle_for(half_arc))

    expected_yaw = 2.0 * half_arc / WHEEL_SEPARATION_M
    assert update.status == UPDATE_OK
    assert (update.pose.x, update.pose.y) == pytest.approx((0.0, 0.0))
    assert update.pose.theta == pytest.approx(expected_yaw)
    assert update.pose.theta > 0.0, 'right side forward must turn LEFT'
    assert update.linear_velocity == pytest.approx(0.0)
    assert update.angular_velocity == pytest.approx(expected_yaw / 0.1)


def test_in_place_rotation_to_the_right_mirrors_it():
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)
    update = odometry.update(0.1, _angle_for(0.01), _angle_for(-0.01))
    assert (update.pose.x, update.pose.y) == pytest.approx((0.0, 0.0))
    assert update.pose.theta < 0.0
    assert update.angular_velocity < 0.0


def test_combined_turn_moves_forward_and_to_the_left():
    """A real arc: both sides forward, the right one further."""
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)
    update = odometry.update(0.1, _angle_for(0.01), _angle_for(0.03))

    expected_ds = 0.02
    expected_dtheta = 0.02 / WHEEL_SEPARATION_M
    assert update.status == UPDATE_OK
    assert update.pose.theta == pytest.approx(expected_dtheta)
    assert update.pose.x > 0.0
    assert update.pose.y > 0.0, 'a left turn curves towards +y'
    assert update.pose.x < expected_ds, 'the arc is not a straight line'
    assert update.linear_velocity == pytest.approx(expected_ds / 0.1)
    assert update.angular_velocity == pytest.approx(expected_dtheta / 0.1)


def test_a_full_left_circle_returns_to_the_origin():
    """End-to-end: 360 degrees of constant turn must close the loop.

    This is the test that would catch a sign error, a factor of two, or a
    radius/separation mix-up, because all three stop the circle closing.
    """
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)

    turn_radius = 0.5
    steps = 720
    dtheta = 2.0 * math.pi / steps
    # Per step: ds = R * dtheta, and the sides differ by separation * dtheta.
    ds = turn_radius * dtheta
    d_left = ds - 0.5 * WHEEL_SEPARATION_M * dtheta
    d_right = ds + 0.5 * WHEEL_SEPARATION_M * dtheta

    left_angle = right_angle = 0.0
    for i in range(1, steps + 1):
        left_angle += _angle_for(d_left)
        right_angle += _angle_for(d_right)
        update = odometry.update(i * 0.01, left_angle, right_angle)
        assert update.status == UPDATE_OK

    assert update.pose.x == pytest.approx(0.0, abs=1e-9)
    assert update.pose.y == pytest.approx(0.0, abs=1e-9)
    assert update.pose.theta == pytest.approx(0.0, abs=1e-9)


# --------------------------------------------------------------------------
# WheelOdometry.update: the guards
# --------------------------------------------------------------------------

def test_a_stamp_that_does_not_advance_is_skipped():
    odometry = WheelOdometry()
    odometry.update(1.0, 0.0, 0.0)
    update = odometry.update(1.0, _angle_for(0.02), _angle_for(0.02))
    assert update.status == UPDATE_BACKWARD_TIME
    assert (update.pose.x, update.pose.y) == (0.0, 0.0)


def test_a_backwards_stamp_is_skipped():
    odometry = WheelOdometry()
    odometry.update(1.0, 0.0, 0.0)
    update = odometry.update(0.9, _angle_for(0.02), _angle_for(0.02))
    assert update.status == UPDATE_BACKWARD_TIME


def test_an_out_of_order_sample_does_not_cost_the_anchor():
    """The whole reason `UPDATE_BACKWARD_TIME` keeps the anchor.

    After a stale sample is dropped, the next good one must still measure from
    the last GOOD reading, so no distance is lost.
    """
    odometry = WheelOdometry()
    odometry.update(1.0, 0.0, 0.0)
    odometry.update(0.9, _angle_for(99.0), _angle_for(99.0))  # dropped

    update = odometry.update(1.1, _angle_for(0.02), _angle_for(0.02))
    assert update.status == UPDATE_OK
    assert update.pose.x == pytest.approx(0.02)


def test_a_long_gap_is_re_anchored_and_not_integrated():
    """A gap is not a fair arc, so the pose holds still instead of jumping."""
    odometry = WheelOdometry(max_sample_gap_sec=0.5)
    odometry.update(0.0, 0.0, 0.0)
    update = odometry.update(10.0, _angle_for(3.0), _angle_for(-3.0))
    assert update.status == UPDATE_GAP
    assert (update.pose.x, update.pose.y, update.pose.theta) == (0.0, 0.0, 0.0)


def test_the_sample_after_a_gap_integrates_normally():
    """Re-anchoring means the gap costs one interval, not the whole run."""
    odometry = WheelOdometry(max_sample_gap_sec=0.5)
    odometry.update(0.0, 0.0, 0.0)
    odometry.update(10.0, _angle_for(3.0), _angle_for(3.0))  # the gap

    update = odometry.update(
        10.1, _angle_for(3.0 + 0.02), _angle_for(3.0 + 0.02))
    assert update.status == UPDATE_OK
    assert update.pose.x == pytest.approx(0.02)


def test_a_gap_exactly_at_the_limit_is_still_integrated():
    """The bound is inclusive, so jitter right at it does not flap."""
    odometry = WheelOdometry(max_sample_gap_sec=0.5)
    odometry.update(0.0, 0.0, 0.0)
    update = odometry.update(0.5, _angle_for(0.02), _angle_for(0.02))
    assert update.status == UPDATE_OK


def test_an_impossible_wheel_speed_is_skipped():
    """An accumulator reset looks like metres of travel in one sample."""
    odometry = WheelOdometry(max_wheel_speed=1.0)
    odometry.update(0.0, 0.0, 0.0)
    update = odometry.update(0.0125, _angle_for(5.0), _angle_for(5.0))
    assert update.status == UPDATE_IMPLAUSIBLE
    assert (update.pose.x, update.pose.y) == (0.0, 0.0)


def test_an_impossible_wheel_speed_in_reverse_is_also_skipped():
    odometry = WheelOdometry(max_wheel_speed=1.0)
    odometry.update(0.0, 0.0, 0.0)
    update = odometry.update(0.0125, _angle_for(-5.0), _angle_for(-5.0))
    assert update.status == UPDATE_IMPLAUSIBLE


def test_one_side_alone_can_trip_the_speed_guard():
    """`max` over both sides, not an average that a single bad side could hide."""
    odometry = WheelOdometry(max_wheel_speed=1.0)
    odometry.update(0.0, 0.0, 0.0)
    update = odometry.update(0.0125, 0.0, _angle_for(5.0))
    assert update.status == UPDATE_IMPLAUSIBLE


def test_a_fast_but_possible_sample_is_not_rejected():
    """0.3 m/s is nav2's configured top speed; it must sail through."""
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)
    travelled = 0.3 * 0.0125
    update = odometry.update(
        0.0125, _angle_for(travelled), _angle_for(travelled))
    assert update.status == UPDATE_OK


def test_the_pose_survives_every_skipped_sample_unchanged():
    """Whatever a sample's fate, the pose it reports is the one to publish."""
    odometry = WheelOdometry()
    odometry.update(0.0, 0.0, 0.0)
    odometry.update(0.1, _angle_for(0.02), _angle_for(0.02))
    good = odometry.pose

    for update in (odometry.update(0.05, _angle_for(9.0), _angle_for(9.0)),
                   odometry.update(5.0, _angle_for(9.0), _angle_for(9.0)),
                   odometry.update(5.01, _angle_for(99.0), _angle_for(99.0))):
        assert update.status != UPDATE_OK
        assert update.pose == good

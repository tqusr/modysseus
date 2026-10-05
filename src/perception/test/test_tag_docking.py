"""Unit tests for perception.tag_docking -- the pure logic behind
`tag_dock_server` (which answers /dock_robot).

Plain Python only, like test_dock_capture.py: no rclpy, tf2 or ROS graph.
"""

import math
import os

import pytest

from perception.dock_capture import (STAGING_STANDOFF_M, Pose2D,
                                     capture_from_tag_transform)
from perception.tag_docking import (BACKWARD, FORWARD, SPIN,
                                    MeasurementWindow, PredockError, Tolerance,
                                    compose, correction, inverse,
                                    load_dock_database, mean_pose, needs_replan,
                                    plan_alignment, predock_error,
                                    predock_pose_for, yaw_from_quaternion)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'docking.yaml')


def assert_pose_close(a: Pose2D, b: Pose2D, tol=1e-6):
    assert a.x == pytest.approx(b.x, abs=tol)
    assert a.y == pytest.approx(b.y, abs=tol)
    assert math.remainder(a.theta - b.theta, math.tau) == pytest.approx(0.0, abs=tol)


def wall_tag_quaternion(outward_theta):
    """Quaternion of a wall-mounted tag frame whose +z (outward) is horizontal
    at heading `outward_theta`, +y up, +x = y cross z."""
    c, s = math.cos(outward_theta), math.sin(outward_theta)
    # Columns are the tag's local x, y, z axes expressed in the parent frame.
    m = [[-s, 0.0, c],
         [c, 0.0, s],
         [0.0, 1.0, 0.0]]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        k = math.sqrt(trace + 1.0) * 2.0
        return ((m[2][1] - m[1][2]) / k, (m[0][2] - m[2][0]) / k,
                (m[1][0] - m[0][1]) / k, 0.25 * k)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        k = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return (0.25 * k, (m[0][1] + m[1][0]) / k,
                (m[0][2] + m[2][0]) / k, (m[2][1] - m[1][2]) / k)
    if m[1][1] > m[2][2]:
        k = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return ((m[0][1] + m[1][0]) / k, 0.25 * k,
                (m[1][2] + m[2][1]) / k, (m[0][2] - m[2][0]) / k)
    k = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return ((m[0][2] + m[2][0]) / k, (m[1][2] + m[2][1]) / k,
            0.25 * k, (m[1][0] - m[0][1]) / k)


# --------------------------------------------------------------------------
# Dock database
# --------------------------------------------------------------------------

def test_loads_every_dock_from_the_real_docking_yaml():
    with open(CONFIG, encoding='utf-8') as stream:
        database = load_dock_database(stream.read())
    assert sorted(database) == list('ABCDEFGH')
    assert [database[letter].tag_id for letter in 'ABCDEFGH'] == list(range(8))


def test_rejects_a_dock_not_in_map():
    text = '''
docking_server:
  ros__parameters:
    docks: ["A"]
    A: {type: "arena_dock", frame: "odom", pose: [1.0, 2.0, 0.0], id: "0"}
'''
    with pytest.raises(ValueError, match='odom'):
        load_dock_database(text)


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------

def test_compose_with_inverse_is_identity():
    a = Pose2D(1.2, -0.7, 2.5)
    assert_pose_close(compose(a, inverse(a)), Pose2D(0.0, 0.0, 0.0))
    assert_pose_close(compose(inverse(a), a), Pose2D(0.0, 0.0, 0.0))


def test_yaw_from_quaternion_round_trips():
    for theta in (-3.0, -1.0, 0.0, 0.5, 3.1):
        q = (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))
        assert yaw_from_quaternion(*q) == pytest.approx(theta)


@pytest.mark.parametrize('outward_theta', [-3.1, -1.6, -0.08, 0.0, 1.55, 3.0])
def test_docking_pose_maths_is_frame_independent(outward_theta):
    """A tag converted to a docking pose in odom and then moved to map equals
    the same tag converted in map. The server relies on this to measure in
    odom with capture_from_tag_transform (written for map)."""
    map_to_odom = Pose2D(0.31, -0.12, 0.2)
    tag_in_odom = Pose2D(-0.9, -1.2, outward_theta)
    tag_in_map = compose(map_to_odom, tag_in_odom)
    in_odom = capture_from_tag_transform(
        4, (tag_in_odom.x, tag_in_odom.y), wall_tag_quaternion(tag_in_odom.theta))
    in_map = capture_from_tag_transform(
        4, (tag_in_map.x, tag_in_map.y), wall_tag_quaternion(tag_in_map.theta))
    assert in_odom.dock_pose.theta == pytest.approx(outward_theta)
    assert_pose_close(compose(map_to_odom, in_odom.docking_pose),
                      in_map.docking_pose)


def test_correction_is_measured_minus_stored():
    dx, dy, dtheta = correction(Pose2D(1.0, 2.0, 3.1), Pose2D(1.1, 1.9, -3.1))
    assert (dx, dy) == (pytest.approx(0.1), pytest.approx(-0.1))
    assert dtheta == pytest.approx(math.tau - 6.2)   # wraps across +-pi


def test_mean_pose_averages_heading_across_pi():
    mean = mean_pose([Pose2D(0.0, 0.0, 3.1), Pose2D(2.0, 0.0, -3.1)])
    assert (mean.x, mean.y) == (pytest.approx(1.0), pytest.approx(0.0))
    assert abs(mean.theta) == pytest.approx(math.pi)


# --------------------------------------------------------------------------
# Pre-dock pose and tolerance
# --------------------------------------------------------------------------

@pytest.mark.parametrize('theta', [0.0, 1.0, math.pi, -1.6])
def test_predock_is_straight_back_from_the_dock(theta):
    dock = Pose2D(1.0, -2.0, theta)
    predock = predock_pose_for(dock)
    assert predock.theta == pytest.approx(dock.theta)
    # Driving forward STAGING_STANDOFF_M from the pre-dock lands on the dock.
    assert_pose_close(compose(predock, Pose2D(STAGING_STANDOFF_M, 0.0, 0.0)), dock)


def test_predock_matches_capture_staging_convention():
    """capture_dock_pose's docks.yaml staging pose sits 0.6 m out from the
    dock WALL; the pre-dock sits 0.6 m back from the ROBOT's docked pose.
    Both share the approach heading."""
    capture = capture_from_tag_transform(4, (0.0, 0.0), wall_tag_quaternion(0.0))
    predock = predock_pose_for(capture.docking_pose)
    assert predock.theta == pytest.approx(capture.staging_pose.theta)


def test_predock_error_axes():
    predock = Pose2D(1.0, 1.0, math.pi / 2)   # approach heading +y
    # 0.1 m further in (+y), 0.05 m to the robot's left (-x), turned 5 deg left.
    robot = Pose2D(0.95, 1.1, math.pi / 2 + math.radians(5))
    error = predock_error(robot, predock)
    assert error.along == pytest.approx(0.1)
    assert error.lateral == pytest.approx(0.05)
    assert math.degrees(error.heading) == pytest.approx(5.0)


TOL = Tolerance(lateral_m=0.10, along_m=0.20, heading_rad=0.25)


@pytest.mark.parametrize('error,ok', [
    (PredockError(0.0, 0.0, 0.0), True),
    (PredockError(-0.19, 0.09, -0.24), True),
    (PredockError(0.0, 0.11, 0.0), False),
    (PredockError(0.21, 0.0, 0.0), False),
    (PredockError(0.0, 0.0, -0.26), False),
])
def test_tolerance(error, ok):
    assert TOL.accepts(error) is ok


def test_scaled_tolerance_is_the_give_up_window():
    wider = TOL.scaled(2.0)
    assert wider.accepts(PredockError(0.0, 0.15, 0.0))
    assert not wider.accepts(PredockError(0.0, 0.25, 0.0))


def test_replan_only_for_real_moves():
    goal = Pose2D(0.0, 0.0, 0.0)
    assert not needs_replan(goal, Pose2D(0.03, 0.02, 0.02), 0.05, 0.05)
    assert needs_replan(goal, Pose2D(0.06, 0.0, 0.0), 0.05, 0.05)
    assert needs_replan(goal, Pose2D(0.0, 0.0, 0.08), 0.05, 0.05)


# --------------------------------------------------------------------------
# Measurement window
# --------------------------------------------------------------------------

def window():
    return MeasurementWindow(required=3, spread_m=0.04, spread_rad=0.06,
                             max_age_sec=2.0)


def test_needs_enough_samples():
    w = window()
    w.add(Pose2D(1.0, 1.0, 0.0), 10.0)
    w.add(Pose2D(1.01, 1.0, 0.0), 10.2)
    assert w.stable(10.3) is None
    w.add(Pose2D(1.02, 1.0, 0.0), 10.4)
    assert w.stable(10.5).x == pytest.approx(1.01)


def test_disagreeing_samples_are_not_trusted_until_they_settle():
    w = window()
    for t, x in ((0.0, 1.0), (0.2, 1.10), (0.4, 1.0)):
        w.add(Pose2D(x, 0.0, 0.0), t)
    assert w.stable(0.5) is None
    for t in (0.6, 0.8, 1.0):
        w.add(Pose2D(1.0, 0.0, 0.0), t)
    assert w.stable(1.1).x == pytest.approx(1.0)


def test_old_samples_expire():
    w = window()
    for t in (0.0, 0.2, 0.4):
        w.add(Pose2D(1.0, 0.0, 0.0), t)
    assert w.stable(0.5) is not None
    assert w.stable(2.3) is None


def test_clear_forgets_everything():
    w = window()
    for t in (0.0, 0.2, 0.4):
        w.add(Pose2D(1.0, 0.0, 0.0), t)
    w.clear()
    assert w.stable(0.5) is None


# --------------------------------------------------------------------------
# Alignment: turn - drive - turn
# --------------------------------------------------------------------------

def execute(robot: Pose2D, moves) -> Pose2D:
    """Apply planned moves to a pose, as an ideal skid-steer robot would."""
    for kind, amount in moves:
        if kind == SPIN:
            robot = compose(robot, Pose2D(0.0, 0.0, amount))
        elif kind == FORWARD:
            robot = compose(robot, Pose2D(amount, 0.0, 0.0))
        else:
            robot = compose(robot, Pose2D(-amount, 0.0, 0.0))
    return robot


# Docks C and B on the 2026-10-05 map: same wall, ~0.4 m apart.
DOCK_C = Pose2D(0.374, 1.635, 1.58198)
DOCK_B = Pose2D(-0.044, 1.657, 1.52546)


def test_moving_from_c_predock_to_b_predock_ends_square_on_b():
    """The reported failure: after undocking from C the robot sits at C's
    pre-dock pose and must line up on B's before driving in."""
    at_c = predock_pose_for(DOCK_C)
    target = predock_pose_for(DOCK_B)
    moves = plan_alignment(at_c, target)
    kinds = [kind for kind, _ in moves]
    assert kinds == [SPIN, FORWARD, SPIN]
    # About a quarter turn each way (B is to the robot's left, facing the wall).
    assert math.degrees(moves[0][1]) == pytest.approx(90, abs=10)
    end = execute(at_c, moves)
    assert_pose_close(end, target, tol=1e-9)
    error = predock_error(end, target)
    assert Tolerance(0.03, 0.08, 0.05).accepts(error)


@pytest.mark.parametrize('start', [
    Pose2D(0.0, 0.0, 0.0), Pose2D(0.5, -0.3, 2.0), Pose2D(-0.2, 0.4, -2.9),
    Pose2D(0.05, 0.02, 0.1)])
def test_alignment_always_lands_on_target(start):
    target = Pose2D(0.1, 0.2, 1.0)
    assert_pose_close(execute(start, plan_alignment(start, target)), target, tol=1e-9)


def test_target_behind_backs_up_instead_of_turning_around():
    robot = Pose2D(0.0, 0.0, 0.0)
    target = Pose2D(-0.2, 0.0, 0.0)          # 0.2 m straight behind
    assert plan_alignment(robot, target) == [(BACKWARD, pytest.approx(0.2))]


def test_turns_never_exceed_a_quarter_turn_to_translate():
    robot = Pose2D(0.0, 0.0, 0.0)
    for angle in range(-180, 180, 15):
        a = math.radians(angle)
        moves = plan_alignment(robot, Pose2D(0.3 * math.cos(a), 0.3 * math.sin(a), 0.0))
        first = moves[0]
        if first[0] == SPIN:
            assert abs(first[1]) <= math.pi / 2 + 1e-9


def test_tiny_errors_produce_no_moves():
    robot = Pose2D(1.0, 1.0, 0.5)
    target = Pose2D(1.005, 1.0, 0.51)
    assert plan_alignment(robot, target) == []


def test_heading_only_error_is_a_single_spin():
    moves = plan_alignment(Pose2D(1.0, 1.0, 0.5), Pose2D(1.0, 1.0, 0.7))
    assert moves == [(SPIN, pytest.approx(0.2))]

"""Unit tests for localization.wheel_drive, the maths behind `wheel_drive_node.py`.

No ROS, serial port or robot. They cannot say what the MCU does with a T:1 line
(units, speed ceiling, deadband); that is a hardware check.
"""

import json
import math

import pytest

from localization.wheel_drive import encode_wheels, twist_to_wheels

SEP = 0.4084


def test_straight_line_drives_both_sides_equally():
    assert twist_to_wheels(0.3, 0.0, SEP) == (0.3, 0.3)


def test_pivot_in_place_is_equal_and_opposite():
    left, right = twist_to_wheels(0.0, 1.0, SEP)
    assert left == pytest.approx(-SEP / 2)
    assert right == pytest.approx(SEP / 2)


def test_positive_yaw_turns_left_so_right_side_is_faster():
    left, right = twist_to_wheels(0.2, 0.5, SEP)
    assert right > left


def test_round_trip_recovers_the_twist():
    left, right = twist_to_wheels(0.25, -0.8, SEP)
    assert (right + left) / 2 == pytest.approx(0.25)
    assert (right - left) / SEP == pytest.approx(-0.8)


def test_effective_separation_asks_for_a_bigger_wheel_difference():
    """The point of the exercise: 0.4084 needs 2.34x the geometric difference."""
    eff = twist_to_wheels(0.0, 1.0, 0.4084)
    geo = twist_to_wheels(0.0, 1.0, 0.17452)
    assert eff[1] / geo[1] == pytest.approx(0.4084 / 0.17452)


def test_limit_scales_both_sides_and_keeps_the_ratio():
    left, right = twist_to_wheels(0.3, 1.5, SEP, max_wheel_speed=0.4)
    assert max(abs(left), abs(right)) == pytest.approx(0.4)
    unlimited = twist_to_wheels(0.3, 1.5, SEP)
    assert left / right == pytest.approx(unlimited[0] / unlimited[1])


def test_limit_does_not_touch_a_command_inside_it():
    assert twist_to_wheels(0.1, 0.2, SEP, max_wheel_speed=0.5) == \
        twist_to_wheels(0.1, 0.2, SEP)


def test_zero_limit_means_no_limit():
    left, right = twist_to_wheels(0.3, 3.0, SEP, max_wheel_speed=0.0)
    assert right > 0.5


@pytest.mark.parametrize('separation', [0.0, -0.1])
def test_rejects_a_non_positive_separation(separation):
    with pytest.raises(ValueError):
        twist_to_wheels(0.1, 0.1, separation)


def test_encode_is_one_json_line_with_the_documented_fields():
    data = encode_wheels(0.5, -0.25)
    assert data.endswith(b'\n') and data.count(b'\n') == 1
    assert json.loads(data) == {'T': 1, 'L': 0.5, 'R': -0.25}


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_encode_refuses_non_finite_speeds(bad):
    with pytest.raises(ValueError):
        encode_wheels(bad, 0.0)
    with pytest.raises(ValueError):
        encode_wheels(0.0, bad)


def test_angular_cap_limits_the_turn_not_the_drive():
    left, right = twist_to_wheels(0.3, 1.5, SEP, max_angular=0.75)
    assert (right - left) / SEP == pytest.approx(0.75)
    assert (right + left) / 2 == pytest.approx(0.3)


def test_angular_cap_applies_to_both_directions():
    left, right = twist_to_wheels(0.0, -3.0, SEP, max_angular=0.75)
    assert (right - left) / SEP == pytest.approx(-0.75)


def test_angular_cap_leaves_a_slower_turn_alone():
    assert twist_to_wheels(0.1, 0.4, SEP, max_angular=0.75) == \
        twist_to_wheels(0.1, 0.4, SEP)

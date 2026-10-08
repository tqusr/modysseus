"""Differential-drive kinematics for the UGV02, run on the Jetson.

No ROS and no serial port, so it is unit-tested without either. The node that
uses it is `scripts/wheel_drive_node.py`.

The vendor driver sends the MCU a body twist (`{"T":13,"X":..,"Z":..}`) and
the MCU applies its own track width, which we cannot see or change. The
6-wheel skid-steer scrubs sideways to turn, so the real yaw rate is well
below `Z` (`wheel_odometry.py` measured an effective separation of 0.4084 m
against a 0.17452 m geometric track). This module instead turns (v, w) into
the per-side speeds of `{"T":1,"L":..,"R":..}` (m/s) using the effective
separation, so a commanded yaw rate is what the robot does:

    left  = v - w * separation / 2
    right = v + w * separation / 2

This is the `diff_drive_controller` maths, without its ros2_control
dependencies.

`separation` comes from `config/hardware.yaml`, the single place the turn
calibration lives, so odometry and drive cannot disagree. It is a single
clockwise measurement.
"""

import json
import math

#: Wire type for a per-side wheel speed command.
CMD_WHEELS_TYPE = 1


def twist_to_wheels(linear, angular, separation, max_wheel_speed=0.0,
                    max_angular=0.0):
    """Converts a body twist to wheel speeds.

    A positive `max_angular` caps turn rate without limiting drive speed,
    which a wheel-speed limit cannot do. A positive `max_wheel_speed` scales
    both sides by the same factor when either exceeds it, so the robot keeps
    its curve and only slows down.

    Args:
        linear: Forward speed in m/s.
        angular: Yaw rate in rad/s.
        separation: Effective wheel separation in m.
        max_wheel_speed: Per-side speed limit in m/s; 0 disables it.
        max_angular: Yaw-rate limit in rad/s; 0 disables it.

    Returns:
        `(left, right)` wheel speeds in m/s.

    Raises:
        ValueError: If `separation` is not positive.
    """
    if separation <= 0.0:
        raise ValueError('separation must be positive, got {}'.format(separation))
    if max_angular > 0.0:
        angular = max(-max_angular, min(max_angular, angular))
    half = angular * separation / 2.0
    left = linear - half
    right = linear + half
    if max_wheel_speed > 0.0:
        peak = max(abs(left), abs(right))
        if peak > max_wheel_speed:
            scale = max_wheel_speed / peak
            left *= scale
            right *= scale
    return left, right


def encode_wheels(left, right):
    """Encodes a `T:1` command line, newline included, ready for the port.

    Args:
        left: Left wheel speed in m/s.
        right: Right wheel speed in m/s.

    Returns:
        The UTF-8 encoded line.

    Raises:
        ValueError: If a speed is NaN or infinite. `json.dumps` would write
            bare `NaN`/`Infinity`, which is not JSON and would desynchronise
            the MCU's parser.
    """
    left = float(left)
    right = float(right)
    if not math.isfinite(left) or not math.isfinite(right):
        raise ValueError('non-finite wheel speeds: L={}, R={}'.format(left, right))
    return (json.dumps({'T': CMD_WHEELS_TYPE, 'L': left, 'R': right})
            + '\n').encode('utf-8')

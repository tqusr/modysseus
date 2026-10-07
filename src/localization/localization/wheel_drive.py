"""Differential-drive kinematics for the UGV02, run on the Jetson.

No ROS and no serial port in here, so it is unit-tested without either. The
node that uses it is `scripts/wheel_drive_node.py`.

WHY THIS EXISTS. The vendor driver sends the MCU a body twist, `{"T":13,"X":..,
"Z":..}`, and the MCU does the wheel kinematics itself, with a track width we
cannot see or change. The chassis is a 6-wheel skid-steer, which has to scrub
its wheels sideways to rotate, so the real yaw rate comes out well below `Z`
(`wheel_odometry.py` measured the effective separation at 0.4084 m against a
0.17452 m geometric track: a scrub factor of 2.34). The alternative the MCU
offers is `{"T":1,"L":..,"R":..}`, a speed for each side, in m/s. This module
turns (v, w) into that pair using the EFFECTIVE separation, so a commanded
yaw rate is what the robot does.

    left  = v - w * separation / 2
    right = v + w * separation / 2

This is the same maths `diff_drive_controller` runs. That package is a
ros2_control plugin and needs a controller manager and a hardware interface;
neither is installed or worth adding for two numbers, so the kinematics are
here and the serial write is in the node.

`separation` is read from `config/hardware.yaml`, the single place the robot's
turn calibration lives, so the odometry and this controller cannot disagree.
It is a single clockwise measurement; see that file.
"""

import json
import math

#: Wire type for a per-side wheel speed command.
CMD_WHEELS_TYPE = 1


def twist_to_wheels(linear, angular, separation, max_wheel_speed=0.0,
                    max_angular=0.0):
    """Return `(left, right)` wheel speeds in m/s for a body twist.

    If `max_angular` is positive, |angular| is clamped to it first. That caps
    how fast the robot may turn without touching how fast it may drive, which
    a wheel-speed limit cannot do: a pivot needs only about 0.15 m/s per side
    at 0.75 rad/s, far under any limit that still lets it drive straight.

    If `max_wheel_speed` is positive and either side would exceed it, both are
    scaled down by the same factor. That keeps the ratio of the two sides, so
    the robot follows the same curve more slowly instead of a flatter one, which
    is what `diff_drive_controller` does too. Clamping each side on its own
    would turn a hard turn into a gentler one.
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
    """Encode a `T:1` command line, newline included, ready for the port.

    Raises ValueError for NaN or infinity: `json.dumps` would write the bare
    tokens `NaN`/`Infinity`, which are not JSON and would desynchronise the
    MCU's parser. The vendor driver guards `T:13` the same way.
    """
    left = float(left)
    right = float(right)
    if not math.isfinite(left) or not math.isfinite(right):
        raise ValueError('non-finite wheel speeds: L={}, R={}'.format(left, right))
    return (json.dumps({'T': CMD_WHEELS_TYPE, 'L': left, 'R': right})
            + '\n').encode('utf-8')

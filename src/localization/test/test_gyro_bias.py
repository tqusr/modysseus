"""Unit tests for localization.gyro_bias -- the pure logic behind
`ros2 run localization gyro_bias_node.py`.

No rclpy, no ROS graph, no robot. The streams below are synthetic:
gyro and encoders at 80 Hz with identical stamps, the way the vendor driver
publishes them from one `T:1001` line.

WHAT THESE TESTS CANNOT DO: confirm the real ICM-20948's bias, its drift with
temperature, or that its z axis points up on the real board. Those are
hardware checks, listed in the fusion plan's Task 6.
"""

import random

import pytest

from localization.gyro_bias import (ENCODER_TIMEOUT_SEC,
                                    MAX_STILL_RATE_RADPS,
                                    STARTUP_CALIBRATION_SEC, STILL_HOLD_SEC,
                                    GyroBias)

RATE_HZ = 80.0
DT = 1.0 / RATE_HZ
#: The ICM-20948's per-sample noise, as in model.sdf and the vendor driver.
GYRO_NOISE = 2.62e-3


def run(estimator, start, duration, bias, true_rate=0.0, wheel_speed=0.0,
        seed=0, encoders=True, angles=(0.0, 0.0)):
    """Feed `duration` seconds of samples; return (end_time, angles, corrections).

    `wheel_speed` is per side, rad/s of wheel angle, equal on both sides.
    """
    rng = random.Random(seed)
    left, right = angles
    corrections = []
    steps = int(round(duration * RATE_HZ))
    t = start
    for _ in range(steps):
        t += DT
        left += wheel_speed * DT
        right += wheel_speed * DT
        if encoders:
            estimator.add_encoders(t, left, right)
        corrections.append(estimator.correct(
            t, true_rate + bias + rng.gauss(0.0, GYRO_NOISE)))
    return t, (left, right), corrections


def test_startup_calibration_recovers_constant_bias():
    estimator = GyroBias()
    run(estimator, 0.0, STILL_HOLD_SEC + STARTUP_CALIBRATION_SEC + 0.2, 0.03)
    assert estimator.calibrated
    assert estimator.bias == pytest.approx(0.03, abs=1e-3)


def test_corrected_rate_is_measured_minus_bias():
    estimator = GyroBias()
    t, angles, _ = run(estimator, 0.0, 3.0, 0.03)
    _, _, corrections = run(estimator, t, 1.0, 0.03, true_rate=0.5,
                            wheel_speed=5.0, angles=angles)
    rates = [c.rate for c in corrections]
    assert sum(rates) / len(rates) == pytest.approx(0.5, abs=2e-3)


def test_not_calibrated_before_enough_still_time():
    estimator = GyroBias()
    run(estimator, 0.0, STILL_HOLD_SEC + 0.5 * STARTUP_CALIBRATION_SEC, 0.03)
    assert not estimator.calibrated


def test_hold_time_must_pass_before_samples_count():
    estimator = GyroBias()
    _, _, corrections = run(estimator, 0.0, 0.9 * STILL_HOLD_SEC, 0.03)
    assert not any(c.used_for_bias for c in corrections)
    assert estimator.bias == 0.0


def test_motion_does_not_change_bias():
    estimator = GyroBias()
    t, angles, _ = run(estimator, 0.0, 3.0, 0.03)
    calibrated_bias = estimator.bias
    # Turning hard, wheels moving: a large rate that must not leak into bias.
    _, _, corrections = run(estimator, t, 5.0, 0.03, true_rate=1.0,
                            wheel_speed=5.0, angles=angles)
    assert not any(c.used_for_bias for c in corrections)
    assert estimator.bias == calibrated_bias


def test_slow_creep_counts_as_motion():
    """Motion below epsilon per sample still adds up past it at the anchor."""
    estimator = GyroBias()
    # 0.004 rad/s is 5e-5 rad per sample, far below the 1e-3 epsilon.
    _, _, corrections = run(estimator, 0.0, 3.0, 0.03, wheel_speed=0.004)
    used = sum(c.used_for_bias for c in corrections)
    assert used < 0.5 * len(corrections)


def test_no_encoders_means_no_bias_learning():
    estimator = GyroBias()
    _, _, corrections = run(estimator, 0.0, 3.0, 0.03, encoders=False)
    assert not any(c.used_for_bias for c in corrections)
    assert not estimator.calibrated


def test_stale_encoders_stop_bias_learning():
    estimator = GyroBias()
    t, _, _ = run(estimator, 0.0, 3.0, 0.03)
    bias = estimator.bias
    _, _, corrections = run(estimator, t, 2.0, 0.03, encoders=False)
    # The last encoder sample still vouches for ENCODER_TIMEOUT_SEC; after
    # that nothing may be learned.
    fresh = int(ENCODER_TIMEOUT_SEC * RATE_HZ)
    assert not any(c.used_for_bias for c in corrections[fresh:])
    learned_until_timeout = estimator.bias
    run(estimator, t + 2.0, 2.0, 0.05, encoders=False)
    assert estimator.bias == learned_until_timeout
    assert learned_until_timeout == pytest.approx(bias, abs=1e-3)


def test_bump_while_parked_is_rejected():
    estimator = GyroBias()
    t, _, _ = run(estimator, 0.0, 3.0, 0.03)
    bias = estimator.bias
    t += DT
    estimator.add_encoders(t, 0.0, 0.0)
    correction = estimator.correct(t, 0.03 + 10 * MAX_STILL_RATE_RADPS)
    assert not correction.used_for_bias
    assert estimator.bias == bias


def test_tracks_slow_drift_after_calibration():
    estimator = GyroBias()
    t, _, _ = run(estimator, 0.0, 3.0, 0.03)
    # Bias steps by 0.01 rad/s (inside max_still_rate); after ~5 time
    # constants of standing still the average has followed it.
    run(estimator, t, 150.0, 0.04)
    assert estimator.bias == pytest.approx(0.04, abs=1e-3)


def test_out_of_order_gyro_sample_is_corrected_but_not_learned():
    estimator = GyroBias()
    t, _, _ = run(estimator, 0.0, 3.0, 0.03)
    bias = estimator.bias
    correction = estimator.correct(t - 0.5, 0.03)
    assert not correction.used_for_bias
    assert correction.rate == pytest.approx(0.03 - bias)
    assert estimator.bias == bias


def test_negative_bias_sign_is_preserved():
    estimator = GyroBias()
    run(estimator, 0.0, 3.0, -0.05)
    assert estimator.bias == pytest.approx(-0.05, abs=1e-3)


@pytest.mark.parametrize('field', ['time_constant', 'startup_sec'])
def test_rejects_non_positive_times(field):
    with pytest.raises(ValueError):
        GyroBias(**{field: 0.0})

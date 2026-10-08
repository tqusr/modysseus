"""Pure gyro-bias estimation behind `gyro_bias_node.py`.

Imports no ROS, so it is unit-testable without a graph or a robot (see
`localization/test/test_gyro_bias.py`).

`ekf.yaml` takes the turn rate from the chassis gyro (ICM-20948) and
integrates it into heading. A gyro reads a small constant offset (bias) on
top of the true rate: up to +/-5 deg/s out of the box per the datasheet, and
it drifts with temperature. A bias of 0.1 deg/s is already 6 degrees of
heading per minute while parked, and robot_localization does not estimate it.

A gyro on a standing-still robot should read zero, so whatever it reads then
is the bias. The wheels say when the robot is still. Two phases:

  - Startup calibration: the first `startup_sec` seconds of standing still
    are averaged with equal weight.
  - Zero-velocity updates: afterwards each standing-still sample nudges the
    bias through an exponential moving average with time constant
    `time_constant`, following slow drift without reacting to noise.

While the robot moves, the bias is held and only subtracted.

Standing still means neither side's accumulated encoder angle has moved more
than `still_angle_epsilon` from where it was at least `still_hold` seconds
ago. The anchor comparison means very slow motion still accumulates past the
threshold. One hardware encoder count is a whole centimetre, far above the
threshold, so any count change is motion.

A standing-still sample is still refused when:

  - the encoder data is older than `encoder_timeout` at the gyro's stamp, or
  - the reading is implausible for a parked robot (`max_startup_rate` before
    calibration, `max_still_rate` from the bias after it), e.g. the robot
    was bumped or lifted.

Only the z axis is handled: `ekf.yaml` runs in `two_d_mode` and fuses only
yaw rate.
"""

from dataclasses import dataclass

#: Largest encoder-angle change (rad) treated as not moving. One hardware
#: count is 0.25 rad, so this never hides a real count.
STILL_ANGLE_EPSILON_RAD = 1e-3

#: Seconds the encoders must be still before gyro samples count as bias.
STILL_HOLD_SEC = 0.5

#: Seconds of standing-still data averaged before the bias is "calibrated".
STARTUP_CALIBRATION_SEC = 2.0

#: Time constant (s) of the post-calibration moving average: long against
#: noise (2.62e-3 rad/s per sample), short against thermal drift (minutes).
BIAS_TIME_CONSTANT_SEC = 30.0

#: Oldest encoder sample (s before the gyro stamp) that still vouches for
#: standing still. Encoders run near 80 Hz.
ENCODER_TIMEOUT_SEC = 0.2

#: Largest |rate| (rad/s) accepted as bias during calibration; above the
#: ICM-20948's worst-case +/-5 deg/s (0.087 rad/s) offset.
MAX_STARTUP_RATE_RADPS = 0.2

#: Largest |rate - bias| (rad/s) accepted after calibration; about 8 sigma of
#: per-sample noise.
MAX_STILL_RATE_RADPS = 0.02

#: Longest gyro gap (s) counted as calibration time or an averaging step, so
#: a stalled stream does not look like seconds of data.
MAX_GYRO_GAP_SEC = 0.1


@dataclass(frozen=True)
class Correction:
    """The result of one gyro sample.

    Attributes:
        rate: Bias-corrected yaw rate, rad/s.
        bias: Bias that was subtracted, rad/s.
        used_for_bias: True if this sample updated the bias.
    """

    rate: float
    bias: float
    used_for_bias: bool


class GyroBias:
    """Estimates and removes the gyro's yaw-rate bias.

    Stateful but ROS-free: the node feeds it encoder and gyro samples, each
    with its header stamp in seconds. See the module docstring.
    """

    def __init__(self,
                 still_angle_epsilon=STILL_ANGLE_EPSILON_RAD,
                 still_hold=STILL_HOLD_SEC,
                 startup_sec=STARTUP_CALIBRATION_SEC,
                 time_constant=BIAS_TIME_CONSTANT_SEC,
                 encoder_timeout=ENCODER_TIMEOUT_SEC,
                 max_startup_rate=MAX_STARTUP_RATE_RADPS,
                 max_still_rate=MAX_STILL_RATE_RADPS,
                 max_gyro_gap=MAX_GYRO_GAP_SEC):
        """Initializes the estimator; defaults are the module constants.

        Args:
            still_angle_epsilon: Encoder angle change, rad, still counted as not moving.
            still_hold: Seconds the encoders must be still before samples count.
            startup_sec: Seconds of still data averaged before calibration.
            time_constant: Seconds, of the moving average after calibration.
            encoder_timeout: Oldest encoder sample, s, that still vouches.
            max_startup_rate: Largest |rate| accepted during calibration, rad/s.
            max_still_rate: Largest |rate - bias| accepted afterwards, rad/s.
            max_gyro_gap: Longest gyro gap, s, counted as a sample interval.

        Raises:
            ValueError: If `time_constant` or `startup_sec` is not positive.
        """
        if time_constant <= 0.0:
            raise ValueError(
                'time_constant must be > 0, got {}'.format(time_constant))
        if startup_sec <= 0.0:
            raise ValueError(
                'startup_sec must be > 0, got {}'.format(startup_sec))
        self.still_angle_epsilon = still_angle_epsilon
        self.still_hold = still_hold
        self.startup_sec = startup_sec
        self.time_constant = time_constant
        self.encoder_timeout = encoder_timeout
        self.max_startup_rate = max_startup_rate
        self.max_still_rate = max_still_rate
        self.max_gyro_gap = max_gyro_gap

        self._bias = 0.0
        self._calibrated = False
        # Startup averaging: sum of rate * dt and of dt over still samples.
        self._startup_weighted_sum = 0.0
        self._startup_time = 0.0
        # (stamp, left, right) where the encoders last moved, or None.
        self._still_anchor = None
        # Stamp of the newest encoder sample, or None.
        self._last_encoder_stamp = None
        # Stamp of the previous gyro sample, or None.
        self._last_gyro_stamp = None

    @property
    def bias(self):
        """Returns the current bias estimate in rad/s; 0.0 until data arrives."""
        return self._bias

    @property
    def calibrated(self):
        """Returns True once `startup_sec` of still data has been averaged."""
        return self._calibrated

    def add_encoders(self, stamp_sec, left_angle, right_angle):
        """Adds one encoder sample.

        Args:
            stamp_sec: Sample header stamp, seconds.
            left_angle: Accumulated left-side angle, radians.
            right_angle: Accumulated right-side angle, radians.
        """
        if (self._last_encoder_stamp is not None
                and stamp_sec <= self._last_encoder_stamp):
            return
        self._last_encoder_stamp = stamp_sec

        anchor = self._still_anchor
        if (anchor is None
                or abs(left_angle - anchor[1]) > self.still_angle_epsilon
                or abs(right_angle - anchor[2]) > self.still_angle_epsilon):
            self._still_anchor = (stamp_sec, left_angle, right_angle)

    def is_still(self, stamp_sec):
        """Returns True if the encoders vouch for standing still at `stamp_sec`."""
        if self._still_anchor is None or self._last_encoder_stamp is None:
            return False
        if stamp_sec - self._last_encoder_stamp > self.encoder_timeout:
            return False
        return self._last_encoder_stamp - self._still_anchor[0] >= self.still_hold

    def correct(self, stamp_sec, rate):
        """Adds one gyro yaw-rate sample and corrects it.

        Args:
            stamp_sec: Sample header stamp, seconds.
            rate: Measured yaw rate, rad/s.

        Returns:
            A `Correction`. The bias is updated first, so a still sample is
            corrected with the estimate it just contributed to.
        """
        previous = self._last_gyro_stamp
        if previous is not None and stamp_sec <= previous:
            # Out of order: correct it, but learn nothing from it.
            return Correction(rate - self._bias, self._bias, False)
        self._last_gyro_stamp = stamp_sec
        dt = 0.0 if previous is None else min(stamp_sec - previous,
                                              self.max_gyro_gap)

        used = dt > 0.0 and self.is_still(stamp_sec) and self._plausible(rate)
        if used:
            if self._calibrated:
                alpha = min(dt / self.time_constant, 1.0)
                self._bias += alpha * (rate - self._bias)
            else:
                self._startup_weighted_sum += rate * dt
                self._startup_time += dt
                self._bias = self._startup_weighted_sum / self._startup_time
                if self._startup_time >= self.startup_sec:
                    self._calibrated = True
        return Correction(rate - self._bias, self._bias, used)

    def _plausible(self, rate):
        """Returns whether a standing-still reading could be bias."""
        if self._calibrated:
            return abs(rate - self._bias) <= self.max_still_rate
        return abs(rate) <= self.max_startup_rate


__all__ = [
    'BIAS_TIME_CONSTANT_SEC', 'Correction', 'ENCODER_TIMEOUT_SEC', 'GyroBias',
    'MAX_GYRO_GAP_SEC', 'MAX_STARTUP_RATE_RADPS', 'MAX_STILL_RATE_RADPS',
    'STARTUP_CALIBRATION_SEC', 'STILL_ANGLE_EPSILON_RAD', 'STILL_HOLD_SEC',
]

"""Pure gyro-bias estimation behind `gyro_bias_node.py`.

NOTHING HERE IMPORTS rclpy OR ANY ROS MESSAGE, on purpose - the same split as
`wheel_odometry.py` versus `wheel_odometry_node.py`, and for the same reason:
the logic is unit-testable without a ROS graph or a robot (see
`localization/test/test_gyro_bias.py`).

WHY THIS EXISTS. `ekf.yaml` takes the robot's turn rate from the chassis gyro
(ICM-20948) and integrates it into the heading. A gyro reports a small
constant offset (bias) on top of the true rate - the ICM-20948 datasheet
allows up to +/-5 deg/s straight out of the box, and it wanders with
temperature. Integrated, a bias of only 0.1 deg/s is 6 degrees of heading per
minute, even with the robot parked. robot_localization does not estimate gyro
bias itself, so it has to be removed before the EKF sees the data.

HOW. The wheels tell us when the robot is standing still, and a gyro on a
robot that is standing still should read zero - so whatever it reads then IS
the bias. Two phases:

  - STARTUP CALIBRATION: the first `startup_sec` seconds of standing still are
    averaged with equal weight. That converges fast and gives a usable bias
    before anyone drives.
  - ZERO-VELOCITY UPDATES: after that, every standing-still sample nudges the
    bias with an exponential moving average of time constant `time_constant`,
    so it follows slow temperature drift without reacting to noise.

While the robot moves, the bias is held and only subtracted.

WHAT COUNTS AS STANDING STILL. Neither side's accumulated encoder angle has
moved more than `still_angle_epsilon` from where it was at least `still_hold`
seconds ago. Measured against an anchor, not sample to sample, so very slow
motion still accumulates past the threshold. On hardware one encoder count is
a whole centimetre (see `wheel_odometry.py`'s ENCODER UNIT CALIBRATION note),
far above the threshold, so any count change is motion.

GUARDS. A standing-still sample is still refused when:

  - the encoder data is older than `encoder_timeout` at the gyro's stamp - no
    fresh encoders means we do not actually know the robot is still;
  - the reading is implausible for a parked robot (`max_startup_rate` before
    calibration, `max_still_rate` away from the current bias after it) -
    someone bumping or lifting the robot is not bias.

ONLY THE z AXIS. `ekf.yaml` runs in `two_d_mode` and fuses only yaw rate, so
roll and pitch rate biases would change nothing downstream.
"""

from dataclasses import dataclass

#: Largest encoder-angle change, in radians, still treated as not moving.
#: On hardware one count is 0.01 m / 0.040 m = 0.25 rad, so this can never
#: hide a real count.
STILL_ANGLE_EPSILON_RAD = 1e-3

#: How long the encoders must have been still, in seconds, before gyro
#: samples count as bias. Covers the robot settling after it stops.
STILL_HOLD_SEC = 0.5

#: Seconds of standing-still data averaged before the bias is "calibrated".
STARTUP_CALIBRATION_SEC = 2.0

#: Time constant, in seconds, of the moving average used after calibration.
#: Long compared with noise (2.62e-3 rad/s per sample), short compared with
#: thermal drift (minutes).
BIAS_TIME_CONSTANT_SEC = 30.0

#: Oldest encoder sample, in seconds before the gyro stamp, that still vouches
#: for the robot standing still. Encoders run near 80 Hz on hardware.
ENCODER_TIMEOUT_SEC = 0.2

#: During startup calibration: largest |rate| accepted as bias, rad/s. Above
#: the ICM-20948's worst-case +/-5 deg/s (0.087 rad/s) zero-rate offset.
MAX_STARTUP_RATE_RADPS = 0.2

#: After calibration: largest |rate - bias| accepted as bias, rad/s. About 8
#: sigma of the gyro's per-sample noise.
MAX_STILL_RATE_RADPS = 0.02

#: Longest gap between gyro samples that is counted towards calibration time
#: or used as an averaging step, in seconds. Keeps a stalled stream from
#: looking like seconds of calibration data.
MAX_GYRO_GAP_SEC = 0.1


@dataclass(frozen=True)
class Correction:
    """What one gyro sample did.

    Attributes:
        rate: the bias-corrected yaw rate, rad/s.
        bias: the bias that was subtracted, rad/s.
        used_for_bias: True if this sample updated the bias.
    """

    rate: float
    bias: float
    used_for_bias: bool


class GyroBias:
    """Estimates and removes the gyro's yaw-rate bias. See the module docstring.

    Stateful but ROS-free: the node hands it encoder samples and gyro samples,
    each with its own header stamp in seconds.
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
        #: Startup averaging: sum of rate * dt and of dt over still samples.
        self._startup_weighted_sum = 0.0
        self._startup_time = 0.0
        #: `(stamp, left, right)` where the encoders last moved, or None.
        self._still_anchor = None
        #: Stamp of the newest encoder sample, or None.
        self._last_encoder_stamp = None
        #: Stamp of the previous gyro sample, or None.
        self._last_gyro_stamp = None

    @property
    def bias(self):
        """Current bias estimate, rad/s. 0.0 until data arrives."""
        return self._bias

    @property
    def calibrated(self):
        """True once `startup_sec` of standing-still data has been averaged."""
        return self._calibrated

    def add_encoders(self, stamp_sec, left_angle, right_angle):
        """Fold in one encoder sample: accumulated per-side angles, radians."""
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
        """True if the encoders vouch for the robot standing still at `stamp_sec`."""
        if self._still_anchor is None or self._last_encoder_stamp is None:
            return False
        if stamp_sec - self._last_encoder_stamp > self.encoder_timeout:
            return False
        return self._last_encoder_stamp - self._still_anchor[0] >= self.still_hold

    def correct(self, stamp_sec, rate):
        """Fold in one gyro yaw-rate sample and return it bias-corrected.

        Args:
            stamp_sec: the sample's header stamp, seconds.
            rate: measured yaw rate, rad/s.

        Returns:
            A `Correction`. The bias is updated first, so a standing-still
            sample is corrected with the estimate it just contributed to.
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
        """Whether a standing-still reading could really be bias."""
        if self._calibrated:
            return abs(rate - self._bias) <= self.max_still_rate
        return abs(rate) <= self.max_startup_rate


__all__ = [
    'BIAS_TIME_CONSTANT_SEC', 'Correction', 'ENCODER_TIMEOUT_SEC', 'GyroBias',
    'MAX_GYRO_GAP_SEC', 'MAX_STARTUP_RATE_RADPS', 'MAX_STILL_RATE_RADPS',
    'STARTUP_CALIBRATION_SEC', 'STILL_ANGLE_EPSILON_RAD', 'STILL_HOLD_SEC',
]

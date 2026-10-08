#!/usr/bin/env python3
"""Publishes `/imu/corrected`: `/imu` with the gyro's yaw-rate bias removed.

    ros2 run localization gyro_bias_node.py

The EKF (`ekf.yaml`, `imu0: imu/corrected`) takes the turn rate from this
topic. All logic is in `localization.gyro_bias` (ROS-free, unit-tested); this
node subscribes, calls in there and republishes.

Keep the robot still for the first few seconds. Startup calibration needs
`startup_calibration` seconds of standing still plus `still_hold` to settle,
and the node logs "gyro bias calibrated" when done. Driving earlier is not
fatal (calibration finishes at the next stop) but heading drifts faster until
then.

Only `angular_velocity.z` changes. Everything else, including the header stamp
and the REP-145 `-1` orientation covariance, passes through.

QoS: both inputs are best-effort on hardware (`qos_profile_sensor_data`), and
a reliable subscription to a best-effort publisher receives nothing, so the
subscriptions are best-effort. The output is reliable, which serves both
reliable and best-effort subscribers.

`use_sim_time` is not declared here: rclpy declares it and the launch file
sets it (see `wheel_odometry_node.py`).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState

from localization.gyro_bias import (BIAS_TIME_CONSTANT_SEC,
                                    ENCODER_TIMEOUT_SEC,
                                    MAX_STARTUP_RATE_RADPS,
                                    MAX_STILL_RATE_RADPS,
                                    STARTUP_CALIBRATION_SEC,
                                    STILL_ANGLE_EPSILON_RAD, STILL_HOLD_SEC,
                                    GyroBias)

# Throttle for repeated warnings, in seconds (both inputs run near 80 Hz).
WARN_THROTTLE_SEC = 5.0


class GyroBiasNode(Node):
    """Thin ROS shell around `localization.gyro_bias.GyroBias`.

    Parameters:
        left_joint, right_joint: Joint names in `/wheel_encoders`.
        still_angle_epsilon, still_hold, startup_calibration, time_constant,
        encoder_timeout, max_startup_rate, max_still_rate: See
            `localization.gyro_bias.GyroBias`.
    """

    def __init__(self, **kwargs):
        """Initializes the node; `kwargs` go to `Node` (e.g. parameter_overrides)."""
        super().__init__('gyro_bias_node', **kwargs)

        self.declare_parameter('left_joint', 'left_up_wheel_link_joint')
        self.declare_parameter('right_joint', 'right_up_wheel_link_joint')
        self.declare_parameter('still_angle_epsilon', STILL_ANGLE_EPSILON_RAD)
        self.declare_parameter('still_hold', STILL_HOLD_SEC)
        self.declare_parameter('startup_calibration', STARTUP_CALIBRATION_SEC)
        self.declare_parameter('time_constant', BIAS_TIME_CONSTANT_SEC)
        self.declare_parameter('encoder_timeout', ENCODER_TIMEOUT_SEC)
        self.declare_parameter('max_startup_rate', MAX_STARTUP_RATE_RADPS)
        self.declare_parameter('max_still_rate', MAX_STILL_RATE_RADPS)
        p = self.get_parameter

        self._left_joint = p('left_joint').value
        self._right_joint = p('right_joint').value
        self._estimator = GyroBias(
            still_angle_epsilon=p('still_angle_epsilon').value,
            still_hold=p('still_hold').value,
            startup_sec=p('startup_calibration').value,
            time_constant=p('time_constant').value,
            encoder_timeout=p('encoder_timeout').value,
            max_startup_rate=p('max_startup_rate').value,
            max_still_rate=p('max_still_rate').value,
        )
        self._announced_calibration = False
        # Stamp of the first IMU message; keeps the warning below quiet while
        # the robot is expected to still be settling.
        self._first_imu_stamp = None

        self._imu_pub = self.create_publisher(Imu, 'imu/corrected', 10)
        self._imu_sub = self.create_subscription(
            Imu, 'imu', self._on_imu, qos_profile_sensor_data)
        self._encoder_sub = self.create_subscription(
            JointState, 'wheel_encoders', self._on_encoders,
            qos_profile_sensor_data)

        self.get_logger().info(
            'gyro_bias_node up: /imu -> /imu/corrected. Keep the robot still '
            'for ~{:.1f}s until "gyro bias calibrated".'.format(
                self._estimator.still_hold + self._estimator.startup_sec))

    def _on_encoders(self, msg):
        """Feeds the estimator one encoder sample."""
        names = list(msg.name)
        if self._left_joint in names and self._right_joint in names:
            left = names.index(self._left_joint)
            right = names.index(self._right_joint)
        else:
            # Documented wire order, as in wheel_odometry_node.py.
            left, right = 0, 1
        if max(left, right) >= len(msg.position):
            self.get_logger().warning(
                '/wheel_encoders carried {} position(s); need left and right'
                .format(len(msg.position)),
                throttle_duration_sec=WARN_THROTTLE_SEC)
            return
        self._estimator.add_encoders(
            Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
            msg.position[left], msg.position[right])

    def _on_imu(self, msg):
        """Republishes an IMU message with the bias removed from yaw rate."""
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self._first_imu_stamp is None:
            self._first_imu_stamp = stamp
        correction = self._estimator.correct(stamp, msg.angular_velocity.z)
        msg.angular_velocity.z = correction.rate
        self._imu_pub.publish(msg)

        if self._estimator.calibrated and not self._announced_calibration:
            self._announced_calibration = True
            self.get_logger().info(
                'gyro bias calibrated: {:+.5f} rad/s ({:+.3f} deg/s)'.format(
                    correction.bias, correction.bias * 57.29577951308232))
        elif (not self._estimator.calibrated
                and stamp - self._first_imu_stamp > 2.0 * self._estimator.still_hold
                and not self._estimator.is_still(stamp)):
            self.get_logger().warning(
                'gyro bias not calibrated yet and the robot is not standing '
                'still (or /wheel_encoders is missing). Stop for a few '
                'seconds so the heading does not drift.',
                throttle_duration_sec=WARN_THROTTLE_SEC)


def main(args=None):
    """Runs the node until interrupted."""
    rclpy.init(args=args)
    node = None
    try:
        node = GyroBiasNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['GyroBiasNode', 'main']


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Publishes `/odom` and the `odom -> base_footprint` TF from `/wheel_encoders`.

    ros2 run localization wheel_odometry_node.py

This is the robot's odometry source. slam_toolbox looks up `odom ->
base_footprint` for every scan (`mapper_params_hardware.yaml`, `odom_frame:
odom`) and nothing else published it. The EKF (`ekf.yaml`) fuses this node's
`/odom` with the gyro and owns the TF edge, so `hardware_odometry.launch.py`
runs this node with `publish_tf: false`.

slam_toolbox reads the TF, not `/odom`, so the TF is the half that matters
for mapping. `/odom` is for robot_localization and RViz. Both come from the
same pose and stamp. Two unsynchronised publishers of one TF edge caused a
TF_OLD_DATA storm before (see `ekf.yaml`).

All arithmetic is in `localization.wheel_odometry` (ROS-free, unit-tested).
Read its docstring before changing anything here: the input contract, the
position-delta choice, the scrub-corrected `wheel_separation` and its
calibration, and the unverified forward-positive sign assumption are there.

Timestamps: each pose carries the encoder sample's header stamp, which the
vendor driver back-dates by the serial transmission time (~12 ms at 115200
baud). The TF therefore trails real time slightly, and a scan stamped later
makes tf2 wait rather than extrapolate; `transform_timeout: 1.0` in
`mapper_params_hardware.yaml` covers that. Stamping with `now()` would date the
pose differently from its data.

`use_sim_time` is not declared here: rclpy declares it in `Node.__init__`, and
declaring it again raises `ParameterAlreadyDeclaredException`. The launch file
sets it to false.
"""

import numpy
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState

from localization.wheel_odometry import (ENCODER_SCALE_FACTOR,
                                         MAX_SAMPLE_GAP_SEC,
                                         MAX_WHEEL_SPEED_MPS, UPDATE_BACKWARD_TIME,
                                         UPDATE_GAP, UPDATE_IMPLAUSIBLE,
                                         UPDATE_OK, WHEEL_RADIUS_M,
                                         WHEEL_SEPARATION_M, WheelOdometry,
                                         yaw_to_quaternion)

# Seconds between repeated warnings; the encoder stream runs near 80 Hz.
WARN_THROTTLE_SEC = 5.0

# Operator messages for `update()` outcomes. UPDATE_FIRST_SAMPLE is absent: it
# happens once and is normal.
_STATUS_WARNINGS = {
    UPDATE_BACKWARD_TIME: (
        'encoder sample stamped at or before the previous one; skipping it. '
        'A burst of these means /wheel_encoders stamps are not monotonic.'),
    UPDATE_GAP: (
        'gap of more than {gap:.2f}s between encoder samples; that motion is '
        'NOT in the odometry. One of these after a serial reconnect is '
        'expected; repeated ones mean the link is dropping out and the map '
        'will drift.'),
    UPDATE_IMPLAUSIBLE: (
        'encoder sample implies more than {speed:.2f} m/s at a wheel, which '
        'this chassis cannot do; skipping it. Most likely the MCU\'s distance '
        'accumulator reset.'),
}


class WheelOdometryNode(Node):
    """Thin ROS shell around `localization.wheel_odometry.WheelOdometry`.

    Parameters:
        wheel_radius: Wheel radius in m.
        wheel_separation: Effective, scrub-corrected separation in m.
        left_joint, right_joint: Joint names in `/wheel_encoders`.
        odom_frame, base_frame: Frames of the published pose and TF.
        publish_tf: Whether to broadcast the TF (off when the EKF owns it).
        max_sample_gap, max_wheel_speed, encoder_scale: See
            `localization.wheel_odometry.WheelOdometry`.
        position_stddev, yaw_stddev, linear_velocity_stddev, yaw_rate_stddev:
            Placeholder standard deviations for the published covariances.
    """

    def __init__(self, **kwargs):
        """Initializes the node; `kwargs` go to `Node` (e.g. parameter_overrides)."""
        super().__init__('wheel_odometry_node', **kwargs)

        self._declare_parameters()
        p = self.get_parameter

        self._odom_frame = p('odom_frame').value
        self._base_frame = p('base_frame').value
        self._left_joint = p('left_joint').value
        self._right_joint = p('right_joint').value
        self._publish_tf = p('publish_tf').value

        self._odometry = WheelOdometry(
            wheel_radius=p('wheel_radius').value,
            wheel_separation=p('wheel_separation').value,
            max_sample_gap_sec=p('max_sample_gap').value,
            max_wheel_speed=p('max_wheel_speed').value,
            encoder_scale=p('encoder_scale').value,
        )

        self._pose_covariance = self._planar_covariance(
            p('position_stddev').value, p('yaw_stddev').value)
        self._twist_covariance = self._planar_covariance(
            p('linear_velocity_stddev').value, p('yaw_rate_stddev').value)

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Best-effort like the publisher; a reliable subscription would
        # silently never receive anything.
        self._encoder_sub = self.create_subscription(
            JointState, 'wheel_encoders', self._on_encoders,
            qos_profile_sensor_data)

        self.get_logger().info(
            'wheel_odometry_node up: {} -> {}, wheel_radius={} '
            'wheel_separation={} (effective, scrub-corrected), '
            'encoder_scale={} (confirmed centimetre correction - see '
            'localization.wheel_odometry\'s docstring), joints=({}, {})'
            .format(self._odom_frame, self._base_frame,
                    self._odometry.wheel_radius,
                    self._odometry.wheel_separation,
                    self._odometry.encoder_scale,
                    self._left_joint, self._right_joint))

    def _declare_parameters(self):
        """Declares every tunable parameter."""
        self.declare_parameter('wheel_radius', WHEEL_RADIUS_M)
        # Effective, scrub-corrected; a parameter so it can be re-tuned without a rebuild.
        self.declare_parameter('wheel_separation', WHEEL_SEPARATION_M)

        # As in model.sdf and the vendor driver. Looked up by name so a
        # reordered JointState cannot swap the sides.
        self.declare_parameter('left_joint', 'left_up_wheel_link_joint')
        self.declare_parameter('right_joint', 'right_up_wheel_link_joint')

        # Match mapper_params_hardware.yaml's odom_frame/base_frame.
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Off when the EKF owns the TF edge; /odom is published either way.
        self.declare_parameter('publish_tf', True)

        self.declare_parameter('max_sample_gap', MAX_SAMPLE_GAP_SEC)
        self.declare_parameter('max_wheel_speed', MAX_WHEEL_SPEED_MPS)

        # Centimetre correction. Defaults on because without it the real
        # robot's pose is 100x too large; set 1.0 to disable.
        self.declare_parameter('encoder_scale', ENCODER_SCALE_FACTOR)

        # Placeholder covariances, not measurements: wheel slip is not
        # characterised yet. All zeros would make robot_localization treat the
        # pose as exact.
        self.declare_parameter('position_stddev', 0.05)
        self.declare_parameter('yaw_stddev', 0.05)
        self.declare_parameter('linear_velocity_stddev', 0.05)
        self.declare_parameter('yaw_rate_stddev', 0.05)

    @staticmethod
    def _planar_covariance(linear_stddev, angular_stddev):
        """Returns a constant 6x6 row-major covariance for a planar estimate.

        A `numpy.float64` array avoids a per-element type check on each of the
        ~80 messages a second, and sharing it is safe because it never changes.
        The z, roll and pitch rows stay zero: `two_d_mode` in `ekf.yaml` means
        nothing here estimates them.

        Args:
            linear_stddev: Standard deviation for x and y.
            angular_stddev: Standard deviation for yaw.
        """
        covariance = numpy.zeros(36, dtype=numpy.float64)
        covariance[0] = covariance[7] = linear_stddev ** 2   # x, y
        covariance[35] = angular_stddev ** 2                 # yaw
        return covariance

    def _side_indices(self, msg):
        """Locates the left and right entries of a `/wheel_encoders` message.

        Matches by joint name. Otherwise falls back, with a warning, to the
        documented wire order (indices 0 and 1).

        Args:
            msg: The `JointState` message.

        Returns:
            `(left_index, right_index)`, or None if the message cannot carry
            two sides.
        """
        positions = len(msg.position)
        if positions < 2:
            self.get_logger().warning(
                '/wheel_encoders carried {} position(s); need at least 2 '
                '(left, right)'.format(positions),
                throttle_duration_sec=WARN_THROTTLE_SEC)
            return None

        names = list(msg.name)
        if self._left_joint in names and self._right_joint in names:
            left = names.index(self._left_joint)
            right = names.index(self._right_joint)
            if max(left, right) < positions:
                return (left, right)

        self.get_logger().warning(
            '/wheel_encoders does not name both {!r} and {!r}; falling back to '
            'position[0] as left and position[1] as right. Verify that is '
            'really the order being published.'.format(
                self._left_joint, self._right_joint),
            throttle_duration_sec=WARN_THROTTLE_SEC)
        return (0, 1)

    def _on_encoders(self, msg):
        """Integrates one sample and publishes the result."""
        indices = self._side_indices(msg)
        if indices is None:
            return
        left_index, right_index = indices

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            # A zero stamp makes every dt zero, which would look like a pose
            # that never moves.
            self.get_logger().warning(
                '/wheel_encoders header stamp is zero; nothing can be '
                'integrated. Is a publisher stamping with sim time and '
                'no /clock?', throttle_duration_sec=WARN_THROTTLE_SEC)
            return

        update = self._odometry.update(
            stamp.nanoseconds * 1e-9,
            msg.position[left_index],
            msg.position[right_index])

        warning = _STATUS_WARNINGS.get(update.status)
        if warning is not None:
            self.get_logger().warning(
                warning.format(gap=self._odometry.max_sample_gap_sec,
                               speed=self._odometry.max_wheel_speed),
                throttle_duration_sec=WARN_THROTTLE_SEC)

        # Published even when the sample was not integrated: a live edge at a
        # held pose lets slam_toolbox scan-match instead of failing the lookup.
        self._publish(msg.header.stamp, update)

    def _publish(self, stamp, update):
        """Publishes `update`'s pose as an Odometry message and a TF."""
        pose = update.pose
        _, _, qz, qw = yaw_to_quaternion(pose.theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self._pose_covariance
        odom.twist.twist.linear.x = update.linear_velocity
        odom.twist.twist.angular.z = update.angular_velocity
        odom.twist.covariance = self._twist_covariance
        self._odom_pub.publish(odom)

        if not self._publish_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Runs the node until interrupted."""
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node is None if construction failed (e.g. wheel_separation of 0).
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['WheelOdometryNode', 'main']


if __name__ == '__main__':
    main()

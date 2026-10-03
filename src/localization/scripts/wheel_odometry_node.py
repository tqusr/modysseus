#!/usr/bin/env python3
"""Publish `/odom` and the `odom -> base_footprint` TF from `/wheel_encoders`.

    ros2 run localization wheel_odometry_node.py

This is the hardware robot's odometry source. Before it existed, nothing on
hardware published a dynamic `odom -> base_footprint` transform at all:
`mapper_params_online_async.yaml` sets `odom_frame: odom`, so slam_toolbox
looks that transform up for every single scan, and `ekf.yaml` sets
`publish_tf: false` because in simulation Gazebo's DiffDrive plugin already
owns that edge. This node takes over the plugin's exact role on hardware - the
same `/odom` topic, the same `odom`/`base_footprint` frames, the same TF edge -
so both of those configs stay correct unchanged and nothing downstream has to
know which mode it is running in.

WHAT CONSUMES WHAT. slam_toolbox does NOT subscribe to `/odom`; it reads the
TF. So the `TransformBroadcaster` below is the load-bearing half for mapping,
and the `nav_msgs/Odometry` message is for robot_localization (once the EKF
phase reaches hardware) and for RViz. Both are published from the same pose
and with the same stamp, so they can never disagree.

DO NOT RUN THIS IN SIMULATION. Gazebo's DiffDrive plugin publishes `/odom` and
broadcasts the same TF edge there, and two unsynchronised publishers of one TF
edge is a failure this project has already paid for once - see `ekf.yaml`'s
comment on the TF_OLD_DATA storm that caused. Simulation does publish
`/wheel_encoders` with the same two joint names, so this node would happily
run; the reason not to is the collision, not an incompatibility.

ALL the arithmetic lives in `localization.wheel_odometry`, which imports no
ROS and is unit-tested without a ROS graph. Read that module's docstring
before touching anything here: it is where the input contract, the
position-deltas-not-velocity decision, the scrub-corrected `wheel_separation`
and its simulation-only provenance, and the still-unverified forward-positive
sign assumption are all written down.

TIMESTAMPS. Every pose is stamped with the encoder sample's own header stamp,
which the vendor driver back-dates by the line's serial transmission time
(~12 ms at 115200 baud). The TF therefore trails real time slightly, so a
scan stamped later than the newest encoder sample needs tf2 to wait rather
than extrapolate; `mapper_params_online_async.yaml`'s `transform_timeout: 1.0`
covers that comfortably. The alternative - stamping with `now()` - would date
the pose differently from the data it was computed from, which is worse.

use_sim_time IS NOT DECLARED HERE, deliberately. rclpy's own `TimeSource`
declares it during `Node.__init__`, so declaring it again raises
`ParameterAlreadyDeclaredException` and the node would not start at all. It is
set where it belongs, by the launch file, to false.
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

#: How often repeated complaints about the same condition are allowed through,
#: in seconds. The encoder stream runs near 80 Hz, so an unthrottled warning
#: would bury every other line in the terminal.
WARN_THROTTLE_SEC = 5.0

#: Messages for the `update()` outcomes worth telling the operator about.
#: `UPDATE_FIRST_SAMPLE` is absent: it happens exactly once and is normal.
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
    """Thin ROS shell around `localization.wheel_odometry.WheelOdometry`."""

    def __init__(self, **kwargs):
        # kwargs reach rclpy.node.Node, so a test can pass parameter_overrides.
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
        # Sensor-data QoS to match the publisher: ugv02_serial_node publishes
        # /wheel_encoders with qos_profile_sensor_data (best effort), and a
        # reliable subscription cannot connect to a best-effort publisher at
        # all. It would not error - it would simply never receive a message,
        # which is the same silent nothing this project hit with
        # hardware.launch.py's joint_state_relay, from the opposite direction.
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
        """Declare every tunable. Nothing in this node is hardcoded."""
        self.declare_parameter('wheel_radius', WHEEL_RADIUS_M)
        # The EFFECTIVE, scrub-corrected separation, and a parameter rather
        # than a constant precisely because the shipped default was measured
        # in simulation and the real robot's scrub may differ. See
        # localization.wheel_odometry's docstring.
        self.declare_parameter('wheel_separation', WHEEL_SEPARATION_M)

        # Spelled as model.sdf and the vendor driver's encoder_joint_names
        # spell them, left first. Looked up by name so a reordered or extended
        # JointState cannot silently swap the robot's sides.
        self.declare_parameter('left_joint', 'left_up_wheel_link_joint')
        self.declare_parameter('right_joint', 'right_up_wheel_link_joint')

        # Match the simulated DiffDrive plugin's frame_id/child_frame_id, and
        # mapper_params_online_async.yaml's odom_frame/base_frame.
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Only reason to turn this off: something else already owns the edge -
        # which in practice means simulation, where this node should not be
        # running at all. The /odom topic is published either way.
        self.declare_parameter('publish_tf', True)

        self.declare_parameter('max_sample_gap', MAX_SAMPLE_GAP_SEC)
        self.declare_parameter('max_wheel_speed', MAX_WHEEL_SPEED_MPS)

        # Confirmed centimetre-unit correction. Defaults to the correction,
        # not to 1.0, because this node is the real-hardware odometry source
        # and without it the pose it publishes is 100x too large - see
        # localization.wheel_odometry's module docstring ("ENCODER UNIT
        # CALIBRATION") and ENCODER_SCALE_FACTOR. WheelOdometry itself still
        # defaults to 1.0 for any other caller (e.g. tests) that does not
        # expect a hidden scale. Exposed as a parameter, not baked in, so it
        # can still be overridden - e.g. down to 1.0 to disable it.
        self.declare_parameter('encoder_scale', ENCODER_SCALE_FACTOR)

        # Covariances. These are PLACEHOLDERS, not measurements: nothing has
        # characterised this chassis's wheel slip yet. They are here because a
        # published all-zero covariance is worse than an honest guess -
        # robot_localization takes it literally and treats the pose as exact.
        # Replace them with measured values when the EKF phase reaches
        # hardware; until then only RViz reads them.
        self.declare_parameter('position_stddev', 0.05)
        self.declare_parameter('yaw_stddev', 0.05)
        self.declare_parameter('linear_velocity_stddev', 0.05)
        self.declare_parameter('yaw_rate_stddev', 0.05)

    @staticmethod
    def _planar_covariance(linear_stddev, angular_stddev):
        """One constant 6x6 row-major covariance for a planar estimate.

        Built once as a `numpy.float64` array rather than a Python list: that
        is the generated setter's fast path, and assigning a 36-element list
        costs an element-by-element type check at every one of ~80 messages a
        second. Sharing one array across messages is safe because it never
        changes; the messages themselves are built fresh each cycle.

        The out-of-plane rows (z, roll, pitch) are left at zero rather than
        given a large variance: `two_d_mode` in `ekf.yaml` means no consumer
        here estimates them, and inventing a number for a quantity this node
        cannot observe would be the same mistake as inventing a small one.
        """
        covariance = numpy.zeros(36, dtype=numpy.float64)
        covariance[0] = covariance[7] = linear_stddev ** 2   # x, y
        covariance[35] = angular_stddev ** 2                 # yaw
        return covariance

    def _side_indices(self, msg):
        """Locate the left and right entries of a `/wheel_encoders` message.

        By joint name when both configured names are present, which is the
        case for both the vendor driver and simulation. Falls back to indices
        0 and 1 - the documented wire order - with a warning, so an unexpected
        naming still produces odometry rather than silence, but never silently.

        Returns:
            `(left_index, right_index)`, or None if the message cannot carry
            two sides at all.
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
        """Integrate one sample and publish the result."""
        indices = self._side_indices(msg)
        if indices is None:
            return
        left_index, right_index = indices

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            # A zero stamp is what a publisher on sim time with no /clock
            # produces, and it would make every dt zero. Say so, because the
            # symptom otherwise is just a pose that never moves.
            self.get_logger().warning(
                '/wheel_encoders header stamp is zero; nothing can be '
                'integrated. Is a publisher running with use_sim_time true '
                'and no /clock?', throttle_duration_sec=WARN_THROTTLE_SEC)
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

        # Published for every sample, including the ones that could not be
        # integrated: holding the pose still keeps the TF edge alive, and a
        # live edge at a stale pose is what lets slam_toolbox fall back on
        # scan matching instead of failing the lookup outright.
        self._publish(msg.header.stamp, update)

    def _publish(self, stamp, update):
        """Publish `update`'s pose as both an Odometry message and a TF."""
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
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node stays None if construction itself failed - e.g. a
        # wheel_separation of 0 - and rclpy still has to be shut down.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


__all__ = ['WheelOdometryNode', 'main']


if __name__ == '__main__':
    main()

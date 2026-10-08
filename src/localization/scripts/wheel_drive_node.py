#!/usr/bin/env python3
"""Drives the UGV02 by wheel side speeds, `{"T":1,"L":..,"R":..}`.

    ros2 run localization wheel_drive_node.py

Subscribes to a Twist (`cmd_vel_ctrl` by default), converts it with
`localization.wheel_drive.twist_to_wheels` using the effective wheel
separation from `config/hardware.yaml`, and writes it to the chassis MCU. See
that module for why.

This is a second writer on the serial port next to the vendor's
`ugv02_serial_node` (started by systemd), which keeps reading the sensors;
this node only writes. The vendor node sends anything on `/cmd_vel` as `T:13`,
which would alternate with this node's `T:1`, so nothing here publishes
`/cmd_vel` (Nav2 uses `cmd_vel_ctrl`). Other `/cmd_vel` publishers, e.g.
teleop, still reach the MCU directly.

Safety: the MCU holds the last command forever, so this node re-sends it at
`send_rate` Hz, sends zero if no Twist arrives for `cmd_timeout` seconds and
sends zero on shutdown. Commands with non-finite wheel speeds are dropped.
"""

import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node

from localization.wheel_drive import encode_wheels, twist_to_wheels


class WheelDriveNode(Node):
    """Writes wheel side speeds to the MCU from a Twist topic.

    Parameters:
        port: Serial device.
        baud: Baud rate.
        wheel_separation: Effective separation in m, as in wheel_odometry_node.
        max_wheel_speed: Per-side limit in m/s; 0 disables it.
        max_angular_velocity: Turn-rate limit in rad/s; 0 disables it.
        cmd_timeout: Seconds without a Twist before sending zero.
        send_rate: Re-send rate in Hz.
        input_topic: Twist topic to follow.
    """

    def __init__(self):
        super().__init__('wheel_drive_node')
        self.declare_parameter('port', '/dev/ugv02')
        self.declare_parameter('baud', 115200)
        # Effective, not geometric (see wheel_drive.py); same file as wheel_odometry_node.
        self.declare_parameter('wheel_separation', 0.4084)
        # 0 disables the limit; the MCU's real ceiling is unknown.
        self.declare_parameter('max_wheel_speed', 0.0)
        # 0 disables the limit; caps the turn rate only.
        self.declare_parameter('max_angular_velocity', 0.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('send_rate', 20.0)
        self.declare_parameter('input_topic', 'cmd_vel_ctrl')

        p = self.get_parameter
        self._separation = p('wheel_separation').value
        self._max_wheel_speed = p('max_wheel_speed').value
        self._max_angular = p('max_angular_velocity').value
        self._timeout = p('cmd_timeout').value
        self._wheels = (0.0, 0.0)
        self._last_cmd = 0.0  # Monotonic; 0 = never, so we start at zero.

        self._port = serial.Serial(p('port').value, p('baud').value,
                                   write_timeout=0.1)
        self.create_subscription(Twist, p('input_topic').value, self._on_twist, 10)
        self.create_timer(1.0 / p('send_rate').value, self._send)
        self.get_logger().info(
            'wheel_drive_node up: port={} separation={} max_wheel_speed={} '
            'max_angular_velocity={} input={}'.format(
                p('port').value, self._separation, self._max_wheel_speed,
                self._max_angular, p('input_topic').value))

    def _on_twist(self, msg):
        """Stores the wheel speeds for a Twist, ignoring invalid ones."""
        try:
            wheels = twist_to_wheels(msg.linear.x, msg.angular.z,
                                     self._separation, self._max_wheel_speed,
                                     self._max_angular)
            encode_wheels(*wheels)  # Validate now, not at send time.
        except ValueError as exc:
            self.get_logger().warning('ignoring twist: {}'.format(exc))
            return
        self._wheels = wheels
        self._last_cmd = time.monotonic()

    def _send(self):
        """Sends the current wheel speeds, or zero if the command is stale."""
        stale = time.monotonic() - self._last_cmd > self._timeout
        left, right = (0.0, 0.0) if stale else self._wheels
        self._write(encode_wheels(left, right))

    def _write(self, data):
        """Writes `data` to the port, logging (throttled) on failure."""
        try:
            self._port.write(data)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error('write failed: {}'.format(exc),
                                    throttle_duration_sec=2.0)

    def shutdown(self):
        """Sends zero speeds and closes the port, best effort."""
        self._write(encode_wheels(0.0, 0.0))
        try:
            self._port.close()
        except Exception:  # noqa: BLE001 - nothing to do about a dead port
            pass


def main():
    """Runs the node until interrupted."""
    rclpy.init()
    node = WheelDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

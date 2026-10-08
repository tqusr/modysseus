#!/usr/bin/env python3
"""Drive the UGV02 by wheel side speeds, `{"T":1,"L":..,"R":..}`.

    ros2 run localization wheel_drive_node.py

Subscribes to a Twist (`cmd_vel_ctrl` by default), converts it with
`localization.wheel_drive.twist_to_wheels` using the effective wheel
separation from `config/hardware.yaml`, and writes the result to the chassis
MCU. All the maths is in that module; read its docstring for why.

THIS IS A SECOND WRITER ON THE SERIAL PORT, ALONGSIDE THE VENDOR'S
`ugv02_serial_node`, which is started by the systemd service and keeps reading
the sensors. This node only writes. The two must not both be commanded:
anything the vendor node receives on `/cmd_vel` it sends as `T:13`, which
would alternate with this node's `T:1`. So nothing here publishes `/cmd_vel`:
Nav2 and the mission's safe-stop hold use `cmd_vel_ctrl`. Anything else that
publishes `/cmd_vel` (teleop) still reaches the MCU directly.

SAFETY. The MCU holds the last command forever, so this node re-sends it at
`send_rate` Hz and, if no Twist arrives for `cmd_timeout` seconds, sends zero.
It also sends zero on shutdown. A command whose wheel speeds are not finite
is dropped.
"""

import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node

from localization.wheel_drive import encode_wheels, twist_to_wheels


class WheelDriveNode(Node):

    def __init__(self):
        super().__init__('wheel_drive_node')
        self.declare_parameter('port', '/dev/ugv02')
        self.declare_parameter('baud', 115200)
        # Effective, not geometric: see wheel_drive.py. Same value as
        # wheel_odometry_node's, from the same file.
        self.declare_parameter('wheel_separation', 0.4084)
        # 0 disables the limit. The MCU's real ceiling is not known yet.
        self.declare_parameter('max_wheel_speed', 0.0)
        # 0 disables the limit. Caps the turn rate only; see wheel_drive.py.
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
        self._last_cmd = 0.0  # monotonic; 0 means never, so we start at zero

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
        try:
            wheels = twist_to_wheels(msg.linear.x, msg.angular.z,
                                     self._separation, self._max_wheel_speed,
                                     self._max_angular)
            encode_wheels(*wheels)  # validate now, not at send time
        except ValueError as exc:
            self.get_logger().warning('ignoring twist: {}'.format(exc))
            return
        self._wheels = wheels
        self._last_cmd = time.monotonic()

    def _send(self):
        stale = time.monotonic() - self._last_cmd > self._timeout
        left, right = (0.0, 0.0) if stale else self._wheels
        self._write(encode_wheels(left, right))

    def _write(self, data):
        try:
            self._port.write(data)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error('write failed: {}'.format(exc),
                                    throttle_duration_sec=2.0)

    def shutdown(self):
        # Best effort, straight down the port, as the vendor driver does.
        self._write(encode_wheels(0.0, 0.0))
        try:
            self._port.close()
        except Exception:  # noqa: BLE001 - nothing to do about a dead port
            pass


def main():
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

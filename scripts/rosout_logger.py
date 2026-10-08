#!/usr/bin/env python3
"""Save everything the nodes log on /rosout to a file, with wall-clock times.

    python3 scripts/rosout_logger.py [output_file]

Default output: ~/ros_logs/rosout-<start time>.log. Needs the ROS environment
sourced. Line format: HH:MM:SS.mmm LEVEL [node] message. Stop with Ctrl-C.
"""

import os
import sys
import time

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


class RosoutLogger(Node):

    def __init__(self, path):
        super().__init__('rosout_logger')
        self._file = open(path, 'a', buffering=1, encoding='utf-8')
        self.create_subscription(
            Log, '/rosout', self._on_log,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(f'logging /rosout to {path}')

    def _on_log(self, msg):
        if msg.name == self.get_name():
            return
        stamp = msg.stamp.sec + msg.stamp.nanosec / 1e9
        clock = time.strftime('%H:%M:%S', time.localtime(stamp))
        self._file.write(
            f'{clock}.{int(stamp % 1 * 1000):03d} '
            f'{LEVELS.get(msg.level, msg.level)} [{msg.name}] {msg.msg}\n')


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        directory = os.path.expanduser('~/ros_logs')
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(
            directory, time.strftime('rosout-%Y%m%d-%H%M%S.log'))
    rclpy.init()
    node = RosoutLogger(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

import os
import time

import pytest
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from rosidl_runtime_py.utilities import get_message

from mission.topic_health import TopicFreshness, parse_topic_spec

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class _FakeClock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def _spin_until(executor, condition, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not condition():
        executor.spin_once(timeout_sec=0.05)
    return condition()


def test_parse_topic_spec():
    assert parse_topic_spec(' /scan  sensor_msgs/msg/LaserScan 1.5 ') == (
        '/scan', 'sensor_msgs/msg/LaserScan', 1.5)


@pytest.mark.parametrize('spec', ['/scan sensor_msgs/msg/LaserScan', '/scan a b c'])
def test_parse_topic_spec_rejects_malformed(spec):
    with pytest.raises(ValueError):
        parse_topic_spec(spec)


def test_hardware_config_specs_parse_and_name_real_message_types():
    with open(os.path.join(PKG, 'config', 'health_hardware.yaml')) as f:
        specs = yaml.safe_load(f)['mission_node']['ros__parameters']['health_topics']
    assert specs
    for topic, type_name, max_age in map(parse_topic_spec, specs):
        assert topic.startswith('/') and max_age > 0
        get_message(type_name)  # raises if the type does not exist


def test_never_seen_stale_then_fresh_then_stale_again():
    rclpy.init()
    try:
        node = rclpy.create_node('test_topic_freshness')
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        clock = _FakeClock()
        freshness = TopicFreshness(node, [
            ('/test_health/scan', 'sensor_msgs/msg/LaserScan', 1.0),
            ('/test_health/imu', 'sensor_msgs/msg/Imu', 0.5)], clock=clock)
        assert freshness.stale_topics() == ['/test_health/scan', '/test_health/imu']

        # Reliable publisher: a best-effort watcher must still hear it.
        scan = node.create_publisher(LaserScan, '/test_health/scan', 10)
        imu = node.create_publisher(Imu, '/test_health/imu', qos_profile_sensor_data)

        def publish_both_and_check():
            scan.publish(LaserScan())
            imu.publish(Imu())
            return freshness.stale_topics() == []
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not publish_both_and_check():
            executor.spin_once(timeout_sec=0.05)
        assert freshness.stale_topics() == []
        # Drain messages still queued from the loop, so none lands after the
        # clock jumps below and refreshes a topic the test expects stale.
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)

        clock.now += 0.75  # past imu's 0.5 s, within scan's 1.0 s
        assert freshness.stale_topics() == ['/test_health/imu']
        clock.now += 0.5
        assert freshness.stale_topics() == ['/test_health/scan', '/test_health/imu']

        imu.publish(Imu())
        assert _spin_until(executor, lambda: freshness.stale_topics() == ['/test_health/scan'])
    finally:
        rclpy.shutdown()


def test_no_specs_is_never_stale():
    rclpy.init()
    try:
        assert TopicFreshness(rclpy.create_node('test_topic_freshness_empty'), []).stale_topics() == []
    finally:
        rclpy.shutdown()

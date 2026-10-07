"""Sensor/driver liveness for VehicleStatusOK: is each topic still arriving?

A dead driver does not always show up anywhere else. If the lidar stops,
slam_toolbox keeps republishing its last map->odom and Nav2 simply loses its
obstacles; if the wheel MCU stops, the EKF keeps predicting for a while. So
each watched topic must have delivered a message within its own max age.

Only arrival matters, so subscriptions are raw (no deserialization) and
best-effort (matches both reliable and best-effort publishers). Arrival is
timed on the monotonic clock, not the node's, so it means the same thing in
simulation and on the robot.
"""

import time

from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def parse_topic_spec(spec):
    """'<topic> <pkg/msg/Type> <max_age_sec>' -> (topic, type_name, max_age)."""
    parts = spec.split()
    if len(parts) != 3:
        raise ValueError(f"health topic {spec!r} is not '<topic> <pkg/msg/Type> <max_age_sec>'")
    topic, type_name, max_age = parts
    return topic, type_name, float(max_age)


class TopicFreshness:
    def __init__(self, node, specs, clock=time.monotonic):
        """`specs`: iterable of (topic, type_name, max_age_sec)."""
        self._clock = clock
        self._max_age = {}
        self._last_seen = {}
        self._subscriptions = []
        for topic, type_name, max_age in specs:
            self._max_age[topic] = max_age
            self._subscriptions.append(node.create_subscription(
                get_message(type_name), topic,
                lambda _msg, topic=topic: self._last_seen.__setitem__(topic, self._clock()),
                qos_profile_sensor_data, raw=True))

    def stale_topics(self):
        """Watched topics never seen, or silent for longer than their max age."""
        now = self._clock()
        return [topic for topic, max_age in self._max_age.items()
                if now - self._last_seen.get(topic, float('-inf')) > max_age]

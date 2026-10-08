import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

from mission.dock_markers import build_dock_markers
from mission.docks import load_docks
from mission.mission_tree import build_mission_tree
from mission.saga_client import SagaApi
from mission.topic_health import parse_topic_spec

import py_trees_ros


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameter('saga_url', 'http://localhost:8000')
        self.declare_parameter('saga_token', '')
        self.declare_parameter('docks_file', get_package_share_directory('mission') + '/config/docks.yaml')
        self.declare_parameter('tick_hz', 1.0)
        # false = safe stop does not publish zero velocity; diagnostic only.
        self.declare_parameter('safe_stop_zero_hold', True)
        # '<topic> <pkg/msg/Type> <max_age_sec>' each; unset = no topic checks
        # (config/health_hardware.yaml sets it on the robot).
        self.declare_parameter('health_topics', Parameter.Type.STRING_ARRAY)
        health_specs = self.get_parameter_or(
            'health_topics', Parameter('health_topics', value=None)).value or []
        health_topics = [parse_topic_spec(spec) for spec in health_specs]

        saga_client = SagaApi(
            self.get_parameter('saga_url').value,
            self.get_parameter('saga_token').value,
        )
        docks = load_docks(self.get_parameter('docks_file').value)

        # Latched, so Foxglove/RViz get the markers whenever they subscribe.
        self._dock_markers_pub = self.create_publisher(
            MarkerArray, '~/dock_markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._dock_markers_pub.publish(build_dock_markers(docks))

        root = build_mission_tree(self, saga_client, docks, health_topics,
                                  self.get_parameter('safe_stop_zero_hold').value)
        # py_trees_ros.trees.BehaviourTree (not plain py_trees.trees.BehaviourTree) is the
        # ROS2-integrated tree class: its setup() wires the tree to this node (blackboard
        # exchange, snapshot-stream services, a declared `setup_timeout` parameter) instead
        # of just calling each behaviour's setup() in isolation.
        self.tree = py_trees_ros.trees.BehaviourTree(root)
        self.tree.setup(node=self, timeout=15.0)

        tick_hz = self.get_parameter('tick_hz').value
        self.create_timer(1.0 / tick_hz, self.tree.tick)


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

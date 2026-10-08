"""ROS 2 node that runs the mission behaviour tree."""
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import MarkerArray

from mission.dock_markers import build_dock_markers
from mission.docks import load_docks
from mission.mission_tree import build_mission_tree
from mission.saga_client import SagaApi

import py_trees_ros


class MissionNode(Node):
    """Builds the mission tree and ticks it at `tick_hz`.

    Parameters:
        saga_url: Saga server base URL.
        saga_token: Saga bearer token.
        docks_file: Path to the docks YAML.
        tick_hz: Tree tick rate in Hz.
    """

    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameter('saga_url', 'http://localhost:8000')
        self.declare_parameter('saga_token', '')
        self.declare_parameter('docks_file', get_package_share_directory('mission') + '/config/docks.yaml')
        self.declare_parameter('tick_hz', 1.0)

        saga_client = SagaApi(
            self.get_parameter('saga_url').value,
            self.get_parameter('saga_token').value,
        )
        docks = load_docks(self.get_parameter('docks_file').value)

        # Latched so late subscribers still get the markers.
        self._dock_markers_pub = self.create_publisher(
            MarkerArray, '~/dock_markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._dock_markers_pub.publish(build_dock_markers(docks))

        root = build_mission_tree(self, saga_client, docks)
        # The ROS tree class wires setup() to this node (blackboard exchange,
        # snapshot services); plain py_trees only calls each behaviour's setup().
        self.tree = py_trees_ros.trees.BehaviourTree(root)
        self.tree.setup(node=self, timeout=15.0)

        tick_hz = self.get_parameter('tick_hz').value
        self.create_timer(1.0 / tick_hz, self.tree.tick)


def main():
    """Runs the mission node until shutdown."""
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

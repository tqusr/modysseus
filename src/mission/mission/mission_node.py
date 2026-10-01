import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from mission.docks import load_docks
from mission.mission_tree import build_mission_tree
from mission.saga_client import SagaApi

import py_trees_ros


class MissionNode(Node):
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

        root = build_mission_tree(self, saga_client, docks)
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

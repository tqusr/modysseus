"""Stub Nav2 and docking action servers for running the mission without them."""
import time

import rclpy
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionServer
from rclpy.node import Node


class StubActionServers(Node):
    """Accepts any NavigateToPose, DockRobot or UndockRobot goal and succeeds.

    Each goal sleeps `delay_sec` first. The execute callbacks run on the
    single spin thread, so the sleep blocks the node. That is fine because
    the mission tree sends one goal at a time and this runs in its own
    process. Do not add a multi-threaded executor here.

    Attributes:
        delay_sec: Seconds each goal takes before it succeeds.
    """

    def __init__(self, delay_sec: float = 2.0):
        """Initializes the node and its three action servers.

        Args:
            delay_sec: Seconds each goal takes before it succeeds.
        """
        super().__init__('stub_action_servers')
        self.delay_sec = delay_sec
        self._navigate_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self._execute_navigate)
        self._dock_server = ActionServer(
            self, DockRobot, 'dock_robot', self._execute_dock)
        self._undock_server = ActionServer(
            self, UndockRobot, 'undock_robot', self._execute_undock)

    def _execute_navigate(self, goal_handle):
        """Succeeds a NavigateToPose goal after `delay_sec`."""
        self.get_logger().info('Stub NavigateToPose: driving...')
        time.sleep(self.delay_sec)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _execute_dock(self, goal_handle):
        """Succeeds a DockRobot goal after `delay_sec`."""
        self.get_logger().info('Stub DockRobot: docking...')
        time.sleep(self.delay_sec)
        goal_handle.succeed()
        return DockRobot.Result()

    def _execute_undock(self, goal_handle):
        """Succeeds an UndockRobot goal after `delay_sec`."""
        self.get_logger().info('Stub UndockRobot: undocking...')
        time.sleep(self.delay_sec)
        goal_handle.succeed()
        return UndockRobot.Result()


def main():
    """Runs the stub servers until shutdown."""
    rclpy.init()
    node = StubActionServers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

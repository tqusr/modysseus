import time

import rclpy
from nav2_msgs.action import DockRobot, NavigateToPose
from rclpy.action import ActionServer
from rclpy.node import Node


class StubActionServers(Node):
    """Stands in for Nav2's NavigateToPose and the docking server's DockRobot
    until those owners' real implementations land. Accepts any goal, waits
    `delay_sec`, succeeds.

    Threading note: `ActionServer` schedules each goal's execute callback via
    `self._node.executor.create_task(...)` (rclpy/action/server.py), which just
    queues it for the executor's own spin loop to run. With the default
    `SingleThreadedExecutor`, that means the execute callback runs inline on
    whatever thread calls `spin()` (see `SingleThreadedExecutor`'s docstring:
    "Runs callbacks in the thread that calls Executor.spin()") -- there is no
    separate worker thread. So `time.sleep(self.delay_sec)` below blocks that
    one thread for the full delay, during which this node can't service any
    other callback. That's fine here: this stub only ever has one goal in
    flight at a time (the mission tree drives navigate/wait/dock strictly
    sequentially) and runs as its own process (see the mission launch file),
    so blocking its own single thread for `delay_sec` per goal has no effect
    on anything else. Do not add a `ReentrantCallbackGroup`/
    `MultiThreadedExecutor` for this -- nothing in this plan needs concurrent
    goal handling here.
    """

    def __init__(self, delay_sec: float = 2.0):
        super().__init__('stub_action_servers')
        self.delay_sec = delay_sec
        self._navigate_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self._execute_navigate)
        self._dock_server = ActionServer(
            self, DockRobot, 'dock_robot', self._execute_dock)

    def _execute_navigate(self, goal_handle):
        self.get_logger().info('Stub NavigateToPose: driving...')
        time.sleep(self.delay_sec)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _execute_dock(self, goal_handle):
        self.get_logger().info('Stub DockRobot: docking...')
        time.sleep(self.delay_sec)
        goal_handle.succeed()
        return DockRobot.Result()


def main():
    rclpy.init()
    node = StubActionServers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

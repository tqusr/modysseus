import time

import py_trees
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient


class BlackboardFlagTrue(py_trees.behaviour.Behaviour):
    """Generic reusable condition: SUCCESS if blackboard[key] is truthy."""

    def __init__(self, name, key):
        super().__init__(name)
        self.key = key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def update(self):
        value = self.blackboard.get(self.key) if self.blackboard.exists(self.key) else False
        return py_trees.common.Status.SUCCESS if value else py_trees.common.Status.FAILURE


class SetBlackboardFlag(py_trees.behaviour.Behaviour):
    """Generic reusable action: writes `value` to blackboard[key], always SUCCESS."""

    def __init__(self, name, key, value):
        super().__init__(name)
        self.key = key
        self.value = value
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.set(self.key, self.value)
        return py_trees.common.Status.SUCCESS


class WaitUntil(py_trees.behaviour.Behaviour):
    """Polls `condition` until it's true or `timeout_sec` elapses.

    Non-blocking: each tick either returns immediately (no sleeping) or
    reports RUNNING. `clock` is injectable so tests don't need real time.
    """

    def __init__(self, name, condition, timeout_sec, poll_sec=1.0, clock=time.monotonic):
        super().__init__(name)
        self.condition = condition
        self.timeout_sec = timeout_sec
        self.poll_sec = poll_sec
        self.clock = clock
        self._start_time = None
        self._last_poll = None

    def initialise(self):
        self._start_time = self.clock()
        self._last_poll = None

    def update(self):
        now = self.clock()
        if now - self._start_time > self.timeout_sec:
            return py_trees.common.Status.FAILURE
        if self._last_poll is None:
            # Force one RUNNING tick before ever checking the condition, so a
            # condition that's already true doesn't resolve in a single tick.
            self._last_poll = now
            return py_trees.common.Status.RUNNING
        if (now - self._last_poll) < self.poll_sec:
            return py_trees.common.Status.RUNNING
        self._last_poll = now
        if self.condition():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class CallAction(py_trees.behaviour.Behaviour):
    """Sends a ROS 2 action goal on initialise(); RUNNING until a result
    arrives, then SUCCESS/FAILURE based on it. `on_success` fires once,
    exactly when the result is SUCCEEDED, before returning SUCCESS.
    """

    def __init__(self, name, node, action_type, action_name, goal_builder, on_success=None):
        super().__init__(name)
        self.node = node
        self.goal_builder = goal_builder
        self.on_success = on_success
        self.client = ActionClient(node, action_type, action_name)
        self._send_future = None
        self._result_future = None

    def initialise(self):
        self._result_future = None
        self._send_future = self.client.send_goal_async(self.goal_builder())

    def update(self):
        if self._send_future is not None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            goal_handle = self._send_future.result()
            self._send_future = None
            if not goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            self._result_future = goal_handle.get_result_async()

        if self._result_future is not None:
            if not self._result_future.done():
                return py_trees.common.Status.RUNNING
            result = self._result_future.result()
            self._result_future = None
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                if self.on_success is not None:
                    self.on_success()
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

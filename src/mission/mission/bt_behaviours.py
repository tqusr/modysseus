"""Reusable py_trees behaviours for the mission tree."""
import time

import py_trees
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient


class BlackboardFlagTrue(py_trees.behaviour.Behaviour):
    """Succeeds if `blackboard[key]` is truthy, otherwise fails."""

    def __init__(self, name, key):
        """Initializes the condition.

        Args:
            name: Behaviour name.
            key: Blackboard key to read.
        """
        super().__init__(name)
        self.key = key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def update(self):
        value = self.blackboard.get(self.key) if self.blackboard.exists(self.key) else False
        return py_trees.common.Status.SUCCESS if value else py_trees.common.Status.FAILURE


class SetBlackboardFlag(py_trees.behaviour.Behaviour):
    """Writes `value` to `blackboard[key]` and always succeeds."""

    def __init__(self, name, key, value):
        """Initializes the action.

        Args:
            name: Behaviour name.
            key: Blackboard key to write.
            value: Value to write.
        """
        super().__init__(name)
        self.key = key
        self.value = value
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.set(self.key, self.value)
        return py_trees.common.Status.SUCCESS


class WaitUntil(py_trees.behaviour.Behaviour):
    """Polls `condition` until it is true or `timeout_sec` elapses.

    Never sleeps: each tick returns immediately, RUNNING while waiting.
    The first tick always returns RUNNING, even if the condition is already
    true.
    """

    def __init__(self, name, condition, timeout_sec, poll_sec=1.0, clock=time.monotonic):
        """Initializes the wait.

        Args:
            name: Behaviour name.
            condition: Zero-argument callable returning a truthy value when done.
            timeout_sec: Seconds after which the behaviour fails.
            poll_sec: Minimum seconds between condition checks.
            clock: Monotonic clock in seconds; injectable for tests.
        """
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
            self._last_poll = now
            return py_trees.common.Status.RUNNING
        if (now - self._last_poll) < self.poll_sec:
            return py_trees.common.Status.RUNNING
        self._last_poll = now
        if self.condition():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


def _cancel_once_accepted(send_future):
    """Cancels the goal behind a `send_goal_async` future, if it was accepted.

    Stops an abandoned goal from driving the robot when the behaviour was
    invalidated before `update()` took the goal handle. Used both as a done
    callback (server not yet answered) and called directly (future already
    resolved).

    Args:
        send_future: Future returned by `ActionClient.send_goal_async`.
    """
    try:
        goal_handle = send_future.result()
    except Exception:  # Cancelled or errored: nothing was accepted.
        return
    if goal_handle is not None and goal_handle.accepted:
        goal_handle.cancel_goal_async()


class CallAction(py_trees.behaviour.Behaviour):
    """Sends an action goal and reports its result.

    Sends the goal on `initialise()`, returns RUNNING until the result
    arrives, then SUCCESS or FAILURE. `on_success` runs once, just before
    SUCCESS is returned.

    The goal lives on the remote server. If py_trees invalidates this
    behaviour (a guard failed), `terminate()` cancels the goal there so Nav2
    or the docking server stops driving the robot.
    """

    def __init__(self, name, node, action_type, action_name, goal_builder, on_success=None,
                 log_tag=None, describe=None):
        """Initializes the action call.

        Args:
            name: Behaviour name.
            node: Node that owns the action client.
            action_type: ROS action type.
            action_name: Action server name.
            goal_builder: Zero-argument callable returning the goal.
            on_success: Optional callable run when the result is SUCCEEDED.
            log_tag: Prefix for operator-facing log lines.
            describe: Optional zero-argument callable returning a short
                description of the goal, e.g. "going to dock A". Without it
                the behaviour logs nothing.
        """
        super().__init__(name)
        self.node = node
        self.goal_builder = goal_builder
        self.on_success = on_success
        self._log_tag = log_tag
        self._describe = describe
        self._described = None
        self.client = ActionClient(node, action_type, action_name)
        self._send_future = None
        self._result_future = None
        # Kept so terminate() can cancel the goal; None when no goal is in flight.
        self._goal_handle = None

    def _say(self, level, text):
        """Logs `text` as "[log_tag] text" at `level` if a description exists."""
        if self._described is not None:
            getattr(self.logger, level)(f"[{self._log_tag}] {text}")

    def initialise(self):
        self._result_future = None
        self._goal_handle = None
        self._described = self._describe() if self._describe is not None else None
        self._say("info", self._described)
        self._send_future = self.client.send_goal_async(self.goal_builder())

    def update(self):
        if self._send_future is not None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            goal_handle = self._send_future.result()
            self._send_future = None
            if not goal_handle.accepted:
                self._say("warning", f"failed (goal rejected): {self._described}")
                return py_trees.common.Status.FAILURE
            self._goal_handle = goal_handle
            self._result_future = goal_handle.get_result_async()

        if self._result_future is not None:
            if not self._result_future.done():
                return py_trees.common.Status.RUNNING
            result = self._result_future.result()
            self._result_future = None
            # The goal is terminal on the server; nothing left to cancel.
            self._goal_handle = None
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                if self.on_success is not None:
                    self.on_success()
                self._say("info", f"done: {self._described}")
                return py_trees.common.Status.SUCCESS
            self._say("warning", f"failed (status {result.status}): {self._described}")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Cancels an in-flight goal when the behaviour is invalidated.

        INVALID means the behaviour left the active branch, e.g. a failing
        guard in the reactive Mission sequence. SUCCESS and FAILURE need no
        cancel because `update()` already saw a terminal result.

        Both NavigateToPose (nav2_bt_navigator) and DockRobot/UndockRobot
        (opennav_docking) support cancellation.

        Args:
            new_status: The status the behaviour is moving to.
        """
        if new_status == py_trees.common.Status.INVALID:
            if self._goal_handle is not None or self._send_future is not None:
                self._say("warning", f"cancelled: {self._described}")
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            elif self._send_future is not None:
                # `update()` may never have taken the handle: the goal is sent
                # and the next tick's failing guard invalidates us before
                # `update()` runs again. Cancel whether the future has
                # resolved or not.
                if self._send_future.done():
                    _cancel_once_accepted(self._send_future)
                else:
                    self._send_future.add_done_callback(_cancel_once_accepted)
            self._send_future = None
            self._result_future = None
            self._goal_handle = None

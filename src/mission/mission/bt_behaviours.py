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


def _cancel_once_accepted(send_future):
    """Cancel the goal behind a send_goal_async future whose behaviour was
    already invalidated before update() harvested the goal handle, so an
    abandoned goal can't keep driving the robot. Without this, the window
    between "goal sent" and "goal handle harvested" (at 1 Hz ticks, up to a
    full tick wide) would leak an uncancellable goal.

    Used two ways by terminate(), because the future can be in either state
    at invalidation time and BOTH must cancel (see terminate()):
      - as a done-callback, when the server hasn't answered yet;
      - called directly, when the future has already resolved but no tick
        has run since to move the handle into _goal_handle.
    Safe either way: it only touches the future's already-available result."""
    try:
        goal_handle = send_future.result()
    except Exception:  # future cancelled/errored -- nothing was ever accepted
        return
    if goal_handle is not None and goal_handle.accepted:
        goal_handle.cancel_goal_async()


class CallAction(py_trees.behaviour.Behaviour):
    """Sends a ROS 2 action goal on initialise(); RUNNING until a result
    arrives, then SUCCESS/FAILURE based on it. `on_success` fires once,
    exactly when the result is SUCCEEDED, before returning SUCCESS.

    SAFETY: an action goal is state held by a *remote* server, not by this
    behaviour. py_trees invalidating this behaviour (a reactive Sequence
    killing a dangling runner when a guard fails -- see mission_tree.build_mission_tree) only drops this side's
    futures; Nav2's controller_server / opennav_docking's docking_server go
    right on executing the goal, i.e. right on driving the robot, with
    nobody left watching. `terminate()` below closes that by actually
    cancelling the goal on the server (both NavigateToPose and
    DockRobot/UndockRobot support cancellation -- see its docstring).
    """

    def __init__(self, name, node, action_type, action_name, goal_builder, on_success=None):
        super().__init__(name)
        self.node = node
        self.goal_builder = goal_builder
        self.on_success = on_success
        self.client = ActionClient(node, action_type, action_name)
        self._send_future = None
        self._result_future = None
        # The accepted goal's handle, held for exactly one reason: so
        # terminate() can cancel the goal server-side. None whenever no goal
        # of ours is in flight.
        self._goal_handle = None

    def initialise(self):
        self._result_future = None
        self._goal_handle = None
        self._send_future = self.client.send_goal_async(self.goal_builder())

    def update(self):
        if self._send_future is not None:
            if not self._send_future.done():
                return py_trees.common.Status.RUNNING
            goal_handle = self._send_future.result()
            self._send_future = None
            if not goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            self._goal_handle = goal_handle
            self._result_future = goal_handle.get_result_async()

        if self._result_future is not None:
            if not self._result_future.done():
                return py_trees.common.Status.RUNNING
            result = self._result_future.result()
            self._result_future = None
            # The goal has reached a terminal state server-side; there is
            # nothing left to cancel.
            self._goal_handle = None
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                if self.on_success is not None:
                    self.on_success()
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Cancel an in-flight goal when this behaviour is INVALIDATED.

        INVALID (as opposed to SUCCESS/FAILURE, where update() has already
        seen a terminal result and cleared the handle) is py_trees' "you are
        no longer part of the active branch" signal -- for this tree, that's
        a failed guard: VehicleStatusOK/Localize failing makes the
        reactive `Mission` Sequence invalidate the whole `route` subtree
        underneath it, which propagates down to whichever CallAction was
        RUNNING. Letting that goal keep running would mean the behaviour
        tree reports "stopped" while Nav2/opennav_docking is still actively
        commanding velocity.

        Cancellation is supported by every action type this class is used
        with here (grep `CallAction(` in mission_tree.py: NavigateToPose,
        DockRobot, UndockRobot -- all three), confirmed against the
        installed Jazzy packages on this machine, not assumed:
          - NavigateToPose: nav2_bt_navigator's BtActionServer polls
            `action_server_->is_cancel_requested()` from its BT run loop and
            halts the tree on a cancel, logging "Goal canceled"
            (/opt/ros/jazzy/include/nav2_behavior_tree/
            bt_action_server_impl.hpp, lines ~303-351).
          - DockRobot/UndockRobot: opennav_docking's docking_server runs
            `checkAndWarnIfCancelled(...)` every control cycle and publishes
            zero velocity on its way out -- both template instantiations
            (`...checkAndWarnIfCancelledI...DockRobot...` and
            `...UndockRobot...`) are present in
            /opt/ros/jazzy/lib/opennav_docking/opennav_docking.
        """
        if new_status == py_trees.common.Status.INVALID:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            elif self._send_future is not None:
                # Both halves of the accept race have to cancel, and the
                # already-resolved half is the LIKELIER one, not an exotic
                # corner: initialise() sends the goal and update() runs in
                # the same tick with no spin in between, so the handle is
                # never harvested on the dispatching tick; the executor then
                # resolves the send future within milliseconds; and the next
                # tick's failing VehicleStatusOK invalidates the `route`
                # subtree through a memory=False Sequence, which stops
                # children[index+1:] WITHOUT ticking them -- so update()
                # never runs again and _goal_handle stays None. Checking only
                # `not done()` here (what the first fix wave did) therefore
                # issued no cancel at all for a full tick after every single
                # goal dispatch.
                if self._send_future.done():
                    _cancel_once_accepted(self._send_future)
                else:
                    self._send_future.add_done_callback(_cancel_once_accepted)
            self._send_future = None
            self._result_future = None
            self._goal_handle = None

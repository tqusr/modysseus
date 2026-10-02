import time

import py_trees

from mission.bt_behaviours import WaitUntil


class _FakeClock:
    def __init__(self, start=0.0):
        self.now = start

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def test_wait_until_succeeds_once_condition_is_true():
    clock = _FakeClock()
    condition = {"value": False}
    node = WaitUntil(
        "wait",
        condition=lambda: condition["value"],
        timeout_sec=10.0,
        poll_sec=1.0,
        clock=clock,
    )
    node.initialise()

    assert node.update() == py_trees.common.Status.RUNNING

    condition["value"] = True
    clock.advance(1.0)  # past the 1s poll interval, so the next tick re-checks
    assert node.update() == py_trees.common.Status.SUCCESS


def test_wait_until_stays_running_between_polls_even_if_true():
    clock = _FakeClock()
    node = WaitUntil("wait", condition=lambda: True, timeout_sec=10.0, poll_sec=1.0, clock=clock)
    node.initialise()

    assert node.update() == py_trees.common.Status.RUNNING  # first tick always RUNNING...
    clock.advance(1.0)  # ...then poll_sec must elapse before the next check
    assert node.update() == py_trees.common.Status.SUCCESS


def test_wait_until_fails_after_timeout():
    clock = _FakeClock()
    node = WaitUntil("wait", condition=lambda: False, timeout_sec=5.0, poll_sec=1.0, clock=clock)
    node.initialise()

    node.update()
    clock.advance(6.0)
    assert node.update() == py_trees.common.Status.FAILURE


from mission.bt_behaviours import BlackboardFlagTrue, SetBlackboardFlag


def test_blackboard_flag_true_fails_when_unset():
    node = BlackboardFlagTrue("flag?", key="holds_package")
    node.setup()
    assert node.update() == py_trees.common.Status.FAILURE


def test_set_then_check_blackboard_flag():
    setter = SetBlackboardFlag("set flag", key="holds_package", value=True)
    checker = BlackboardFlagTrue("flag?", key="holds_package")
    setter.setup()
    checker.setup()

    assert setter.update() == py_trees.common.Status.SUCCESS
    assert checker.update() == py_trees.common.Status.SUCCESS


def test_wait_until_does_not_poll_condition_before_poll_interval_elapses():
    clock = _FakeClock()
    calls = {"count": 0}

    def counting_condition():
        calls["count"] += 1
        return False

    node = WaitUntil("wait", condition=counting_condition, timeout_sec=10.0, poll_sec=5.0, clock=clock)
    node.initialise()

    node.update()  # first tick: forced RUNNING, condition not called yet
    node.update()  # second tick, same clock time: still within poll_sec, condition not called
    node.update()  # third tick, same clock time: still within poll_sec, condition not called

    assert calls["count"] == 0

    clock.advance(5.0)
    node.update()  # poll_sec has now elapsed: condition gets called exactly once

    assert calls["count"] == 1


import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

from mission.bt_behaviours import CallAction
from mission.stub_action_servers import StubActionServers


def test_call_action_reaches_success_and_fires_on_success_callback():
    rclpy.init()
    try:
        server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_bt_client")
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)

        fired = {"value": False}
        node = CallAction(
            "navigate",
            node=client_node,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            goal_builder=NavigateToPose.Goal,
            on_success=lambda: fired.__setitem__("value", True),
        )
        node.client.wait_for_server(timeout_sec=5.0)
        node.initialise()

        status = py_trees.common.Status.RUNNING
        deadline = time.monotonic() + 5.0
        while status == py_trees.common.Status.RUNNING and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            status = node.update()

        assert status == py_trees.common.Status.SUCCESS
        assert fired["value"] is True
    finally:
        rclpy.shutdown()


class _RecordingGoalHandle:
    """Stands in for an rclpy ClientGoalHandle: records cancel requests.

    A real in-flight goal can't be used here without a cancellable action
    server AND a multi-threaded executor (the stub servers block their one
    executor thread inside execute, so they can't service a cancel request).
    The contract under test is this class's own: "an invalidated CallAction
    asks the server to cancel", i.e. that cancel_goal_async() is called at
    all. The server side of cancellation is verified live, not here -- see
    this fix wave's report."""

    def __init__(self):
        self.accepted = True
        self.cancel_calls = 0

    def cancel_goal_async(self):
        self.cancel_calls += 1


def _call_action_with_fake_handle(node):
    behaviour = CallAction(
        "navigate",
        node=node,
        action_type=NavigateToPose,
        action_name="navigate_to_pose",
        goal_builder=NavigateToPose.Goal,
    )
    goal_handle = _RecordingGoalHandle()
    behaviour._goal_handle = goal_handle
    behaviour._result_future = object()  # stands in for a pending result
    return behaviour, goal_handle


def test_call_action_terminate_invalid_cancels_the_in_flight_goal():
    """The safe-stop fix: py_trees invalidating a RUNNING CallAction (root
    Selector switching to safe_stop) must actually cancel the goal on the
    server, not just drop this side's futures -- otherwise Nav2 /
    opennav_docking keeps driving the robot while the tree reports
    "stopped"."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_call_action_terminate_invalid")
        behaviour, goal_handle = _call_action_with_fake_handle(node)

        behaviour.terminate(py_trees.common.Status.INVALID)

        assert goal_handle.cancel_calls == 1
        assert behaviour._goal_handle is None
        assert behaviour._send_future is None
        assert behaviour._result_future is None
    finally:
        rclpy.shutdown()


def test_call_action_terminate_success_does_not_cancel():
    """terminate() also fires on every ordinary completion (py_trees calls
    stop(SUCCESS)/stop(FAILURE) from tick()). Cancelling there would be a
    pointless round trip against an already-terminal goal, so only INVALID
    cancels."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_call_action_terminate_success")
        behaviour, goal_handle = _call_action_with_fake_handle(node)

        behaviour.terminate(py_trees.common.Status.SUCCESS)
        behaviour.terminate(py_trees.common.Status.FAILURE)

        assert goal_handle.cancel_calls == 0
    finally:
        rclpy.shutdown()


def test_call_action_terminate_invalid_without_a_goal_handle_is_a_noop():
    """Invalidated before ever sending/accepting a goal (e.g. safe-stop trips
    on the very tick a branch is first reached) -- must not raise."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_call_action_terminate_no_goal")
        behaviour = CallAction(
            "navigate", node=node, action_type=NavigateToPose,
            action_name="navigate_to_pose", goal_builder=NavigateToPose.Goal,
        )

        behaviour.terminate(py_trees.common.Status.INVALID)

        assert behaviour._goal_handle is None
    finally:
        rclpy.shutdown()


def test_call_action_terminate_invalid_cancels_a_goal_still_being_accepted():
    """The race the `_cancel_once_accepted` callback exists for: the goal was
    sent but the server hadn't returned its handle yet when the behaviour was
    invalidated. Dropping the future there would leak a goal nobody can
    cancel afterwards, so the cancel has to be deferred onto the future."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_call_action_terminate_racing_send")
        behaviour = CallAction(
            "navigate", node=node, action_type=NavigateToPose,
            action_name="navigate_to_pose", goal_builder=NavigateToPose.Goal,
        )
        send_future = Future()
        behaviour._send_future = send_future

        behaviour.terminate(py_trees.common.Status.INVALID)

        # Server only now accepts the goal -- the deferred callback must
        # cancel it.
        goal_handle = _RecordingGoalHandle()
        send_future.set_result(goal_handle)

        assert goal_handle.cancel_calls == 1
    finally:
        rclpy.shutdown()


class _CancellableServer:
    """A real action server that accepts goals, never finishes them, and
    records cancel requests.

    Deferred execution (`handle_accepted_callback` that only stashes the
    handle) is what makes this usable on a SingleThreadedExecutor: an
    execute callback that looped waiting for a cancel would block the one
    executor thread and never service the cancel request itself."""

    def __init__(self, node):
        self.accepted_goals = []
        self.cancel_requests = []
        self._server = ActionServer(
            node, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute,
            handle_accepted_callback=self.accepted_goals.append,
            cancel_callback=self._cancel,
        )

    def _execute(self, goal_handle):  # pragma: no cover - never invoked
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _cancel(self, goal_handle):
        self.cancel_requests.append(goal_handle)
        return CancelResponse.ACCEPT


def test_call_action_cancel_reaches_a_real_action_server():
    """End-to-end over real rclpy action plumbing, not a fake handle: tick
    CallAction until it holds an accepted goal, invalidate it the way
    py_trees does on a safe stop, and assert the SERVER actually receives
    the cancel request. This is the behaviour the whole safe-stop fix rests
    on -- Nav2/opennav_docking keep driving the robot until this request
    arrives."""
    rclpy.init()
    try:
        server_node = rclpy.create_node("test_cancellable_server")
        client_node = rclpy.create_node("test_cancel_client")
        executor = SingleThreadedExecutor()
        executor.add_node(server_node)
        executor.add_node(client_node)
        server = _CancellableServer(server_node)

        behaviour = CallAction(
            "navigate", node=client_node, action_type=NavigateToPose,
            action_name="navigate_to_pose", goal_builder=NavigateToPose.Goal,
        )
        assert behaviour.client.wait_for_server(timeout_sec=10.0)
        behaviour.initialise()

        deadline = time.monotonic() + 10.0
        while behaviour._goal_handle is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            assert behaviour.update() == py_trees.common.Status.RUNNING
        assert behaviour._goal_handle is not None, "goal was never accepted"

        behaviour.terminate(py_trees.common.Status.INVALID)

        deadline = time.monotonic() + 10.0
        while not server.cancel_requests and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        assert server.cancel_requests, \
            "the action SERVER never received a cancel request -- it would keep driving"
    finally:
        rclpy.shutdown()


def test_call_action_records_the_goal_handle_once_the_goal_is_accepted():
    """terminate() can only cancel what update() captured -- this is the
    other half of the fix, against a real action server."""
    rclpy.init()
    try:
        server = StubActionServers(delay_sec=2.0)
        client_node = rclpy.create_node("test_bt_goal_handle_capture")
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)

        behaviour = CallAction(
            "navigate", node=client_node, action_type=NavigateToPose,
            action_name="navigate_to_pose", goal_builder=NavigateToPose.Goal,
        )
        behaviour.client.wait_for_server(timeout_sec=5.0)
        behaviour.initialise()

        deadline = time.monotonic() + 5.0
        while behaviour._goal_handle is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            behaviour.update()

        assert behaviour._goal_handle is not None, \
            "CallAction never captured the accepted goal's handle"
        assert behaviour._goal_handle.accepted is True
    finally:
        rclpy.shutdown()

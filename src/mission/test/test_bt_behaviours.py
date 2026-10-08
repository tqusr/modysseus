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
    """Stands in for an rclpy ClientGoalHandle and records cancel requests.

    A real in-flight goal would need a cancellable action server and a
    multi-threaded executor (the stub servers block their one executor thread in
    execute). The contract under test is that an invalidated CallAction calls
    cancel_goal_async(); the server side is verified live."""

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
    """Invalidating a RUNNING CallAction (a failed Mission guard) cancels the goal.

    Otherwise Nav2 or opennav_docking keeps driving while the tree reports
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
    """terminate() also fires on ordinary completion, which must not cancel.

    Cancelling an already-terminal goal is a pointless round trip, so only
    INVALID cancels."""
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
    """Invalidated before ever sending/accepting a goal (e.g. a guard fails
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
    """The race `_cancel_once_accepted` exists for.

    The goal was sent but the handle had not returned when the behaviour was
    invalidated. Dropping the future would leak an uncancellable goal, so the
    cancel is deferred onto it."""
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
    """A real action server that accepts goals, never finishes them and records cancels.

    Deferred execution (`handle_accepted_callback` only stashes the handle)
    keeps it usable on a SingleThreadedExecutor, where an execute callback
    waiting for a cancel would block the thread that must service it."""

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
    """End to end over real rclpy action plumbing, not a fake handle.

    Ticks CallAction until it holds an accepted goal, invalidates it as py_trees
    does when a guard fails, and asserts the server receives the cancel. The
    stop-on-fault design rests on this: Nav2 keeps driving until it arrives."""
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


def test_call_action_terminate_invalid_cancels_an_accepted_but_unharvested_goal():
    """The other half of the accept race, and the one that actually happens.

    The send future has resolved but no tick has run since, so update() never
    moved the handle into _goal_handle. That is the normal state for a tick after
    every dispatch: initialise() sends and update() runs in the same tick, and the
    memory=False `Mission` sequence then invalidates `route` without ticking it
    again. Deferring onto an already-done future would be too late, and the old
    `not done()` guard skipped the cancel, so the server kept driving through the
    fault.

    Uses real rclpy plumbing and a real server on purpose: a hand-built Future
    would let a wrong fix pass."""
    rclpy.init()
    try:
        server_node = rclpy.create_node("test_unharvested_server")
        client_node = rclpy.create_node("test_unharvested_client")
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

        # Spin until the server has accepted, but never call update(), so the
        # handle is never taken: the interleaving a fault hits after a dispatch.
        deadline = time.monotonic() + 10.0
        while not behaviour._send_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert behaviour._send_future.done(), "the server never answered the goal request"
        assert behaviour._goal_handle is None, \
            "fixture assumption broken: the handle must still be unharvested"

        behaviour.terminate(py_trees.common.Status.INVALID)

        deadline = time.monotonic() + 10.0
        while not server.cancel_requests and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        assert server.cancel_requests, \
            ("an accepted-but-unharvested goal was abandoned without a cancel -- "
             "the server keeps driving the robot through the fault")
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


class _LogRecorder:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warning(self, msg):
        self.lines.append(("warning", msg))


def test_call_action_logs_start_and_arrival_with_its_tag():
    rclpy.init()
    try:
        server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_bt_log_client")
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)
        behaviour = CallAction(
            "navigate", node=client_node, action_type=NavigateToPose,
            action_name="navigate_to_pose", goal_builder=NavigateToPose.Goal,
            log_tag="nav", describe=lambda: "going to dock A staging (1.00, 2.00)")
        behaviour.logger = _LogRecorder()
        behaviour.client.wait_for_server(timeout_sec=5.0)
        behaviour.initialise()
        status = py_trees.common.Status.RUNNING
        deadline = time.monotonic() + 5.0
        while status == py_trees.common.Status.RUNNING and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            status = behaviour.update()
        assert status == py_trees.common.Status.SUCCESS
        assert behaviour.logger.lines == [
            ("info", "[nav] going to dock A staging (1.00, 2.00)"),
            ("info", "[nav] done: going to dock A staging (1.00, 2.00)"),
        ]
    finally:
        rclpy.shutdown()


def test_call_action_logs_a_rejected_goal_as_a_failure():
    rclpy.init()
    try:
        node = rclpy.create_node("test_bt_log_rejected")
        behaviour = CallAction(
            "dock", node=node, action_type=NavigateToPose,
            action_name="navigate_to_pose", goal_builder=NavigateToPose.Goal,
            log_tag="dock", describe=lambda: "docking at A")
        behaviour.logger = _LogRecorder()

        class _Rejected:
            accepted = False

        class _Done:
            def done(self):
                return True

            def result(self):
                return _Rejected()

        behaviour._described = "docking at A"
        behaviour._send_future = _Done()
        assert behaviour.update() == py_trees.common.Status.FAILURE
        assert behaviour.logger.lines == [("warning", "[dock] failed (goal rejected): docking at A")]
    finally:
        rclpy.shutdown()


def test_call_action_without_describe_logs_nothing():
    rclpy.init()
    try:
        node = rclpy.create_node("test_bt_log_none")
        behaviour = CallAction(
            "quiet", node=node, action_type=NavigateToPose,
            action_name="navigate_to_pose", goal_builder=NavigateToPose.Goal)
        behaviour.logger = _LogRecorder()
        behaviour.initialise()
        assert behaviour.logger.lines == []
    finally:
        rclpy.shutdown()

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
from rclpy.executors import SingleThreadedExecutor

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

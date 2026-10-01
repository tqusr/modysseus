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

    assert node.update() == py_trees.common.Status.RUNNING  # first poll happens immediately...
    assert node.update() == py_trees.common.Status.SUCCESS  # ...and succeeds right away


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

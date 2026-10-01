import time

import py_trees


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
            self._last_poll = now
            return py_trees.common.Status.RUNNING
        if self.condition():
            return py_trees.common.Status.SUCCESS
        if (now - self._last_poll) >= self.poll_sec:
            self._last_poll = now
        return py_trees.common.Status.RUNNING

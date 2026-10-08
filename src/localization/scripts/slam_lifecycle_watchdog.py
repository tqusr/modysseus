#!/usr/bin/env python3
"""Drives a slam_toolbox lifecycle node to `active` and verifies it got there.

    slam_lifecycle_watchdog.py [node_name] [timeout_sec]   (/slam_toolbox, 60)

slam_toolbox is a managed lifecycle node that processes no scans until it is
configured and activated. Here its `change_state` response is often lost ("failed
to send response to /slam_toolbox/change_state (timeout)"), so anything
waiting on that response can hang while the node sits `inactive`. The failure
is silent: launch looks healthy, `/map` is never published, and the problem
shows up as an empty saved map.

The watchdog never trusts a response. It polls `get_state` and re-issues the
transition the observed state calls for, so a dropped request or response
just means the next poll retries. If the node is not `active` by the deadline
it logs an ERROR naming the stuck state and exits non-zero.

Exit code 0 means the node was seen `active`. Non-zero means mapping is not
running and the resulting map will be empty or partial.
"""

import sys

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

# States we can act on, mapped to the transition that moves them forward.
_ADVANCE = {
    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
}

# Transitional states: the node is already working, so wait and re-poll.
_IN_FLIGHT = {
    State.TRANSITION_STATE_CONFIGURING,
    State.TRANSITION_STATE_ACTIVATING,
    State.TRANSITION_STATE_CLEANINGUP,
    State.TRANSITION_STATE_DEACTIVATING,
}

_CALL_TIMEOUT = 5.0  # Per service call, so a lost response cannot hang us.
_POLL_PERIOD = 1.0


class LifecycleWatchdog(Node):
    """Polls a lifecycle node and nudges it to `active`."""

    def __init__(self, target, deadline):
        """Initializes the watchdog.

        Args:
            target: Name of the lifecycle node, e.g. "/slam_toolbox".
            deadline: Clock time in nanoseconds after which `run` gives up.
        """
        super().__init__('slam_lifecycle_watchdog')
        self._target = target
        self._deadline = deadline
        self._get = self.create_client(GetState, f'{target}/get_state')
        self._change = self.create_client(ChangeState, f'{target}/change_state')

    def _call(self, client, request):
        """Calls a service, returning None instead of blocking forever.

        None is never an error here; callers just re-poll.

        Args:
            client: Service client.
            request: Request to send.

        Returns:
            The response, or None if the service or its response timed out.
        """
        if not client.wait_for_service(timeout_sec=_CALL_TIMEOUT):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=_CALL_TIMEOUT)
        return future.result() if future.done() else None

    def current_state(self):
        """Returns the target's `(state_id, label)`, or `(None, 'unreachable')`."""
        response = self._call(self._get, GetState.Request())
        if response is None:
            return None, 'unreachable'
        return response.current_state.id, response.current_state.label

    def request(self, transition_id):
        """Requests a lifecycle transition; the result is ignored (we re-poll)."""
        req = ChangeState.Request()
        req.transition.id = transition_id
        self._call(self._change, req)

    def run(self):
        """Polls until `active`, nudging the node forward.

        Returns:
            True if the node became active before the deadline.
        """
        last_label = None
        while rclpy.ok() and self.get_clock().now().nanoseconds < self._deadline:
            state_id, label = self.current_state()
            if label != last_label:
                self.get_logger().info(f'{self._target} state: {label}')
                last_label = label

            if state_id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(
                    f'{self._target} is ACTIVE - scans are being processed.')
                return True
            if state_id in _ADVANCE:
                self.request(_ADVANCE[state_id])
            elif state_id is not None and state_id not in _IN_FLIGHT:
                self.get_logger().warn(
                    f'{self._target} in unexpected state {label}; still waiting.')

            self._sleep(_POLL_PERIOD)

        _, label = self.current_state()
        self.get_logger().error(
            f'TIMED OUT waiting for {self._target} to reach "active" - it is '
            f'"{label}". NO SCANS ARE BEING PROCESSED and any map saved from '
            f'this run will be empty or partial. Do not trust this run.')
        return False

    def _sleep(self, seconds):
        """Spins for `seconds` so responses that do arrive are handled."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    """Runs the watchdog and exits 0 if the node became active, else 1."""
    target = sys.argv[1] if len(sys.argv) > 1 else '/slam_toolbox'
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

    rclpy.init()
    # Wall-clock deadline so it still times out if /clock stalls.
    node = LifecycleWatchdog(target, 0)
    node._deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Drive and verify a slam_toolbox lifecycle node's transition to `active`.

Why this exists: slam_toolbox's `*_slam_toolbox_node` is a managed lifecycle
node that processes no scans at all until it is CONFIGUREd and ACTIVATEd. In
this environment its `change_state` service response is regularly lost -
the node logs "failed to send response to /slam_toolbox/change_state
(timeout)" - so anything that waits on that response, or on the state-change
event that launch derives from it, can hang forever while the node sits in
`inactive` doing nothing. That failure is silent: the launch looks healthy,
`/map` is simply never published, and the problem only surfaces when a map
save comes back empty.

This watchdog is immune to the lost response because it never trusts one. It
polls `get_state` as the single source of truth and re-issues whatever
transition the observed state calls for, so a dropped request or a dropped
response just means the next poll retries. It also makes failure loud: if the
node has not reached `active` before the deadline it logs an ERROR naming the
state it got stuck in and exits non-zero, instead of leaving the run to
silently collect nothing.

Exit code 0 means the node was observed in `active`. Any non-zero exit means
mapping is NOT running and the map that results will be empty or partial.
"""

import sys

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

#: States we can act on, mapped to the transition that moves them forward.
_ADVANCE = {
    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
}

#: Transitional states - the node is already working, just wait and re-poll.
_IN_FLIGHT = {
    State.TRANSITION_STATE_CONFIGURING,
    State.TRANSITION_STATE_ACTIVATING,
    State.TRANSITION_STATE_CLEANINGUP,
    State.TRANSITION_STATE_DEACTIVATING,
}

_CALL_TIMEOUT = 5.0  # per service call; a lost response must not hang us
_POLL_PERIOD = 1.0


class LifecycleWatchdog(Node):
    def __init__(self, target, deadline):
        super().__init__('slam_lifecycle_watchdog')
        self._target = target
        self._deadline = deadline
        self._get = self.create_client(GetState, f'{target}/get_state')
        self._change = self.create_client(ChangeState, f'{target}/change_state')

    def _call(self, client, request):
        """Call a service, returning None rather than blocking forever.

        A None result is not treated as an error anywhere in this file: the
        caller simply re-polls. That is the whole point - this environment
        drops `change_state` responses, and a watchdog that waited on them
        would reproduce the bug it exists to catch.
        """
        if not client.wait_for_service(timeout_sec=_CALL_TIMEOUT):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=_CALL_TIMEOUT)
        return future.result() if future.done() else None

    def current_state(self):
        response = self._call(self._get, GetState.Request())
        if response is None:
            return None, 'unreachable'
        return response.current_state.id, response.current_state.label

    def request(self, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        self._call(self._change, req)  # result deliberately ignored; we re-poll

    def run(self):
        """Poll until `active`, nudging the node forward. True iff active."""
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
        # spin while waiting so service responses that do arrive get handled
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else '/slam_toolbox'
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

    rclpy.init()
    # Deliberately the steady system clock: this watches a node's lifecycle,
    # which happens in wall-clock time and must still time out if /clock stalls
    # or never starts.
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

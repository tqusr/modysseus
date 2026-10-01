import time

import py_trees
import rclpy
from rclpy.executors import SingleThreadedExecutor

from mission.docks import Dock
from mission.mission_tree import (
    BB_CURRENT_POSITION,
    BB_DESTINATION,
    BB_ROUTE_ID,
    BB_SOURCE,
    build_mission_tree,
)
from mission.stub_action_servers import StubActionServers


class _FakeSagaApi:
    """Serves one route, then a different one, like the real server does
    once a delivery completes."""

    def __init__(self):
        self._routes = [
            {"route_id": 1, "source": {"letter": "A", "tag_id": 0},
             "destination": {"letter": "B", "tag_id": 1}, "status": "loading_pending"},
            {"route_id": 2, "source": {"letter": "C", "tag_id": 2},
             "destination": {"letter": "D", "tag_id": 3}, "status": "loading_pending"},
        ]
        self.route_index = 0

    def get_route(self):
        return self._routes[self.route_index]

    def get_status(self):
        return {"status": self._routes[self.route_index]["status"]}

    def set_status(self, status):
        self._routes[self.route_index]["status"] = status

    def advance_to_next_route(self):
        self.route_index = min(self.route_index + 1, len(self._routes) - 1)
        self._routes[self.route_index]["status"] = "loading_pending"


def _docks():
    return {
        letter: Dock(letter=letter, tag_id=i,
                     dock_pose={"x": 0.0, "y": 0.0, "theta": 0.0},
                     staging_pose={"x": 0.0, "y": 0.0, "theta": 0.0})
        for i, letter in enumerate("ABCD")
    }


def test_mission_tree_completes_a_route_cycle():
    rclpy.init()
    try:
        action_server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_mission_tree")
        executor = SingleThreadedExecutor()
        executor.add_node(action_server)
        executor.add_node(client_node)

        saga = _FakeSagaApi()
        root = build_mission_tree(client_node, saga, _docks())
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(node=client_node, timeout=5.0)

        observer = py_trees.blackboard.Client(name="observer")
        observer.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)
        observer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.READ)
        observer.register_key(key=BB_SOURCE, access=py_trees.common.Access.READ)
        observer.register_key(key=BB_DESTINATION, access=py_trees.common.Access.READ)

        # Two full routes cross several real-time poll_sec=1.0 WaitUntil gates
        # (DockIsFree x2, SagaLoaded, SagaUnloaded) using the real wall clock,
        # not a fake one -- give it real margin.
        deadline = time.monotonic() + 40.0
        route_ids_seen = set()
        route_id_existed = False
        while time.monotonic() < deadline and len(route_ids_seen) < 2:
            executor.spin_once(timeout_sec=0.05)
            tree.tick()

            route_id_exists_now = observer.exists(BB_ROUTE_ID)
            if route_id_exists_now:
                route_ids_seen.add(observer.get(BB_ROUTE_ID))
            # ClearRoute unsets BB_ROUTE_ID within the same tick it completes a
            # route; GetRouteFromSaga only repopulates it on a LATER tick, so
            # this existed->not-existed transition is a reliable, non-transient
            # signal that a route just finished (unlike peeking at
            # BB_PACKAGE_DELIVERED, which gets reset in the same tick).
            if route_id_existed and not route_id_exists_now:
                saga.advance_to_next_route()
            route_id_existed = route_id_exists_now

            # Drive the fake Saga status forward based on where the robot has
            # reported itself, mirroring an organizer observing arrival and
            # pressing the real admin-page buttons.
            if observer.exists(BB_CURRENT_POSITION) and observer.exists(BB_SOURCE) and \
                    observer.get(BB_CURRENT_POSITION) == f"dock:{observer.get(BB_SOURCE)}":
                saga.set_status("loading_complete")
            if observer.exists(BB_CURRENT_POSITION) and observer.exists(BB_DESTINATION) and \
                    observer.get(BB_CURRENT_POSITION) == f"dock:{observer.get(BB_DESTINATION)}":
                saga.set_status("unloading_complete")

            time.sleep(0.05)

        assert route_ids_seen == {1, 2}, f"expected to see both routes, saw {route_ids_seen}"
    finally:
        rclpy.shutdown()

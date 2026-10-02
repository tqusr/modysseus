import time

import py_trees
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

from mission.docks import Dock
from mission.mission_tree import (
    BB_CURRENT_POSITION,
    BB_DESTINATION,
    BB_ROUTE_ID,
    BB_SOURCE,
    LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE,
    Localize,
    VehicleStatusOK,
    build_mission_tree,
)
from mission.stub_action_servers import StubActionServers


def _serve_lifecycle_manager_is_active(node):
    """Fakes nav2_lifecycle_manager's is_active service so VehicleStatusOK's
    real check has something to poll -- this test has no real Nav2 stack."""
    return node.create_service(
        Trigger, LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE,
        lambda request, response: _trigger_success(response))


def _trigger_success(response):
    response.success = True
    return response


def _broadcast_fake_localization(broadcaster, node):
    """Fakes slam_toolbox's map->odom plus the sim's odom->base_footprint,
    both re-stamped with "now" on every call -- VehicleStatusOK/Localize's
    real TF recency checks need a live-looking broadcaster, not a one-shot
    static transform that goes stale after MAX_TF_AGE_SEC."""
    now = node.get_clock().now().to_msg()
    transforms = []
    for parent, child in (('map', 'odom'), ('odom', 'base_footprint')):
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    broadcaster.sendTransform(transforms)


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

    def get_route_by_id(self, route_id):
        for route in self._routes:
            if route["route_id"] == route_id:
                return route
        raise ValueError(f"no such route {route_id}")

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

        # VehicleStatusOK/Localize (Task 7) now do real checks -- fake their
        # two dependencies (nav2_lifecycle_manager's is_active service and a
        # live map->odom->base_footprint TF chain) the same way StubActionServers
        # fakes NavigateToPose/DockRobot above, so this test still exercises the
        # saga/route logic without needing the full real stack running.
        lifecycle_service = _serve_lifecycle_manager_is_active(client_node)
        tf_broadcaster = tf2_ros.TransformBroadcaster(client_node)

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
            _broadcast_fake_localization(tf_broadcaster, client_node)
            # Drain, don't just sample once: a single spin_once(timeout_sec=0.05)
            # processes at most ONE ready callback per call (that's spin_once's
            # own contract), not everything that's ready. The real MissionNode
            # drains continuously via rclpy.spin(), but this test now has several
            # concurrent ROS streams in flight at once (TF broadcasts for Task 7's
            # VehicleStatusOK/Localize, the fake lifecycle-manager Trigger polls,
            # plus the existing action goal/result futures) -- sampling only one
            # per iteration let a backlog build up (TF lookups kept returning an
            # ever-staler "latest" transform, eventually older than
            # MAX_TF_AGE_SEC), which read as a real localization dropout and
            # wrongly tripped the safe-stop branch mid-route.
            inner_deadline = time.monotonic() + 0.05
            while time.monotonic() < inner_deadline:
                executor.spin_once(timeout_sec=0.0)
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


def _spin_and_drain(executor, duration_sec):
    """Spin for duration_sec, processing every ready callback along the way
    (not just one per call -- see test_mission_tree_completes_a_route_cycle's
    own comment on why a plain single spin_once() call isn't enough once
    there's more than one concurrent ROS stream in play)."""
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.0)


def test_vehicle_status_ok_fails_with_no_lifecycle_manager_and_no_tf():
    """Neither of VehicleStatusOK's two dependencies exist in this bare test
    node -- it must FAILURE, not default to SUCCESS just because nothing
    actively errored."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_vsok_nothing_live")
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
        behaviour = VehicleStatusOK("VehicleStatusOK", node, tf_buffer, tf_listener)

        assert behaviour.update() == py_trees.common.Status.FAILURE
    finally:
        rclpy.shutdown()


def test_vehicle_status_ok_fails_on_tf_alone_even_if_lifecycle_manager_is_active():
    """The lifecycle-manager half can be confirmed active while the TF half
    is still missing -- both must hold for SUCCESS, so this must FAILURE."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_vsok_tf_missing")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        _serve_lifecycle_manager_is_active(node)

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
        behaviour = VehicleStatusOK("VehicleStatusOK", node, tf_buffer, tf_listener)

        # Poll until the (fake) lifecycle manager is confirmed active -- no TF
        # broadcaster is ever started in this test, so that half never will be.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not behaviour._last_known_active:
            _spin_and_drain(executor, 0.05)
            behaviour.update()
        assert behaviour._last_known_active is True, "lifecycle-manager fake never confirmed active"

        assert behaviour.update() == py_trees.common.Status.FAILURE
    finally:
        rclpy.shutdown()


def test_vehicle_status_ok_succeeds_once_both_dependencies_are_live():
    rclpy.init()
    try:
        node = rclpy.create_node("test_vsok_all_live")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        _serve_lifecycle_manager_is_active(node)
        tf_broadcaster = tf2_ros.TransformBroadcaster(node)

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
        behaviour = VehicleStatusOK("VehicleStatusOK", node, tf_buffer, tf_listener)

        deadline = time.monotonic() + 5.0
        status = py_trees.common.Status.FAILURE
        while time.monotonic() < deadline and status != py_trees.common.Status.SUCCESS:
            _broadcast_fake_localization(tf_broadcaster, node)
            _spin_and_drain(executor, 0.05)
            status = behaviour.update()

        assert status == py_trees.common.Status.SUCCESS
    finally:
        rclpy.shutdown()


def test_localize_fails_with_no_tf_published():
    rclpy.init()
    try:
        node = rclpy.create_node("test_localize_no_tf")
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
        behaviour = Localize("Localize", node, tf_buffer)

        assert behaviour.update() == py_trees.common.Status.FAILURE
    finally:
        rclpy.shutdown()


def test_localize_succeeds_once_map_to_base_footprint_is_live():
    rclpy.init()
    try:
        node = rclpy.create_node("test_localize_live")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        tf_broadcaster = tf2_ros.TransformBroadcaster(node)

        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
        behaviour = Localize("Localize", node, tf_buffer)

        deadline = time.monotonic() + 5.0
        status = py_trees.common.Status.FAILURE
        while time.monotonic() < deadline and status != py_trees.common.Status.SUCCESS:
            _broadcast_fake_localization(tf_broadcaster, node)
            _spin_and_drain(executor, 0.05)
            status = behaviour.update()

        assert status == py_trees.common.Status.SUCCESS
    finally:
        rclpy.shutdown()

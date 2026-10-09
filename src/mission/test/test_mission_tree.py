import time

import py_trees
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import UndockRobot
from rclpy.action import ActionServer
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from std_srvs.srv import Trigger

from mission.docks import Dock
from mission.mission_tree import (
    BB_CURRENT_POSITION,
    BB_DESTINATION,
    BB_ROUTE_ID,
    BB_SOURCE,
    DOCK_PLUGIN_TYPE,
    DOCK_PROXIMITY_TOLERANCE_M,
    FUTURE_TF_TOLERANCE_SEC,
    LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE,
    MAX_TF_AGE_SEC,
    Localize,
    VehicleStatusOK,
    build_ensure_undocked,
    build_mission_tree,
)
from mission.stub_action_servers import StubActionServers


def _serve_lifecycle_manager_is_active(node, success=True, call_log=None):
    """Fakes nav2_lifecycle_manager's is_active service for VehicleStatusOK to poll.

    `success=False` fakes a reachable but unhealthy manager (a managed node not
    active), as distinct from the service being absent. `call_log`, if given, is
    appended to on every request, so a test can confirm the fake was reached
    before asserting on its effect ("never polled" and "reported False" both
    read as "not confirmed active")."""
    def handle(request, response):
        if call_log is not None:
            call_log.append(True)
        response.success = success
        return response

    return node.create_service(Trigger, LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE, handle)


def _stamped_transform(stamp_msg, parent, child):
    transform = TransformStamped()
    transform.header.stamp = stamp_msg
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.rotation.w = 1.0
    return transform


def _stamped_transform_at(stamp_msg, parent, child, x, y):
    """Like _stamped_transform, but at an explicit (x, y) translation.

    Needed by the UndockRobot TF tests, where the position relative to specific
    docks is what is under test (the implicit origin would coincide with
    whichever dock sits there)."""
    transform = _stamped_transform(stamp_msg, parent, child)
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    return transform


def _broadcast_fake_localization(broadcaster, node):
    """Fakes map->odom and odom->base_footprint, re-stamped with "now" on every call.

    The real TF recency checks need a live-looking broadcaster, not a one-shot
    static transform that goes stale after MAX_TF_AGE_SEC."""
    now = node.get_clock().now().to_msg()
    transforms = [_stamped_transform(now, parent, child)
                  for parent, child in (('map', 'odom'), ('odom', 'base_footprint'))]
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


def _dock_pose_docks():
    """Docks at well-separated poses, unlike _docks() (all at (0, 0, 0)).

    The UndockRobot TF tests need a position to be unambiguously "at dock B" or
    "nowhere near any dock"."""
    return {
        letter: Dock(letter=letter, tag_id=i,
                     dock_pose={"x": float(i * 10), "y": float(i * 10), "theta": 0.0},
                     staging_pose={"x": float(i * 10) - 1.0, "y": float(i * 10), "theta": 0.0})
        for i, letter in enumerate("ABCD")
    }


class _CountingUndockServer:
    """Lightweight fake for just the `undock_robot` action.

    Always succeeds at once and counts the goals it received. It is not the full
    StubActionServers, whose navigate_to_pose and dock_robot would be noise: the
    point is to prove build_ensure_undocked's gating fires, or not, on its own."""

    def __init__(self, node):
        self.goal_count = 0
        self.dock_types = []
        self._server = ActionServer(node, UndockRobot, 'undock_robot', self._execute)

    def _execute(self, goal_handle):
        self.goal_count += 1
        self.dock_types.append(goal_handle.request.dock_type)
        goal_handle.succeed()
        return UndockRobot.Result()


def _tick_until_settled(tree, executor, deadline_sec=5.0):
    """Ticks `tree` (a build_ensure_undocked subtree) until it settles or times out.

    Drains the executor between ticks so CallAction's futures can resolve, and
    stops when the root is SUCCESS/FAILURE (not RUNNING) or `deadline_sec`
    elapses."""
    deadline = time.monotonic() + deadline_sec
    status = py_trees.common.Status.RUNNING
    while time.monotonic() < deadline:
        _spin_and_drain(executor, 0.05)
        tree.tick()
        status = tree.root.status
        if status != py_trees.common.Status.RUNNING:
            return status
        time.sleep(0.02)
    return status


def test_mission_tree_completes_a_route_cycle():
    rclpy.init()
    try:
        action_server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_mission_tree")
        executor = SingleThreadedExecutor()
        executor.add_node(action_server)
        executor.add_node(client_node)

        # VehicleStatusOK and Localize do real checks, so fake their dependencies
        # (the is_active service and a live TF chain) as StubActionServers fakes
        # the actions, keeping this test on the saga/route logic.
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

        # Two routes cross several real-time poll_sec=1.0 WaitUntil gates
        # (DockIsFree x2, SagaLoaded, SagaUnloaded) on the wall clock, so allow
        # real margin. It also runs UndockRobot three times (A->B, B->C, C->D).
        deadline = time.monotonic() + 40.0
        route_ids_seen = set()
        route_id_existed = False
        while time.monotonic() < deadline and len(route_ids_seen) < 2:
            _broadcast_fake_localization(tf_broadcaster, client_node)
            # Drain, don't sample once: spin_once(timeout_sec=0.05) processes at
            # most one ready callback. Several ROS streams run at once here (TF,
            # the fake Trigger polls, action futures), and sampling one per
            # iteration let a backlog build until "latest" TF exceeded
            # MAX_TF_AGE_SEC, which read as a localization dropout and failed the
            # health check mid-route.
            inner_deadline = time.monotonic() + 0.05
            while time.monotonic() < inner_deadline:
                executor.spin_once(timeout_sec=0.0)
            tree.tick()

            route_id_exists_now = observer.exists(BB_ROUTE_ID)
            if route_id_exists_now:
                route_ids_seen.add(observer.get(BB_ROUTE_ID))
            # ClearRoute unsets BB_ROUTE_ID in the tick a route completes and
            # GetRouteFromSaga repopulates it on a later tick, so this existed to
            # not-existed transition reliably marks a finished route (unlike
            # BB_PACKAGE_DELIVERED, which resets in the same tick).
            if route_id_existed and not route_id_exists_now:
                saga.advance_to_next_route()
            route_id_existed = route_id_exists_now

            # Advance the fake Saga status from where the robot says it is,
            # mirroring an organizer pressing the admin-page buttons.
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
    """Spins for `duration_sec`, processing every ready callback.

    A single spin_once() is not enough with several concurrent ROS streams (see
    the comment in test_mission_tree_completes_a_route_cycle)."""
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.0)


def test_vehicle_status_ok_fails_with_no_lifecycle_manager_and_no_tf():
    """With neither dependency present, VehicleStatusOK must FAILURE, not default to SUCCESS."""
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


def test_vehicle_status_ok_fails_when_lifecycle_manager_reports_not_active():
    """The service is up but reports success=False, as if a managed node were inactive.

    Distinct from "service absent". TF is fresh throughout, isolating the
    lifecycle-manager half of the check."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_vsok_lifecycle_reports_false")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        call_log = []
        _serve_lifecycle_manager_is_active(node, success=False, call_log=call_log)

        tf_buffer = tf2_ros.Buffer()
        tf_buffer.set_transform(
            _stamped_transform(node.get_clock().now().to_msg(), 'map', 'odom'), "test")

        behaviour = VehicleStatusOK("VehicleStatusOK", node, tf_buffer, tf_listener=None)

        # Wait on call_log: "not yet polled" and "polled, reported False" look
        # the same from the behaviour's own state.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not call_log:
            _spin_and_drain(executor, 0.05)
            behaviour.update()
        assert call_log, "fake lifecycle-manager service was never actually reached"

        assert behaviour.update() == py_trees.common.Status.FAILURE
    finally:
        rclpy.shutdown()


def test_localize_fails_when_tf_is_present_but_too_stale_in_the_past():
    """A transform that resolves but whose stamp is older than MAX_TF_AGE_SEC.

    Distinct from "no transform published". Inserted with Buffer.set_transform
    (what TransformListener calls) for a deterministic stamp with no spin-timing
    dependency."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_localize_stale_past")
        tf_buffer = tf2_ros.Buffer()
        stale_stamp = (node.get_clock().now() - Duration(seconds=MAX_TF_AGE_SEC + 1.0)).to_msg()
        tf_buffer.set_transform(_stamped_transform(stale_stamp, 'map', 'base_footprint'), "test")

        behaviour = Localize("Localize", node, tf_buffer)

        assert behaviour.update() == py_trees.common.Status.FAILURE
    finally:
        rclpy.shutdown()


def test_localize_fails_when_tf_is_stamped_too_far_in_the_future():
    """The mirror case: the future-side ceiling.

    slam_toolbox's ~0.9 s look-ahead is tolerated (see
    test_localize_succeeds_once_map_to_base_footprint_is_live and
    _transform_is_recent), but a stamp beyond FUTURE_TF_TOLERANCE_SEC, as a clock
    domain mismatch upstream would produce, must still FAILURE."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_localize_stale_future")
        tf_buffer = tf2_ros.Buffer()
        future_stamp = (node.get_clock().now() + Duration(seconds=FUTURE_TF_TOLERANCE_SEC + 1.0)).to_msg()
        tf_buffer.set_transform(_stamped_transform(future_stamp, 'map', 'base_footprint'), "test")

        behaviour = Localize("Localize", node, tf_buffer)

        assert behaviour.update() == py_trees.common.Status.FAILURE
    finally:
        rclpy.shutdown()


def test_ensure_undocked_fires_when_blackboard_says_docked():
    """The fast path: BB_CURRENT_POSITION == "dock:<letter>" alone triggers UndockRobot.

    TF is placed far from every dock to isolate the blackboard signal, and a
    successful undock clears BB_CURRENT_POSITION."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_undock_bb_docked")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        undock_server = _CountingUndockServer(node)

        docks = _dock_pose_docks()
        tf_buffer = tf2_ros.Buffer()
        now = node.get_clock().now().to_msg()
        tf_buffer.set_transform(
            _stamped_transform_at(now, 'map', 'base_footprint', 1000.0, 1000.0), "test")

        position_writer = py_trees.blackboard.Client(name="test-position-writer-a")
        position_writer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.WRITE)
        position_writer.set(BB_CURRENT_POSITION, "dock:A")

        root = build_ensure_undocked(node, tf_buffer, docks, position_writer)
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(node=node, timeout=5.0)

        status = _tick_until_settled(tree, executor)

        assert undock_server.goal_count == 1, "UndockRobot should have fired exactly once"
        assert status == py_trees.common.Status.SUCCESS
        assert not position_writer.exists(BB_CURRENT_POSITION), \
            "a successful UndockRobot should have cleared BB_CURRENT_POSITION"
        # An empty dock_type makes the server use its stashed state, which is
        # gone after a docking_server restart (see undock_goal()).
        assert undock_server.dock_types == [DOCK_PLUGIN_TYPE], \
            "UndockRobot goal must name the dock plugin explicitly"
    finally:
        rclpy.shutdown()


def test_ensure_undocked_fires_from_tf_ground_truth_when_blackboard_is_unset():
    """Restart recovery: the blackboard is unset but TF places the robot at a dock.

    This is the state right after mission_node restarts. The TF ground-truth
    signal must still trigger UndockRobot; the blackboard alone would skip it and
    bring back the planner deadlock."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_undock_tf_ground_truth")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        undock_server = _CountingUndockServer(node)

        docks = _dock_pose_docks()
        dock_b = docks["B"]
        tf_buffer = tf2_ros.Buffer()
        now = node.get_clock().now().to_msg()
        tf_buffer.set_transform(
            _stamped_transform_at(now, 'map', 'base_footprint',
                                   dock_b.dock_pose["x"], dock_b.dock_pose["y"]),
            "test")

        position_writer = py_trees.blackboard.Client(name="test-position-writer-b")
        position_writer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.WRITE)
        # Unset explicitly: py_trees' blackboard storage is process-wide, shared
        # across all Clients, so an earlier test (e.g. the full-route test) can
        # leave this set, making isolation depend on pytest's order.
        if position_writer.exists(BB_CURRENT_POSITION):
            position_writer.unset(BB_CURRENT_POSITION)
        # Left unset from here on -- simulates the mission_node restart.

        root = build_ensure_undocked(node, tf_buffer, docks, position_writer)
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(node=node, timeout=5.0)

        status = _tick_until_settled(tree, executor)

        assert undock_server.goal_count == 1, \
            "TF ground truth alone should have triggered UndockRobot despite an unset blackboard"
        assert status == py_trees.common.Status.SUCCESS
    finally:
        rclpy.shutdown()


def test_ensure_undocked_does_not_fire_when_neither_signal_says_docked():
    """The common case must not regress: a robot not docked anywhere makes no undock call.

    Neither the blackboard nor TF says "docked" (e.g. the first leg of a fresh
    mission), so the tree resolves at once with no UndockRobot goal at all.
    Confirms the gate is a real gate, not one that fires unconditionally."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_undock_neither_signal")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        undock_server = _CountingUndockServer(node)

        docks = _dock_pose_docks()
        tf_buffer = tf2_ros.Buffer()
        now = node.get_clock().now().to_msg()
        tf_buffer.set_transform(
            _stamped_transform_at(now, 'map', 'base_footprint', 1000.0, 1000.0), "test")

        position_writer = py_trees.blackboard.Client(name="test-position-writer-c")
        position_writer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.WRITE)
        # Unset explicitly (see test_ensure_undocked_fires_from_tf_ground_truth_
        # when_blackboard_is_unset); left unset like a fresh mission.
        if position_writer.exists(BB_CURRENT_POSITION):
            position_writer.unset(BB_CURRENT_POSITION)

        root = build_ensure_undocked(node, tf_buffer, docks, position_writer)
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(node=node, timeout=5.0)

        status = _tick_until_settled(tree, executor, deadline_sec=2.0)

        assert undock_server.goal_count == 0, \
            "UndockRobot must NOT be called when neither signal indicates docked"
        assert status == py_trees.common.Status.SUCCESS
    finally:
        rclpy.shutdown()


def test_vehicle_status_ok_retries_a_poll_whose_response_never_arrives():
    """A dropped service response must not wedge VehicleStatusOK.

    Responses do get dropped under load (Nav2's lifecycle manager logs it) and
    leave the is_active future pending forever. Without a bound, VehicleStatusOK
    would return its latched False for good with a healthy Nav2. The poll must
    time out and be re-sent.

    The client is faked, not the server: rclpy's service plumbing refuses to "not
    answer" (a callback returning None raises), and only the client-side bound is
    under test."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_vsok_poll_timeout")

        class _NeverAnsweringClient:
            srv_name = LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE

            def __init__(self):
                self.requests = []
                self.abandoned = []

            def service_is_ready(self):
                return True

            def call_async(self, request):
                future = Future()  # never resolved: the response was "lost"
                self.requests.append(future)
                return future

            def remove_pending_request(self, future):
                self.abandoned.append(future)

        # Tiny timeout so the test doesn't have to wait the production 30s.
        behaviour = VehicleStatusOK("VehicleStatusOK", node, tf2_ros.Buffer(),
                                     tf_listener=None, poll_timeout_sec=0.2)
        client = _NeverAnsweringClient()
        behaviour._client = client

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and len(client.requests) < 2:
            behaviour.update()
            time.sleep(0.05)

        assert len(client.requests) >= 2, \
            ("VehicleStatusOK never re-polled: one lost response would wedge it "
             f"failing forever (requests seen: {len(client.requests)})")
        assert client.abandoned, \
            "the timed-out request must be removed from the client's pending map"
        assert behaviour.update() == py_trees.common.Status.FAILURE, \
            "an unanswered lifecycle-manager poll must still read as NOT ok"
    finally:
        rclpy.shutdown()


def test_ensure_undocked_proximity_tolerance_is_not_absurdly_large():
    """Sanity bound on DOCK_PROXIMITY_TOLERANCE_M itself.

    A position midway between two docks 10 m apart must not read as "docked" via
    TF, which guards against the constant becoming so large that it stops
    telling "docked" from "somewhere in the arena"."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_undock_tolerance_sanity")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        undock_server = _CountingUndockServer(node)

        docks = _dock_pose_docks()  # A=(0,0), B=(10,10), ...
        tf_buffer = tf2_ros.Buffer()
        now = node.get_clock().now().to_msg()
        # Well beyond DOCK_PROXIMITY_TOLERANCE_M from every dock but inside the
        # arena's range, unlike the other tests' absurd (1000, 1000).
        midpoint = 5.0
        assert midpoint > DOCK_PROXIMITY_TOLERANCE_M * 5, \
            "test fixture assumption broken: tighten the midpoint or the docks spacing"
        tf_buffer.set_transform(
            _stamped_transform_at(now, 'map', 'base_footprint', midpoint, midpoint), "test")

        position_writer = py_trees.blackboard.Client(name="test-position-writer-d")
        position_writer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.WRITE)
        # Unset explicitly (see test_ensure_undocked_fires_from_tf_ground_truth_
        # when_blackboard_is_unset).
        if position_writer.exists(BB_CURRENT_POSITION):
            position_writer.unset(BB_CURRENT_POSITION)

        root = build_ensure_undocked(node, tf_buffer, docks, position_writer)
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(node=node, timeout=5.0)

        status = _tick_until_settled(tree, executor, deadline_sec=2.0)

        assert undock_server.goal_count == 0, \
            "a position merely between two docks must not read as docked"
        assert status == py_trees.common.Status.SUCCESS
    finally:
        rclpy.shutdown()


def test_ensure_undocked_ignores_a_robot_standing_beside_a_dock():
    """Within the proximity radius of a dock's wall pose, but off to its side.

    A docked robot sits straight out from the wall, a few centimetres off the
    dock's axis. A robot 0.2 m along the wall from it (as happened beside dock F
    when navigation left it jammed there) is not docked, and asking
    docking_server to undock it is refused forever ("Robot is not in the dock").
    """
    rclpy.init()
    try:
        node = rclpy.create_node("test_undock_beside_dock")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        undock_server = _CountingUndockServer(node)

        docks = _dock_pose_docks()          # B at (10, 10), outward normal +x
        dock_b = docks["B"].dock_pose
        tf_buffer = tf2_ros.Buffer()
        now = node.get_clock().now().to_msg()
        tf_buffer.set_transform(
            _stamped_transform_at(now, 'map', 'base_footprint',
                                   dock_b["x"] + 0.05, dock_b["y"] + 0.2), "test")

        position_writer = py_trees.blackboard.Client(name="test-position-writer-e")
        position_writer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.WRITE)
        if position_writer.exists(BB_CURRENT_POSITION):
            position_writer.unset(BB_CURRENT_POSITION)

        root = build_ensure_undocked(node, tf_buffer, docks, position_writer)
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(node=node, timeout=5.0)

        status = _tick_until_settled(tree, executor, deadline_sec=2.0)

        assert undock_server.goal_count == 0, \
            "a robot 0.2 m beside a dock is not docked; undocking it can never succeed"
        assert status == py_trees.common.Status.SUCCESS
    finally:
        rclpy.shutdown()

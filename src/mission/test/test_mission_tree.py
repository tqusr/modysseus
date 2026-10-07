import time

import py_trees
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.action import UndockRobot
from rclpy.action import ActionServer
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from std_srvs.srv import Trigger

from mission import mission_tree as mission_tree_module
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
    StopRobot,
    VehicleStatusOK,
    build_ensure_undocked,
    build_mission_tree,
)
from mission.stub_action_servers import StubActionServers


def _serve_lifecycle_manager_is_active(node, success=True, call_log=None):
    """Fakes nav2_lifecycle_manager's is_active service so VehicleStatusOK's
    real check has something to poll -- this test has no real Nav2 stack.

    `success=False` fakes a reachable-but-unhealthy lifecycle manager (some
    managed node not active), as distinct from the service being absent
    altogether. `call_log`, when given, is appended to on every request so a
    test can confirm the fake was actually reached at least once before
    asserting on its effect (success=False and "never polled yet" both read
    as "not confirmed active" from VehicleStatusOK's own state, so that
    alone can't distinguish the two without this)."""
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
    """Like _stamped_transform, but at an explicit (x, y) translation --
    needed for the UndockRobot TF-ground-truth tests below, where the
    robot's position relative to specific dock poses is exactly what's
    under test (unlike _stamped_transform's implicit (0, 0, 0), which
    would silently coincide with whatever dock sits at the origin)."""
    transform = _stamped_transform(stamp_msg, parent, child)
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    return transform


def _broadcast_fake_localization(broadcaster, node):
    """Fakes slam_toolbox's map->odom plus the sim's odom->base_footprint,
    both re-stamped with "now" on every call -- VehicleStatusOK/Localize's
    real TF recency checks need a live-looking broadcaster, not a one-shot
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
    """Unlike _docks() above (every letter at the same (0,0,0), fine for
    tests that don't care about specific dock geometry), the UndockRobot
    ground-truth tests below need well-separated dock poses so a TF
    position can be placed unambiguously "at dock B" vs "nowhere near any
    dock"."""
    return {
        letter: Dock(letter=letter, tag_id=i,
                     dock_pose={"x": float(i * 10), "y": float(i * 10), "theta": 0.0},
                     staging_pose={"x": float(i * 10) - 1.0, "y": float(i * 10), "theta": 0.0})
        for i, letter in enumerate("ABCD")
    }


class _CountingUndockServer:
    """Lightweight fake for just the `undock_robot` action -- always
    succeeds immediately, counting how many goals it actually received.
    Deliberately NOT the full StubActionServers (which also stands up
    navigate_to_pose/dock_robot, irrelevant noise for these focused gating
    tests -- see Task 8 review finding #2: the point here is to prove
    _not_docked_anywhere/build_ensure_undocked's own gating logic fires (or
    doesn't) correctly, independent of the rest of the dock/navigate
    subtree)."""

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
    """Ticks `tree` (wrapping a build_ensure_undocked subtree), draining the
    executor between ticks so CallAction's async action futures actually
    get a chance to resolve, until the root settles on SUCCESS/FAILURE (not
    RUNNING) or deadline_sec elapses."""
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
        # not a fake one -- give it real margin. Also exercises UndockRobot
        # (Task 8) three times (A->B, B->C across the route boundary, C->D),
        # each a cheap extra CallAction round trip against the stub.
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


def test_vehicle_status_ok_fails_when_lifecycle_manager_reports_not_active():
    """Distinct from "service absent" (covered above): the service is up and
    reachable, but reports success=False, as it would if some real managed
    navigation node were not active. TF is live/fresh throughout, isolating
    this to the lifecycle-manager half of the check."""
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

        # Poll until the fake service has actually been reached at least once --
        # "not yet polled" and "polled, reported False" both look like "not
        # confirmed active" from the behaviour's own state alone, so wait for
        # call_log instead of trusting behaviour.update()'s result here.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not call_log:
            _spin_and_drain(executor, 0.05)
            behaviour.update()
        assert call_log, "fake lifecycle-manager service was never actually reached"

        assert behaviour.update() == py_trees.common.Status.FAILURE
    finally:
        rclpy.shutdown()


def test_localize_fails_when_tf_is_present_but_too_stale_in_the_past():
    """A transform that actually resolves, but whose own stamp is older than
    MAX_TF_AGE_SEC -- distinct from the "no transform published at all" case
    covered above. Inserted directly via Buffer.set_transform (what
    TransformListener itself calls on message receipt) rather than via a
    live broadcaster, for a deterministic stamp with no spin-timing
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
    """The mirror case of the above, exercising the new future-side ceiling:
    slam_toolbox's own ~0.9s look-ahead is tolerated (see
    test_localize_succeeds_once_map_to_base_footprint_is_live and
    _transform_is_recent's docstring), but a stamp further in the future
    than FUTURE_TF_TOLERANCE_SEC -- e.g. what a clock-domain mismatch
    somewhere upstream (a node missing use_sim_time:true) would produce --
    must still FAILURE, not be waved through as "extra fresh"."""
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
    """The normal, fast path: BB_CURRENT_POSITION == "dock:<letter>" alone
    (TF placed far from every known dock, isolating this to the blackboard
    signal) is enough to trigger a real UndockRobot call, and a successful
    one clears BB_CURRENT_POSITION afterward."""
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
        # An empty dock_type makes the server fall back to its own stashed
        # state, which is gone after a docking_server restart -- see
        # build_ensure_undocked's undock_goal().
        assert undock_server.dock_types == [DOCK_PLUGIN_TYPE], \
            "UndockRobot goal must name the dock plugin explicitly"
    finally:
        rclpy.shutdown()


def test_ensure_undocked_fires_from_tf_ground_truth_when_blackboard_is_unset():
    """Task 8 review finding #1's restart-recovery scenario: BB_CURRENT_
    POSITION is unset -- exactly as it would be right after mission_node
    crashes/restarts -- but the robot's real TF position sits at a known
    dock pose. The independent TF ground-truth signal must still trigger
    UndockRobot; relying on the blackboard alone would silently skip it and
    reintroduce the planner deadlock this whole mechanism exists to
    prevent."""
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
        # Explicitly unset (py_trees' Blackboard storage is a process-wide
        # global, shared across every Client regardless of name -- confirmed
        # directly, not assumed -- so a prior test, e.g. the full-route
        # integration test above, can leave this set; relying on it being
        # "naturally" absent would make this test's isolation depend on
        # pytest's execution order).
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
    """The common case this must NOT regress: the very first leg of a fresh
    mission (or any leg where the robot genuinely isn't docked anywhere) --
    neither the blackboard nor TF ground truth says "docked" -- must resolve
    immediately with NO UndockRobot call at all, not just "eventually
    succeeds after a harmless extra call". Confirms the gate is still a
    real gate, not a change that fires unconditionally."""
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
        # Explicitly unset -- see test_ensure_undocked_fires_from_tf_ground_
        # truth_when_blackboard_is_unset's comment on why this can't just be
        # assumed absent. Left unset from here on, like a fresh mission
        # that's never docked anywhere yet.
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
    """A dropped service RESPONSE (this environment does drop them under
    load -- Nav2's own lifecycle manager logs it) leaves the is_active future
    pending forever. Without a bound on that wait, VehicleStatusOK keeps
    returning its latched False and mission stays in safe_stop for good, with
    a healthy Nav2 right there. The poll must time out and be re-sent.

    The client is faked rather than the server: rclpy's own service plumbing
    refuses to "just not answer" (a callback returning None raises), and what
    is under test is purely the client-side bound -- requests go out, a
    response never comes back, so a new request must eventually go out."""
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
             f"in safe_stop forever (requests seen: {len(client.requests)})")
        assert client.abandoned, \
            "the timed-out request must be removed from the client's pending map"
        assert behaviour.update() == py_trees.common.Status.FAILURE, \
            "an unanswered lifecycle-manager poll must still read as NOT ok"
    finally:
        rclpy.shutdown()


def _count_cmd_vel_messages(node, received):
    return node.create_subscription(
        Twist, '/cmd_vel', lambda msg: received.append(msg), 10)


def test_stop_robot_publishes_a_zero_twist_on_every_tick():
    """The safe-stop fix: a SINGLE zero Twist (what this used to publish) is
    simply overwritten by the next message from collision_monitor or
    docking_server, both of which publish /cmd_vel far faster than Mission
    ticks. Every tick must publish, and every published command must be
    zero."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_stop_robot_per_tick")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        received = []
        _count_cmd_vel_messages(node, received)

        # publish_hz effectively disabled, so this test measures ONLY the
        # per-tick publishes (the timer gets its own test below).
        behaviour = StopRobot("StopRobot", node, publish_hz=0.001)

        for _ in range(3):
            assert behaviour.update() == py_trees.common.Status.SUCCESS
            _spin_and_drain(executor, 0.1)

        assert len(received) >= 3, \
            f"expected at least one zero Twist per tick, got {len(received)}"
        assert all(msg.linear.x == 0.0 and msg.angular.z == 0.0 for msg in received)
    finally:
        rclpy.shutdown()


def test_stop_robot_keeps_publishing_between_ticks_and_stops_when_invalidated():
    """Mission ticks at 1 Hz and `safe_stop` is a memory=True Sequence, so
    ticking alone leaves multi-second holes against a 20 Hz publisher. The
    behaviour starts a timer on its first tick to fill them -- and must tear
    that timer down when py_trees invalidates it (the root Selector switching
    back to `Mission`), or it would fight healthy navigation forever."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_stop_robot_timer")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        received = []
        _count_cmd_vel_messages(node, received)

        behaviour = StopRobot("StopRobot", node, publish_hz=20.0)
        behaviour.update()  # one tick only -- the rest must come from the timer
        _spin_and_drain(executor, 0.5)

        during_stop = len(received)
        assert during_stop > 2, \
            f"timer should have republished zero between ticks, got {during_stop}"

        behaviour.terminate(py_trees.common.Status.INVALID)
        # Drain whatever was already in flight when the timer was cancelled
        # (a message published a moment before the cancel still gets
        # delivered), THEN measure a clean window.
        _spin_and_drain(executor, 0.2)
        received.clear()
        _spin_and_drain(executor, 0.5)

        assert received == [], \
            "StopRobot must stop publishing once the safe-stop branch is invalidated"
    finally:
        rclpy.shutdown()


def test_stop_robot_keeps_its_timer_across_an_ongoing_safe_stop():
    """terminate() fires on ordinary SUCCESS too (py_trees calls
    stop(SUCCESS) from tick()) -- tearing the timer down there would undo
    the whole point, since StopRobot returns SUCCESS on every single tick of
    an active safe stop."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_stop_robot_timer_survives_success")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        received = []
        _count_cmd_vel_messages(node, received)

        behaviour = StopRobot("StopRobot", node, publish_hz=20.0)
        behaviour.update()
        behaviour.terminate(py_trees.common.Status.SUCCESS)
        received.clear()
        _spin_and_drain(executor, 0.5)

        assert len(received) > 2, \
            "the holding timer must survive a SUCCESS terminate, only INVALID stops it"
    finally:
        rclpy.shutdown()


def _find_behaviour(root, name):
    for behaviour in root.iterate():
        if behaviour.name == name:
            return behaviour
    raise AssertionError(f"no behaviour named {name!r} in the real mission tree")


def test_safe_stop_hold_is_released_when_mission_recovers_right_after_safe_stop_succeeds():
    """REGRESSION (re-review finding N1, Critical): the zero-/cmd_vel hold
    must be released when the tree returns to `Mission`, in the ONE tick
    phase where py_trees' Selector does not invalidate the losing branch.

    The first fix wave's teardown (StopRobot.terminate(INVALID)) was correct
    but unreachable here, so this test drives the REAL tree from
    build_mission_tree() -- not a simplified standalone Selector/Sequence --
    the way mission_node.py drives it (py_trees tree, one tick per
    iteration), through: healthy -> unhealthy (safe_stop engages and
    completes SUCCESS on its own via Pause) -> healthy on the VERY NEXT
    tick.

    That phase is the whole point. A recovery landing mid-cycle (while the
    root Selector is still RUNNING on safe_stop) takes py_trees'
    priority-interrupt path and tears the timer down correctly even against
    the pre-fix code -- so a test that doesn't control the phase passes
    either way and proves nothing. Here the root Selector has just returned
    SUCCESS, so its next tick re-enters the `self.status != RUNNING` reset
    (composites.py:384-388) and `previous == current_child == mission`
    suppresses the invalidation at composites.py:424-431.

    Health is stubbed at its two I/O boundaries only (the lifecycle-manager
    service poll and the TF lookups); every behaviour and every composite in
    the tree is the real one, because the composite wiring is exactly what
    is under test. The Pause is compressed to 0 s so `safe_stop` reaches
    SUCCESS on its first tick deterministically, instead of making the test
    sleep through the production 2 s and hope it lands on the right tick."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_safe_stop_recovery")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        received = []
        _count_cmd_vel_messages(node, received)

        healthy = {"value": False}
        real_transform_is_recent = mission_tree_module._transform_is_recent
        # Both health behaviours read TF through this one helper, so the
        # whole health verdict follows `healthy` -- no broadcaster races to
        # make the critical tick phasing non-deterministic.
        mission_tree_module._transform_is_recent = lambda *args, **kwargs: healthy["value"]
        try:
            root = build_mission_tree(node, _FakeSagaApi(), _docks())
            tree = py_trees.trees.BehaviourTree(root)
            tree.setup(node=node, timeout=5.0)

            _find_behaviour(root, "VehicleStatusOK")._lifecycle_manager_reports_active = \
                lambda: healthy["value"]
            stop_robot = _find_behaviour(root, "StopRobot")
            _find_behaviour(root, "Sleep 2s").duration_sec = 0.0
            safe_stop = _find_behaviour(root, "Safe stop")

            # --- tick 1: unhealthy. safe_stop engages AND completes. ---
            tree.tick()
            _spin_and_drain(executor, 0.2)

            assert stop_robot._timer is not None, \
                "the safe stop never engaged -- this test can't prove anything about release"
            assert received, "safe_stop must actually hold /cmd_vel at zero while engaged"
            assert all(msg.linear.x == 0.0 and msg.angular.z == 0.0 for msg in received)
            assert safe_stop.status == py_trees.common.Status.SUCCESS, \
                ("fixture assumption broken: safe_stop must have completed SUCCESS on its "
                 "own this tick -- that is the phase N1 is about")

            # --- tick 2: healthy again, on the very next tick. ---
            healthy["value"] = True
            tree.tick()

            assert stop_robot._timer is None, \
                ("the zero-velocity holding timer survived recovery: /cmd_vel stays pinned "
                 "at zero for the life of the process, interleaving zeros with Nav2's own "
                 "commands (re-review finding N1)")

            # And prove the symptom is gone, not just the bookkeeping: drain
            # whatever was already in flight when the timer died, then watch
            # a clean window.
            _spin_and_drain(executor, 0.2)
            received.clear()
            _spin_and_drain(executor, 0.5)

            assert received == [], \
                f"/cmd_vel is still being held at zero after recovery ({len(received)} msgs)"
        finally:
            mission_tree_module._transform_is_recent = real_transform_is_recent
    finally:
        rclpy.shutdown()


def test_ensure_undocked_proximity_tolerance_is_not_absurdly_large():
    """Sanity bound on DOCK_PROXIMITY_TOLERANCE_M itself: a position roughly
    midway between two docks placed 10m apart (well outside the configured
    tolerance) must NOT read as "docked" via the TF ground-truth check --
    guards against the tolerance constant ever being accidentally set to
    something so large it stops discriminating "docked" from "just
    somewhere in the arena"."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_undock_tolerance_sanity")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        undock_server = _CountingUndockServer(node)

        docks = _dock_pose_docks()  # A=(0,0), B=(10,10), ...
        tf_buffer = tf2_ros.Buffer()
        now = node.get_clock().now().to_msg()
        # Comfortably farther from every dock than DOCK_PROXIMITY_TOLERANCE_M,
        # but still well inside the arena's own coordinate range (unlike the
        # other tests' deliberately absurd (1000, 1000) "nowhere" position).
        midpoint = 5.0
        assert midpoint > DOCK_PROXIMITY_TOLERANCE_M * 5, \
            "test fixture assumption broken: tighten the midpoint or the docks spacing"
        tf_buffer.set_transform(
            _stamped_transform_at(now, 'map', 'base_footprint', midpoint, midpoint), "test")

        position_writer = py_trees.blackboard.Client(name="test-position-writer-d")
        position_writer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.WRITE)
        # Explicitly unset -- see test_ensure_undocked_fires_from_tf_ground_
        # truth_when_blackboard_is_unset's comment on why this can't just be
        # assumed absent.
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


def test_vehicle_status_ok_fails_while_a_health_topic_is_silent():
    """Nav2 active and map->odom live, but a watched sensor topic has never
    arrived: FAILURE. Once it arrives (and keeps arriving): SUCCESS."""
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    try:
        node = rclpy.create_node("test_vsok_health_topic")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        _serve_lifecycle_manager_is_active(node)
        tf_broadcaster = tf2_ros.TransformBroadcaster(node)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
        behaviour = VehicleStatusOK(
            "VehicleStatusOK", node, tf_buffer, tf_listener,
            health_topics=[("/test_vsok/scan", "sensor_msgs/msg/LaserScan", 1.0)])

        # Long enough for the lifecycle and TF halves to come good on their own.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            _broadcast_fake_localization(tf_broadcaster, node)
            _spin_and_drain(executor, 0.05)
            assert behaviour.update() == py_trees.common.Status.FAILURE
        assert behaviour._last_known_active is True
        assert "/test_vsok/scan" in behaviour._last_fault

        scan = node.create_publisher(LaserScan, "/test_vsok/scan", 10)
        deadline = time.monotonic() + 5.0
        status = py_trees.common.Status.FAILURE
        while time.monotonic() < deadline and status != py_trees.common.Status.SUCCESS:
            scan.publish(LaserScan())
            _broadcast_fake_localization(tf_broadcaster, node)
            _spin_and_drain(executor, 0.05)
            status = behaviour.update()
        assert status == py_trees.common.Status.SUCCESS
        assert behaviour._last_fault is None
    finally:
        rclpy.shutdown()


def test_stop_robot_zeroes_cmd_vel_ctrl_too():
    """With wheel_drive on, Nav2 ends in cmd_vel_ctrl; a hold that only covered
    /cmd_vel would leave the robot driving through a safe stop."""
    rclpy.init()
    try:
        node = rclpy.create_node("test_stop_robot_ctrl")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        received = []
        node.create_subscription(
            Twist, '/cmd_vel_ctrl', lambda msg: received.append(msg), 10)
        behaviour = StopRobot("StopRobot", node, publish_hz=0.001)
        behaviour.update()
        _spin_and_drain(executor, 0.2)
        assert received and all(
            m.linear.x == 0.0 and m.angular.z == 0.0 for m in received)
    finally:
        rclpy.shutdown()


def test_stop_robot_publishes_nothing_when_the_hold_is_disabled():
    rclpy.init()
    try:
        node = rclpy.create_node("test_stop_robot_disabled")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        received = []
        _count_cmd_vel_messages(node, received)
        node.create_subscription(
            Twist, '/cmd_vel_ctrl', lambda msg: received.append(msg), 10)
        behaviour = StopRobot("StopRobot", node, publish_hz=20.0, hold_zero=False)
        assert behaviour.update() == py_trees.common.Status.SUCCESS
        _spin_and_drain(executor, 0.3)
        assert received == []
    finally:
        rclpy.shutdown()

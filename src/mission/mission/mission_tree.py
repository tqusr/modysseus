import logging
import math
import time

import py_trees
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import Trigger

from mission.bt_behaviours import BlackboardFlagTrue, CallAction, SetBlackboardFlag, WaitUntil
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot

_logger = logging.getLogger('mission.mission_tree')

# How stale a TF lookup can be before it's treated as "not actually live" --
# e.g. slam_toolbox or the odom->base_footprint broadcaster has died but the
# last transform is still sitting in the tf2 buffer.
MAX_TF_AGE_SEC = 2.0

# How far into the FUTURE a transform's own stamp is allowed to be, relative
# to this node's clock, before it's treated as suspect rather than live.
# Generous above slam_toolbox's observed ~0.9s deliberate look-ahead (see
# _transform_is_recent's docstring), but still a real, finite ceiling: an
# unbounded allowance here would wave through exactly the failure mode this
# whole check exists to catch -- e.g. some upstream node in the TF chain
# missing use_sim_time:true (this task's own mission_node hit that bug) and
# stamping with real wall-clock time while everything else runs on sim time,
# which would show up as an enormous, nonsensical negative age_sec.
FUTURE_TF_TOLERANCE_SEC = 2.0

# nav2_lifecycle_manager's own health-check service for the navigation stack
# (navigation.launch.py's `lifecycle_manager_navigation`) -- a std_srvs/Trigger
# that returns success=True only while all of its managed nodes are active.
LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE = '/lifecycle_manager_navigation/is_active'

# How long a single outstanding is_active poll may stay unanswered before
# VehicleStatusOK abandons it and sends a fresh one. Generous next to the
# service's normal sub-millisecond answer (it has been observed taking >15s
# on a heavily loaded machine mid-route, so this must not be tight), but
# finite: see _lifecycle_manager_reports_active for the permanent-safe_stop
# wedge an unbounded wait causes when a response is dropped.
LIFECYCLE_POLL_TIMEOUT_SEC = 30.0

# Ground-truth radius (meters) for "is the robot currently sitting at a dock
# pose" per the TF-based fallback in _not_docked_anywhere, below -- generous
# above the ~0.03-0.04m docking precision Tasks 5/7/8 have all independently
# measured, tight enough that merely driving past/near a dock (staging
# poses are 0.6m out, per mission/config/docks.yaml's own header comment)
# doesn't false-positive.
DOCK_PROXIMITY_TOLERANCE_M = 0.3

# The single dock plugin configured in perception/config/docking.yaml
# (`dock_plugins: ["arena_dock"]`). Sent as UndockRobot.Goal.dock_type -- see
# build_ensure_undocked's undock_goal() for why that matters.
DOCK_PLUGIN_TYPE = "arena_dock"

BB_ROUTE_ID = "route_id"
BB_SOURCE = "source_dock"
BB_DESTINATION = "destination_dock"
BB_HOLDS_PACKAGE = "holds_package"
BB_PACKAGE_DELIVERED = "package_delivered"
# Mission's own stand-in for real localization until that owner replaces it:
# "dock:<letter>" | "predock:<letter>" | unset.
BB_CURRENT_POSITION = "current_position"


def _saga_status_is(saga_client, status):
    """WaitUntil condition factory: SUCCESS once saga_client.get_status()'s
    "status" field matches. Swallows exceptions (e.g. requests.RequestException
    on a transient network blip) the same way GetRouteFromSaga does -- a
    failed poll should just keep waiting, not crash tree.tick()."""

    def check():
        try:
            return saga_client.get_status()["status"] == status
        except Exception as error:
            _logger.warning(f"Saga AI unreachable while polling status: {error}")
            return False

    return check


def _saga_route_is_unloaded(saga_client):
    """WaitUntil condition for "SagaUnloaded?": unlike loading_complete, the
    real Saga AI server atomically reassigns the truck to its next route
    (status back to "loading_pending") in the same action that applies
    unloading_complete -- so that status is never observable through the
    live get_status() endpoint. Reading the specific route by its own
    route_id via get_route_by_id() instead returns that route's terminal
    status even after the server has moved the truck on to a new route.
    The route_id isn't known until GetRouteFromSaga runs, so it's read from
    the blackboard at check time rather than captured at tree-build time."""

    blackboard = py_trees.blackboard.Client(name="saga-unloaded-check")
    blackboard.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)

    def check():
        try:
            route_id = blackboard.get(BB_ROUTE_ID)
            return saga_client.get_route_by_id(route_id)["status"] == "unloading_complete"
        except Exception as error:
            _logger.warning(f"Saga AI unreachable while polling status: {error}")
            return False

    return check


class HasRoute(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)

    def update(self):
        exists = self.blackboard.exists(BB_ROUTE_ID)
        return py_trees.common.Status.SUCCESS if exists else py_trees.common.Status.FAILURE


class GetRouteFromSaga(py_trees.behaviour.Behaviour):
    def __init__(self, name, saga_client):
        super().__init__(name)
        self.saga_client = saga_client
        self.blackboard = self.attach_blackboard_client(name=name)
        for key in (BB_ROUTE_ID, BB_SOURCE, BB_DESTINATION):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self):
        try:
            route = self.saga_client.get_route()
            if route is None:
                return py_trees.common.Status.FAILURE
            route_id = route["route_id"]
            source = route["source"]["letter"]
            destination = route["destination"]["letter"]
        except Exception as error:  # requests.RequestException in production, or malformed route
            self.logger.warning(f"Saga AI unreachable or malformed route: {error}")
            return py_trees.common.Status.FAILURE
        self.blackboard.set(BB_ROUTE_ID, route_id)
        self.blackboard.set(BB_SOURCE, source)
        self.blackboard.set(BB_DESTINATION, destination)
        return py_trees.common.Status.SUCCESS


class ClearRoute(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        for key in (BB_ROUTE_ID, BB_SOURCE, BB_DESTINATION, BB_HOLDS_PACKAGE, BB_PACKAGE_DELIVERED):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

    def update(self):
        for key in (BB_ROUTE_ID, BB_SOURCE, BB_DESTINATION):
            if self.blackboard.exists(key):
                self.blackboard.unset(key)
        self.blackboard.set(BB_HOLDS_PACKAGE, False)
        self.blackboard.set(BB_PACKAGE_DELIVERED, False)
        return py_trees.common.Status.SUCCESS


class StopRobot(py_trees.behaviour.Behaviour):
    """Holds /cmd_vel at zero for as long as the safe-stop branch is active.

    NOT a single publish (what this was originally, and a real final-review
    finding): /cmd_vel has other publishers -- nav2_collision_monitor, the
    last link in the navigation chain, pushes ~20 Hz, and docking_server
    publishes there directly during a docking/undocking leg (see
    perception/launch/docking.launch.py's documented exception). One zero
    Twist from here is simply overwritten by the next message from either of
    them, so the robot never actually stops. Worse, Mission ticks at 1 Hz
    (mission_node.py) and `safe_stop` is a memory=True Sequence, so this
    behaviour is only *ticked* about once per three-second stop/report/pause
    cycle -- ticking alone can't hold anything down.

    So two mechanisms, deliberately:
      1. update() publishes a zero Twist on EVERY tick it gets (the
         behaviour-tree-visible part, and what the unit tests assert).
      2. The first such tick also starts a `publish_hz` ROS timer on the
         mission node that keeps republishing zero between ticks, which is
         what actually holds the line against a 20 Hz publisher. It is torn
         down in terminate(INVALID) -- i.e. the moment py_trees leaves the
         safe-stop branch because `Mission` succeeded again -- so it can
         never fight a healthy, resumed navigation.

    The real stop is still CallAction.terminate()'s goal cancellation (the
    server-side fix); this is the belt-and-braces cover for the window
    between "goal cancel requested" and "the server has actually wound
    down", and for anything commanding /cmd_vel that this tree never knew
    about.
    """

    #: Fast relative to Mission's 1 Hz tick, and to Nav2's 20 Hz controller
    #: rate -- high enough to dominate the gap between ticks, low enough to
    #: be free on a Jetson Orin Nano.
    DEFAULT_PUBLISH_HZ = 10.0

    def __init__(self, name, node, publish_hz=DEFAULT_PUBLISH_HZ):
        super().__init__(name)
        self._node = node
        self._publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self._publish_hz = publish_hz
        self._timer = None

    def update(self):
        self._publish_zero()
        if self._timer is None:
            self._timer = self._node.create_timer(1.0 / self._publish_hz, self._publish_zero)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        # INVALID means the root Selector has switched back to `Mission`
        # (py_trees invalidates the losing branch) -- stop holding /cmd_vel
        # down so navigation can drive again. SUCCESS/FAILURE, by contrast,
        # happen on every tick within an ongoing safe stop; the timer must
        # survive those.
        if new_status == py_trees.common.Status.INVALID:
            self._cancel_timer()

    def _publish_zero(self):
        self._publisher.publish(Twist())

    def _cancel_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
            self._timer = None


class ReportFaultToSaga(py_trees.behaviour.Behaviour):
    """Saga AI has no report endpoint (truck is read-only) -- this just logs,
    matching ReportRouteDone's pattern."""

    def update(self):
        self.logger.warning("Mission fault -- stopping and holding position")
        return py_trees.common.Status.SUCCESS


class Pause(py_trees.behaviour.Behaviour):
    """RUNNING until duration_sec has elapsed since initialise(), then SUCCESS."""

    def __init__(self, name, duration_sec, clock=time.monotonic):
        super().__init__(name)
        self.duration_sec = duration_sec
        self.clock = clock
        self._start_time = None

    def initialise(self):
        self._start_time = self.clock()

    def update(self):
        if self.clock() - self._start_time >= self.duration_sec:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class ReportRouteDone(py_trees.behaviour.Behaviour):
    """Saga AI has no report endpoint (truck is read-only) — this just logs."""

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)

    def update(self):
        route_id = self.blackboard.get(BB_ROUTE_ID) if self.blackboard.exists(BB_ROUTE_ID) else None
        self.logger.info(f"Route {route_id} complete")
        return py_trees.common.Status.SUCCESS


def _transform_is_recent(tf_buffer, node, target_frame, source_frame,
                          max_age_sec=MAX_TF_AGE_SEC, future_tolerance_sec=FUTURE_TF_TOLERANCE_SEC):
    """True iff target_frame<-source_frame resolves in tf_buffer right now,
    and the resolved transform's own stamp is within max_age_sec of the
    node's current clock -- i.e. some live broadcaster is actually still
    publishing it, not just a stale entry left over in the tf2 cache.

    `rclpy.time.Time()` (zero) is tf2's own convention for "the latest
    transform available", not a literal timestamp to match.

    Both directions are bounded, asymmetrically. Confirmed against the real
    stack (not assumed): slam_toolbox's localization node here consistently
    publishes map->odom stamped ~0.9s AHEAD of the node's own clock sample
    (i.e. age_sec came out around -0.9, not ~0), every single tick -- this
    is deliberate on slam_toolbox's part, not jitter or a bug, matching
    localization_params.yaml's own `transform_timeout: 1.0` (stamping
    slightly into the future is a standard tf2 technique to keep consumers
    from hitting "extrapolation into the past" given this sim's known dual-
    /clock-publisher jumps -- see this stack's shared lessons). So a small
    future stamp is allowed and is NOT treated as staleness. But the future
    allowance is still a real, finite ceiling (`future_tolerance_sec`), not
    unbounded: an arbitrarily-far-future stamp is exactly what a desynced or
    wrongly-configured clock somewhere in the TF chain would produce (e.g. a
    node missing use_sim_time:true, stamping with wall-clock time while
    everything else runs on sim time -- this task's own mission_node hit
    that exact bug on itself), and that failure mode is precisely what this
    health check exists to catch, not wave through.
    """
    try:
        transform = tf_buffer.lookup_transform(target_frame, source_frame, Time())
    except tf2_ros.TransformException as error:
        _logger.warning(f"TF lookup {target_frame}<-{source_frame} failed: {error}")
        return False
    stamp = Time.from_msg(transform.header.stamp)
    age_sec = (node.get_clock().now() - stamp).nanoseconds / 1e9
    return -future_tolerance_sec <= age_sec <= max_age_sec


def _near_any_dock_pose(tf_buffer, docks, tolerance_m=DOCK_PROXIMITY_TOLERANCE_M):
    """Ground-truth fallback for "is the robot currently docked somewhere",
    independent of mission_node's own in-memory blackboard (Task 8 review
    finding #1): reads the LATEST available map->base_footprint transform
    and checks it against every known dock's own dock_pose (not staging_pose)
    within tolerance_m.

    Deliberately does NOT itself re-check TF recency/staleness the way
    _transform_is_recent does: by the time build_goto_dock_subtree's
    `ensure_undocked` gate (the only caller of this) is ever ticked,
    Mission's own root Sequence has already required `VehicleStatusOK` and
    `Localize` to both SUCCEED that same tick (see build_mission_tree) --
    i.e. a live, recent map->base_footprint is already guaranteed upstream,
    so re-deriving that guarantee here would be redundant, not safer.

    Why this exists at all: BB_CURRENT_POSITION is pure in-process
    behaviour-tree state. If mission_node itself crashes or is manually
    relaunched while the robot is physically still sitting docked, that
    blackboard resets to unset -- read alone, _not_docked_anywhere would
    wrongly conclude "not docked", skip UndockRobot, and silently
    reintroduce the exact planner deadlock Task 8's own fix exists to
    prevent, with nothing in the logs pointing at the real cause (the
    planner failure looks identical either way). This TF check is a second,
    independent signal from the one piece of state that DOES survive a
    mission_node restart: the robot's real physical pose.

    What a false positive now costs (CHANGED by the final-review fix wave,
    read this before reusing the old reasoning): this used to be free,
    because UndockRobot.Goal was sent with an empty `dock_type`, so a
    docking_server with nothing stashed threw DockNotValid internally
    (zero velocity published, result.success=False) and nothing moved.
    build_ensure_undocked now sets `dock_type: "arena_dock"` explicitly
    (see undock_goal() there, and the Task 8 limitation it closes), so the
    plugin always resolves and a false positive costs one real, short
    reverse undock manoeuvre instead of a free failed call. That trade is
    deliberate and still the right one: the alternative failure mode is the
    robot being unable to undock at all after a docking_server restart,
    which deadlocks the whole mission (the exact deadlock this mechanism
    exists to prevent), whereas a false positive requires the robot to be
    within DOCK_PROXIMITY_TOLERANCE_M (0.3 m) of a real dock pose while not
    actually docked -- and that is a dock approach corridor, kept clear of
    props by the arena design (see arena.sdf's own prop-layout notes), with
    the undock manoeuvre bounded by docking.yaml's staging offset (0.6 m).

    docking_server is a separate long-running process (docking.launch.py)
    whose own docked-state stash survives a mission_node-only restart intact
    and correct, which is the realistic restart scenario this guards. The
    fuller "docking_server ALSO lost its stash" restart (e.g. a full-stack
    restart while physically docked) is still not recoverable -- confirmed
    live, see undock_goal() in build_ensure_undocked for the measurement and
    why sending dock_type doesn't close it.
    """
    try:
        transform = tf_buffer.lookup_transform('map', 'base_footprint', Time())
    except tf2_ros.TransformException as error:
        _logger.warning(f"TF lookup map<-base_footprint failed while checking "
                         f"dock proximity: {error}")
        return False
    x = transform.transform.translation.x
    y = transform.transform.translation.y
    return any(
        math.hypot(x - dock.dock_pose["x"], y - dock.dock_pose["y"]) <= tolerance_m
        for dock in docks.values()
    )


class VehicleStatusOK(py_trees.behaviour.Behaviour):
    """General vehicle health gate feeding Mission's safe-stop branch.

    SUCCESS only while, at minimum:
      - nav2_lifecycle_manager reports its managed navigation nodes active
        (polled via its own is_active std_srvs/Trigger service), and
      - slam_toolbox is actively publishing map->odom (TF lookup, rejecting
        anything older than max_tf_age_sec).
    FAILURE otherwise -- including while the lifecycle-manager service
    simply isn't up yet, which is the correct read for "status not OK".
    """

    def __init__(self, name, node, tf_buffer, tf_listener,
                 lifecycle_manager_service=LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE,
                 max_tf_age_sec=MAX_TF_AGE_SEC,
                 poll_timeout_sec=LIFECYCLE_POLL_TIMEOUT_SEC):
        super().__init__(name)
        self.node = node
        self.tf_buffer = tf_buffer
        # Not otherwise referenced -- held here purely so the listener (and
        # its tf/tf_static subscriptions) outlives this call and isn't
        # garbage-collected once build_mission_tree() returns.
        self._tf_listener = tf_listener
        self.max_tf_age_sec = max_tf_age_sec
        self.poll_timeout_sec = poll_timeout_sec
        self._client = node.create_client(Trigger, lifecycle_manager_service)
        self._future = None
        self._future_deadline = None
        # Latched, not reset to False while a fresh poll is merely in flight --
        # see _lifecycle_manager_reports_active's docstring for why.
        self._last_known_active = False

    def update(self):
        if not self._lifecycle_manager_reports_active():
            return py_trees.common.Status.FAILURE
        if not _transform_is_recent(self.tf_buffer, self.node, 'map', 'odom', self.max_tf_age_sec):
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def _lifecycle_manager_reports_active(self):
        """Non-blocking, like CallAction: tree.tick() must never block on a
        service response, so this polls via a cached async future instead of
        rclpy.spin_until_future_complete -- but unlike CallAction (a one-shot
        goal with no notion of "still fine from last time"), this repolls
        forever, so the cached *last confirmed* result is returned while a
        fresh poll is in flight, rather than provisionally reporting False
        for the tick(s) it takes that poll to resolve. Without that latch,
        a request fired on tick N (which can't possibly resolve until tick
        N+1's spin_once has run) would make VehicleStatusOK flicker FAILURE
        on every tick a new poll happens to be outstanding, tripping the
        safe-stop branch and invalidating -- and resetting the progress of
        -- whatever in the `route` subtree was RUNNING, even though nothing
        actually became unhealthy.

        The outstanding poll is bounded by poll_timeout_sec, and that bound
        is load-bearing, not defensive decoration: a dropped service
        RESPONSE leaves the future pending forever, and with no timeout this
        behaviour would then return its latched value (False, in the common
        case where the drop happens on the first poll after a restart)
        FOREVER -- mission wedged in safe_stop, never recovering, with a
        perfectly healthy Nav2 sitting right there. That is not theoretical:
        this environment drops service responses under load often enough
        that Nav2's own lifecycle manager logs it ("failed to send response
        to /planner_server/change_state (timeout): client will not receive
        response"), and it was observed wedging mission exactly this way
        during the final-review fix wave's live testing. On timeout the
        request is abandoned (and removed from the client's pending map, so
        it doesn't leak) and a fresh poll goes out on the next tick."""
        if self._future is None:
            if self._client.service_is_ready():
                self._future = self._client.call_async(Trigger.Request())
                self._future_deadline = \
                    self.node.get_clock().now() + Duration(seconds=self.poll_timeout_sec)
            return self._last_known_active
        if not self._future.done():
            if self._future_deadline is not None and \
                    self.node.get_clock().now() > self._future_deadline:
                self.logger.warning(
                    f"{self._client.srv_name} did not answer within "
                    f"{self.poll_timeout_sec}s -- abandoning that poll and retrying")
                self._client.remove_pending_request(self._future)
                self._future = None
                self._future_deadline = None
            return self._last_known_active
        future, self._future = self._future, None
        self._future_deadline = None
        try:
            self._last_known_active = bool(future.result().success)
        except Exception as error:
            self.logger.warning(f"{self._client.srv_name} call failed: {error}")
            self._last_known_active = False
        return self._last_known_active


class Localize(py_trees.behaviour.Behaviour):
    """Narrower than VehicleStatusOK: specifically "are we localized," not
    general system health. SUCCESS iff map->base_footprint resolves and is
    recent (per the established convention, base_frame is base_footprint,
    not base_link -- see localization/config's own header comments)."""

    def __init__(self, name, node, tf_buffer, max_tf_age_sec=MAX_TF_AGE_SEC):
        super().__init__(name)
        self.node = node
        self.tf_buffer = tf_buffer
        self.max_tf_age_sec = max_tf_age_sec

    def update(self):
        is_localized = _transform_is_recent(
            self.tf_buffer, self.node, 'map', 'base_footprint', self.max_tf_age_sec)
        return py_trees.common.Status.SUCCESS if is_localized else py_trees.common.Status.FAILURE


def _at_position(name, dock_key, prefix):
    class _AtPosition(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__(name)
            self.blackboard = self.attach_blackboard_client(name=name)
            self.blackboard.register_key(key=dock_key, access=py_trees.common.Access.READ)
            self.blackboard.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.READ)

        def update(self):
            dock = self.blackboard.get(dock_key)
            current = self.blackboard.get(BB_CURRENT_POSITION) \
                if self.blackboard.exists(BB_CURRENT_POSITION) else None
            return py_trees.common.Status.SUCCESS if current == f"{prefix}:{dock}" \
                else py_trees.common.Status.FAILURE

    return _AtPosition()


def _not_docked_anywhere(name, tf_buffer, docks):
    """SUCCESS iff NEITHER signal says the robot is currently docked
    somewhere -- a fail-safe OR of two independent checks (Task 8 review
    finding #1):

      1. BB_CURRENT_POSITION == "dock:<something>" -- the normal, fast path:
         this tree's own record of having just completed a DockRobot call.
      2. _near_any_dock_pose(tf_buffer, docks) -- a TF ground-truth fallback
         that does NOT depend on this process's own in-memory state, so it
         still catches "docked" correctly even if BB_CURRENT_POSITION was
         reset by a mission_node crash/restart while the robot was
         physically still sitting docked. See _near_any_dock_pose's own
         docstring for why speculatively firing UndockRobot off this signal
         alone is safe (opennav_docking fails it harmlessly, never moves
         the robot, if there's nothing to undock from).

    Either signal alone is enough to conclude "docked" (OR, not AND) --
    this check only needs to decide whether to ATTEMPT an undock, not to
    prove definitively that one is needed; a false positive costs one cheap
    failed action call (see _near_any_dock_pose), while a false negative
    reintroduces the exact planner deadlock this whole mechanism exists to
    prevent. Used to gate UndockRobot: only call it when something says
    we're actually docked."""

    class _NotDocked(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__(name)
            self.blackboard = self.attach_blackboard_client(name=name)
            self.blackboard.register_key(
                key=BB_CURRENT_POSITION, access=py_trees.common.Access.READ)

        def update(self):
            current = self.blackboard.get(BB_CURRENT_POSITION) \
                if self.blackboard.exists(BB_CURRENT_POSITION) else None
            blackboard_says_docked = isinstance(current, str) and current.startswith("dock:")
            if blackboard_says_docked or _near_any_dock_pose(tf_buffer, docks):
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.SUCCESS

    return _NotDocked()


def build_ensure_undocked(node, tf_buffer, docks, position_writer):
    """Selector: SUCCESS immediately if _not_docked_anywhere says we're not
    currently docked; otherwise fires a real UndockRobot action call before
    returning (RUNNING while in flight, SUCCESS/FAILURE on its result).

    Extracted as its own top-level function (rather than inlined in
    build_goto_dock_subtree, where it originally lived) specifically so it
    can be unit-tested in isolation -- see
    test_mission_tree.py's test_ensure_undocked_* tests, which tick this
    directly against a counting fake `undock_robot` action server and
    assert it fires/doesn't fire under each of the blackboard/TF signal
    combinations, independent of the full dock/navigate/undock subtree.
    """

    def mark_undocked():
        if position_writer.exists(BB_CURRENT_POSITION):
            position_writer.unset(BB_CURRENT_POSITION)

    def undock_goal():
        """UndockRobot.Goal with `dock_type` explicitly set.

        nav2_msgs/action/UndockRobot.action's own comment:

            If initialized on a dock so the server doesn't know what type of
            dock its on, you must specify what dock it is to know where to
            stage for undocking. [...] If not set & server instance was used
            to dock, server will use current dock information from last
            docking request.

        Leaving it empty (what this tree did before) makes the server fall
        back to its own in-process `curr_dock_type_`, which is empty after a
        docking_server restart -- so the goal is then rejected outright with
        DockNotValid ("No dock information to undock from!"). Sending the
        type removes that one dependency on server-side memory, and is a
        no-op when the stash is valid (same plugin either way). "arena_dock"
        is the only entry in `dock_plugins` in
        perception/config/docking.yaml; keep the two in sync.

        WHAT THIS DOES NOT FIX, measured live 2026-10-02, do not assume
        otherwise: it does NOT make undocking work after a docking_server
        restart. Verified against the running stack by restarting
        docking_server while the robot sat physically docked at A and
        watching every subsequent UndockRobot goal abort with "Robot is not
        in the dock, no need to undock". opennav_docking's undockRobot()
        gates on `dock->isCharger() && (!dock->isDocked() &&
        !dock->isCharging())` BEFORE it uses dock_type for anything, and
        SimpleChargingDock::isDocked() measures against the dock pose it
        stashed during its own approach -- also lost in the restart, with no
        parameter to supply it and no `is_charger: false` escape hatch in
        Jazzy's plugin (its declared params are use_battery_status,
        use_external_detection_pose, external_detection_*, filter_coef,
        charging_threshold, use_stall_detection, stall_*, docking_threshold,
        staging_*). So Task 8's "full-stack restart while physically docked"
        limitation stands; it is an opennav_docking design constraint, not
        something this client can send its way out of.
        """
        goal = UndockRobot.Goal()
        goal.dock_type = DOCK_PLUGIN_TYPE
        return goal

    undock = CallAction(
        "UndockRobot", node=node, action_type=UndockRobot,
        action_name="undock_robot", goal_builder=undock_goal, on_success=mark_undocked,
    )
    return py_trees.composites.Selector(
        name="Not docked?", memory=False,
        children=[_not_docked_anywhere("NotDockedAnywhere?", tf_buffer, docks), undock])


def build_goto_dock_subtree(node, dock_key, docks, tf_buffer):
    """dock_key is BB_SOURCE or BB_DESTINATION — resolved at tick time, so the
    same subtree definition serves both the Load and Deliver branches.
    `tf_buffer` is build_mission_tree's own shared tf2_ros.Buffer (same one
    VehicleStatusOK/Localize use) -- threaded through to
    build_ensure_undocked's TF ground-truth check, not a new subscription."""

    position_writer = py_trees.blackboard.Client(name=f"position-writer-{dock_key}")
    position_writer.register_key(key=dock_key, access=py_trees.common.Access.READ)
    position_writer.register_key(key=BB_CURRENT_POSITION, access=py_trees.common.Access.WRITE)

    def mark_predock():
        position_writer.set(BB_CURRENT_POSITION, f"predock:{position_writer.get(dock_key)}")

    def mark_dock():
        position_writer.set(BB_CURRENT_POSITION, f"dock:{position_writer.get(dock_key)}")

    at_dock = _at_position("At dock?", dock_key, "dock")
    at_predock = _at_position("At pre-dock?", dock_key, "predock")

    def navigate_goal():
        dock = docks[position_writer.get(dock_key)]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = dock.staging_pose["x"]
        goal.pose.pose.position.y = dock.staging_pose["y"]
        theta = dock.staging_pose["theta"]
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)
        return goal

    def dock_goal():
        dock = docks[position_writer.get(dock_key)]
        goal = DockRobot.Goal()
        goal.dock_id = dock.letter
        return goal

    navigate = CallAction(
        "NavigateToPose(staging)", node=node, action_type=NavigateToPose,
        action_name="navigate_to_pose", goal_builder=navigate_goal, on_success=mark_predock,
    )
    dock_is_free = WaitUntil(
        "DockIsFree?", condition=lambda: True, timeout_sec=60.0, poll_sec=1.0,
    )  # stubbed free-check — real multi-robot occupancy comes from perception later
    dock_action = CallAction(
        "DockRobot", node=node, action_type=DockRobot,
        action_name="dock_robot", goal_builder=dock_goal, on_success=mark_dock,
    )

    # Real-stack discovery (Task 8): opennav_docking parks the robot nose-first
    # against the dock's wall, close enough that the LIDAR/camera obstacle
    # layers mark the robot's own current cell as lethal/inflated in the
    # global costmap -- NavigateToPose's planner then can never find a start
    # pose within tolerance to plan FROM, forever, for any destination. There
    # was no undock step anywhere in this tree before, so the first "dock,
    # then navigate elsewhere" transition (never exercised end-to-end until
    # Task 8's full route test) deadlocked the whole mission. Fixed by
    # calling the real `/undock_robot` action (opennav_docking's own
    # UndockRobot, live alongside dock_robot) before ever trying to navigate
    # away from a dock -- it reverses the robot off the dock using local
    # costmap/odometry control, not a global plan, so it doesn't hit the same
    # chicken-and-egg problem. Gated on build_ensure_undocked's fail-safe-OR
    # check (blackboard state OR live TF ground truth -- see that function
    # and _not_docked_anywhere's docstrings) so it's skipped only when
    # NEITHER signal says we're docked (e.g. the very first leg of the whole
    # mission, spawning free), not just on this process's own possibly-stale
    # blackboard record.
    ensure_undocked = build_ensure_undocked(node, tf_buffer, docks, position_writer)

    move_and_dock = py_trees.composites.Sequence(name="Move and dock", memory=True, children=[
        ensure_undocked,
        py_trees.composites.Selector(name="At pre-dock?", memory=False, children=[at_predock, navigate]),
        dock_is_free,
        dock_action,
    ])

    return py_trees.composites.Selector(name="GoToDock", memory=False, children=[at_dock, move_and_dock])


def build_mission_tree(node, saga_client, docks):
    # Single shared Buffer/TransformListener for both health-check behaviours
    # below, rather than one pair each -- avoids duplicate /tf, /tf_static
    # subscriptions on the same node for the same data.
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)
    vehicle_status_ok = VehicleStatusOK("VehicleStatusOK", node, tf_buffer, tf_listener)
    localize = Localize("Localize", node, tf_buffer)

    have_route = py_trees.composites.Selector(name="Have route", memory=False, children=[
        HasRoute("HasRoute?"),
        GetRouteFromSaga("GetRouteFromSaga", saga_client),
    ])

    load = py_trees.composites.Selector(name="Load", memory=False, children=[
        BlackboardFlagTrue("HoldsPackage?", key=BB_HOLDS_PACKAGE),
        py_trees.composites.Sequence(name="Load at source", memory=True, children=[
            build_goto_dock_subtree(node, BB_SOURCE, docks, tf_buffer),
            WaitUntil("SagaLoaded?", condition=_saga_status_is(saga_client, "loading_complete"),
                      timeout_sec=300.0, poll_sec=1.0),
            SetBlackboardFlag("Set HoldsPackage", key=BB_HOLDS_PACKAGE, value=True),
        ]),
    ])

    deliver = py_trees.composites.Selector(name="Deliver", memory=False, children=[
        BlackboardFlagTrue("PackageDelivered?", key=BB_PACKAGE_DELIVERED),
        py_trees.composites.Sequence(name="Unload at destination", memory=True, children=[
            build_goto_dock_subtree(node, BB_DESTINATION, docks, tf_buffer),
            WaitUntil("SagaUnloaded?", condition=_saga_route_is_unloaded(saga_client),
                      timeout_sec=300.0, poll_sec=1.0),
            SetBlackboardFlag("Set PackageDelivered", key=BB_PACKAGE_DELIVERED, value=True),
        ]),
    ])

    route = py_trees.composites.Sequence(name="Route", memory=True, children=[
        have_route, load, deliver, ReportRouteDone("ReportRouteDone"), ClearRoute("ClearRoute"),
    ])

    # py_trees 2.5.0 (ROS 2 Jazzy's installed version) folded the old standalone
    # ReactiveSequence class into Sequence's `memory` flag: Sequence(memory=False)
    # re-ticks from its first child every tick (never remembers a previously-running
    # child), which is exactly ReactiveSequence's semantics -- needed here so the
    # vehicle-status/localization guards are re-checked every tick even while `route`
    # (memory=True, so it tracks its own progress) stays RUNNING underneath.
    mission = py_trees.composites.Sequence(name="Mission", memory=False, children=[
        vehicle_status_ok, localize, route,
    ])

    safe_stop = py_trees.composites.Sequence(name="Safe stop", memory=True, children=[
        StopRobot("StopRobot", node),
        ReportFaultToSaga("ReportFaultToSaga"),
        Pause("Sleep 2s", duration_sec=2.0),
    ])

    return py_trees.composites.Selector(name="Mission or safe stop", memory=False, children=[mission, safe_stop])

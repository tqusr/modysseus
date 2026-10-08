"""Builds the mission behaviour tree: health guards, then load and deliver."""
import logging
import math

import py_trees
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import Trigger

from mission.bt_behaviours import BlackboardFlagTrue, CallAction, SetBlackboardFlag, WaitUntil
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot

_logger = logging.getLogger('mission.mission_tree')

# Oldest a TF stamp may be before it counts as not live, e.g. a dead
# broadcaster whose last transform is still in the tf2 buffer.
MAX_TF_AGE_SEC = 2.0

# Furthest into the future a TF stamp may be. slam_toolbox deliberately stamps
# about 0.9 s ahead. The ceiling stays finite so a node on the wrong clock
# (e.g. use_sim_time without /clock) is still caught.
FUTURE_TF_TOLERANCE_SEC = 2.0

# nav2_lifecycle_manager's health service (std_srvs/Trigger): success only
# while all managed nodes are active.
LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE = '/lifecycle_manager_navigation/is_active'

# Seconds before an unanswered is_active poll is abandoned and resent. Generous
# because the service has taken over 15 s under load, but finite so a dropped
# response cannot wedge the check.
LIFECYCLE_POLL_TIMEOUT_SEC = 30.0

# Radius in meters within which the robot counts as sitting at a dock pose.
# Above the measured ~0.03-0.04 m docking precision, below the 0.6 m staging
# offset in docks.yaml.
DOCK_PROXIMITY_TOLERANCE_M = 0.3

# Dock plugin from perception/config/docking.yaml (`dock_plugins`); sent as
# UndockRobot.Goal.dock_type.
DOCK_PLUGIN_TYPE = "arena_dock"

BB_ROUTE_ID = "route_id"
BB_SOURCE = "source_dock"
BB_DESTINATION = "destination_dock"
BB_HOLDS_PACKAGE = "holds_package"
BB_PACKAGE_DELIVERED = "package_delivered"
# Stand-in for real localization: "dock:<letter>", "predock:<letter>" or unset.
BB_CURRENT_POSITION = "current_position"


def _saga_status_is(saga_client, status):
    """Returns a `WaitUntil` condition that is true once Saga reports `status`.

    Request errors are logged and treated as false so a network blip does not
    crash `tree.tick()`.

    Args:
        saga_client: Client exposing `get_status()`.
        status: Value of the "status" field to wait for.
    """

    def check():
        try:
            return saga_client.get_status()["status"] == status
        except Exception as error:
            _logger.warning(f"Saga AI unreachable while polling status: {error}")
            return False

    return check


def _saga_route_is_unloaded(saga_client):
    """Returns a `WaitUntil` condition that is true once the route is unloaded.

    Saga reassigns the truck to its next route in the same action that sets
    unloading_complete, so the live status never shows it. This reads the
    route by id instead, taking the id from the blackboard at check time.

    Args:
        saga_client: Client exposing `get_route_by_id()`.
    """
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
    """Succeeds if a route id is on the blackboard."""

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)

    def update(self):
        exists = self.blackboard.exists(BB_ROUTE_ID)
        return py_trees.common.Status.SUCCESS if exists else py_trees.common.Status.FAILURE


class GetRouteFromSaga(py_trees.behaviour.Behaviour):
    """Fetches the current route from Saga into the blackboard.

    Fails if there is no route, Saga is unreachable or the route is malformed.
    """

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
        except Exception as error:  # RequestException in production, or a malformed route.
            self.logger.warning(f"Saga AI unreachable or malformed route: {error}")
            return py_trees.common.Status.FAILURE
        self.logger.info(f"[route] {route_id}: pick up at {source}, deliver to {destination}")
        self.blackboard.set(BB_ROUTE_ID, route_id)
        self.blackboard.set(BB_SOURCE, source)
        self.blackboard.set(BB_DESTINATION, destination)
        return py_trees.common.Status.SUCCESS


class ClearRoute(py_trees.behaviour.Behaviour):
    """Clears the route and package state from the blackboard."""

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


class ReportRouteDone(py_trees.behaviour.Behaviour):
    """Logs route completion. Saga has no report endpoint, so nothing is sent."""

    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=BB_ROUTE_ID, access=py_trees.common.Access.READ)

    def update(self):
        route_id = self.blackboard.get(BB_ROUTE_ID) if self.blackboard.exists(BB_ROUTE_ID) else None
        self.logger.info(f"[route] {route_id} complete")
        return py_trees.common.Status.SUCCESS


def _transform_is_recent(tf_buffer, node, target_frame, source_frame,
                          max_age_sec=MAX_TF_AGE_SEC, future_tolerance_sec=FUTURE_TF_TOLERANCE_SEC):
    """Checks that a live broadcaster is publishing a transform.

    Looks up the latest `target_frame <- source_frame` transform and checks
    that its stamp is at most `max_age_sec` behind the node clock and at most
    `future_tolerance_sec` ahead of it. The future allowance exists because
    slam_toolbox stamps map->odom about 0.9 s ahead on purpose.

    Args:
        tf_buffer: tf2 buffer to query.
        node: Node whose clock defines "now".
        target_frame: Target frame id.
        source_frame: Source frame id.
        max_age_sec: Oldest acceptable stamp age in seconds.
        future_tolerance_sec: Furthest acceptable future stamp in seconds.

    Returns:
        True if the transform resolves and its stamp is within bounds.
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
    """Checks from TF whether the robot sits at any dock pose.

    This survives a mission_node restart, which resets the blackboard
    while the robot is still docked. Without it the tree would skip
    UndockRobot and deadlock the planner, because the robot's own cell is
    inflated. TF recency is not checked here: the Mission sequence has
    already required `VehicleStatusOK` and `Localize` this tick.

    A false positive costs one short reverse undock. That is acceptable
    because it needs the robot within `tolerance_m` of a dock pose without
    being docked, and the arena keeps that approach corridor clear.

    Not recoverable: if docking_server also restarted while the robot was
    docked, it has lost its dock state and rejects undocking.

    Args:
        tf_buffer: tf2 buffer to query.
        docks: Dock letter to `mission.docks.Dock`.
        tolerance_m: Radius around each dock pose in meters.

    Returns:
        True if map->base_footprint is within `tolerance_m` of a dock pose.
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
    """Health gate at the head of Mission.

    Succeeds only while the Nav2 lifecycle manager reports its nodes active
    and map->odom is being published within `max_tf_age_sec`. Fails otherwise,
    including while the lifecycle service is not up yet. Each new fault is
    logged once, when it starts.
    """

    def __init__(self, name, node, tf_buffer, tf_listener,
                 lifecycle_manager_service=LIFECYCLE_MANAGER_IS_ACTIVE_SERVICE,
                 max_tf_age_sec=MAX_TF_AGE_SEC,
                 poll_timeout_sec=LIFECYCLE_POLL_TIMEOUT_SEC):
        """Initializes the gate.

        Args:
            name: Behaviour name.
            node: Node used for the service client and clock.
            tf_buffer: Shared tf2 buffer.
            tf_listener: Listener feeding `tf_buffer`; held so it is not
                garbage-collected.
            lifecycle_manager_service: Name of the is_active Trigger service.
            max_tf_age_sec: Oldest acceptable map->odom stamp in seconds.
            poll_timeout_sec: Seconds before an unanswered poll is abandoned.
        """
        super().__init__(name)
        self.node = node
        self._last_fault = None
        self.tf_buffer = tf_buffer
        self._tf_listener = tf_listener
        self.max_tf_age_sec = max_tf_age_sec
        self.poll_timeout_sec = poll_timeout_sec
        self._client = node.create_client(Trigger, lifecycle_manager_service)
        self._future = None
        self._future_deadline = None
        # Latched so a poll merely in flight does not read as unhealthy.
        self._last_known_active = False

    def update(self):
        fault = self._fault()
        if fault != self._last_fault:
            if fault is None:
                self.logger.info("[mission] ready: Nav2 active, map->odom live")
            else:
                self.logger.warning(f"[mission] waiting: {fault}")
            self._last_fault = fault
        return py_trees.common.Status.FAILURE if fault else py_trees.common.Status.SUCCESS

    def _fault(self):
        """Returns a description of the current fault, or None if healthy.

        The lifecycle poll runs every tick so its latched result never goes
        stale behind an earlier failing check.
        """
        nav_active = self._lifecycle_manager_reports_active()
        if not nav_active:
            return "Nav2 lifecycle manager does not report active"
        if not _transform_is_recent(self.tf_buffer, self.node, 'map', 'odom', self.max_tf_age_sec):
            return "map->odom transform missing or stale"
        return None

    def _lifecycle_manager_reports_active(self):
        """Polls the lifecycle manager without blocking the tick.

        Uses a cached async future and returns the last confirmed result while
        a poll is in flight. Otherwise every new poll would read as FAILURE for
        a tick and invalidate the running route.

        An unanswered poll is abandoned after `poll_timeout_sec` and resent.
        Service responses do get dropped under load, and without the timeout
        a lost first reply would wedge the mission in FAILURE forever.

        Returns:
            True if the last completed poll reported all nodes active.
        """
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
    """Succeeds if map->base_footprint resolves and is recent."""

    def __init__(self, name, node, tf_buffer, max_tf_age_sec=MAX_TF_AGE_SEC):
        """Initializes the check.

        Args:
            name: Behaviour name.
            node: Node whose clock defines "now".
            tf_buffer: Shared tf2 buffer.
            max_tf_age_sec: Oldest acceptable stamp in seconds.
        """
        super().__init__(name)
        self.node = node
        self.tf_buffer = tf_buffer
        self.max_tf_age_sec = max_tf_age_sec

    def update(self):
        is_localized = _transform_is_recent(
            self.tf_buffer, self.node, 'map', 'base_footprint', self.max_tf_age_sec)
        return py_trees.common.Status.SUCCESS if is_localized else py_trees.common.Status.FAILURE


def _at_position(name, dock_key, prefix):
    """Returns a condition that is true if the position is "<prefix>:<dock>".

    Args:
        name: Behaviour name.
        dock_key: Blackboard key holding the dock letter.
        prefix: "dock" or "predock".
    """
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
    """Returns a condition that succeeds only if the robot is not docked.

    Either signal means docked (OR): the blackboard position is "dock:...", or
    `_near_any_dock_pose` finds the robot at a dock pose. The TF signal covers
    a mission_node restart that cleared the blackboard. It gates UndockRobot,
    where a false negative deadlocks the planner and a false positive only
    costs a short undock.

    Args:
        name: Behaviour name.
        tf_buffer: tf2 buffer for the TF check.
        docks: Dock letter to `mission.docks.Dock`.
    """

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
    """Builds a selector that undocks the robot if it is docked.

    Succeeds at once when `_not_docked_anywhere` does. Otherwise sends an
    UndockRobot goal. Kept separate so tests can tick it against a fake
    undock server.

    Args:
        node: Node that owns the action client.
        tf_buffer: tf2 buffer for the dock-proximity check.
        docks: Dock letter to `mission.docks.Dock`.
        position_writer: Blackboard client with write access to
            `BB_CURRENT_POSITION`.

    Returns:
        The "Not docked?" selector.
    """

    def mark_undocked():
        if position_writer.exists(BB_CURRENT_POSITION):
            position_writer.unset(BB_CURRENT_POSITION)

    def undock_goal():
        """Returns an UndockRobot goal with `dock_type` set.

        Without `dock_type` the server falls back to the dock type from its
        last docking request, which is empty after a docking_server restart.
        Setting it removes that dependency. It does not make undocking work
        after such a restart: opennav_docking still aborts with "Robot is not
        in the dock", because the dock pose it stashed was lost too. Verified
        on 2026-10-02 against the running stack.
        """
        goal = UndockRobot.Goal()
        goal.dock_type = DOCK_PLUGIN_TYPE
        return goal

    def describe_undock():
        try:
            where = position_writer.get(BB_CURRENT_POSITION)
        except Exception:  # Nothing recorded: the TF check found a dock.
            where = None
        return f"undocking from {where.split(':')[-1]}" if where else "undocking"

    undock = CallAction(
        "UndockRobot", node=node, action_type=UndockRobot,
        action_name="undock_robot", goal_builder=undock_goal, on_success=mark_undocked,
        log_tag="undock", describe=describe_undock,
    )
    return py_trees.composites.Selector(
        name="Not docked?", memory=False,
        children=[_not_docked_anywhere("NotDockedAnywhere?", tf_buffer, docks), undock])


def build_goto_dock_subtree(node, dock_key, docks, tf_buffer):
    """Builds the subtree that drives to a dock and docks.

    The dock is read from the blackboard at tick time, so one definition
    serves both the load and deliver branches.

    Args:
        node: Node that owns the action clients.
        dock_key: `BB_SOURCE` or `BB_DESTINATION`.
        docks: Dock letter to `mission.docks.Dock`.
        tf_buffer: Shared tf2 buffer, passed on to `build_ensure_undocked`.

    Returns:
        The "GoToDock" selector.
    """
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

    def describe_navigate():
        dock = docks[position_writer.get(dock_key)]
        return (f"going to the pre-dock pose of dock {dock.letter} "
                f"({dock.staging_pose['x']:.2f}, {dock.staging_pose['y']:.2f})")

    def describe_dock():
        return f"docking at {docks[position_writer.get(dock_key)].letter}"

    navigate = CallAction(
        "NavigateToPose(staging)", node=node, action_type=NavigateToPose,
        action_name="navigate_to_pose", goal_builder=navigate_goal, on_success=mark_predock,
        log_tag="nav", describe=describe_navigate,
    )
    dock_is_free = WaitUntil(
        "DockIsFree?", condition=lambda: True, timeout_sec=60.0, poll_sec=1.0,
    )  # Stub: real multi-robot occupancy will come from perception.
    dock_action = CallAction(
        "DockRobot", node=node, action_type=DockRobot,
        action_name="dock_robot", goal_builder=dock_goal, on_success=mark_dock,
        log_tag="dock", describe=describe_dock,
    )

    # A docked robot sits so close to the wall that its own cell is lethal in
    # the global costmap and Nav2 cannot plan away from it. Undock first, using
    # local control; skipped when neither the blackboard nor TF says docked.
    ensure_undocked = build_ensure_undocked(node, tf_buffer, docks, position_writer)

    move_and_dock = py_trees.composites.Sequence(name="Move and dock", memory=True, children=[
        ensure_undocked,
        # A failed Nav2 goal to the pre-dock pose does not stop docking:
        # tag_dock_server checks for itself whether the pose is occupied and
        # drives in past it.
        py_trees.composites.Selector(name="At pre-dock?", memory=False, children=[
            at_predock,
            py_trees.composites.Selector(name="Try the pre-dock", memory=True, children=[
                navigate,
                py_trees.behaviours.Success(name="Let DockRobot handle it"),
            ]),
        ]),
        dock_is_free,
        dock_action,
    ])

    return py_trees.composites.Selector(name="GoToDock", memory=False, children=[at_dock, move_and_dock])


def build_mission_tree(node, saga_client, docks):
    """Builds the full mission tree.

    Args:
        node: Node that owns the clients, services and TF listener.
        saga_client: Client for the Saga event server.
        docks: Dock letter to `mission.docks.Dock`.

    Returns:
        The root "Mission" sequence.
    """
    # One shared buffer and listener avoids duplicate /tf subscriptions.
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

    # In py_trees 2.5 a Sequence with memory=False is the old ReactiveSequence:
    # it re-ticks the guards every tick while `route` (memory=True) keeps its
    # own progress.
    mission = py_trees.composites.Sequence(name="Mission", memory=False, children=[
        vehicle_status_ok, localize, route,
    ])

    return mission

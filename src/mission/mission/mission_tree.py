import logging
import math
import time

import py_trees
from geometry_msgs.msg import Twist

from mission.bt_behaviours import BlackboardFlagTrue, CallAction, SetBlackboardFlag, WaitUntil
from nav2_msgs.action import DockRobot, NavigateToPose

_logger = logging.getLogger('mission.mission_tree')

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
    """Publishes a single zero-velocity Twist to /cmd_vel."""

    def __init__(self, name, node):
        super().__init__(name)
        self._publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        self._publisher.publish(Twist())
        return py_trees.common.Status.SUCCESS


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


def build_goto_dock_subtree(node, dock_key, docks):
    """dock_key is BB_SOURCE or BB_DESTINATION — resolved at tick time, so the
    same subtree definition serves both the Load and Deliver branches."""

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

    move_and_dock = py_trees.composites.Sequence(name="Move and dock", memory=True, children=[
        py_trees.composites.Selector(name="At pre-dock?", memory=False, children=[at_predock, navigate]),
        dock_is_free,
        dock_action,
    ])

    return py_trees.composites.Selector(name="GoToDock", memory=False, children=[at_dock, move_and_dock])


def build_mission_tree(node, saga_client, docks):
    vehicle_status_ok = py_trees.behaviours.Success(name="VehicleStatusOK (stub)")
    localize = py_trees.behaviours.Success(name="Localize (stub)")

    have_route = py_trees.composites.Selector(name="Have route", memory=False, children=[
        HasRoute("HasRoute?"),
        GetRouteFromSaga("GetRouteFromSaga", saga_client),
    ])

    load = py_trees.composites.Selector(name="Load", memory=False, children=[
        BlackboardFlagTrue("HoldsPackage?", key=BB_HOLDS_PACKAGE),
        py_trees.composites.Sequence(name="Load at source", memory=True, children=[
            build_goto_dock_subtree(node, BB_SOURCE, docks),
            WaitUntil("SagaLoaded?", condition=_saga_status_is(saga_client, "loading_complete"),
                      timeout_sec=300.0, poll_sec=1.0),
            SetBlackboardFlag("Set HoldsPackage", key=BB_HOLDS_PACKAGE, value=True),
        ]),
    ])

    deliver = py_trees.composites.Selector(name="Deliver", memory=False, children=[
        BlackboardFlagTrue("PackageDelivered?", key=BB_PACKAGE_DELIVERED),
        py_trees.composites.Sequence(name="Unload at destination", memory=True, children=[
            build_goto_dock_subtree(node, BB_DESTINATION, docks),
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

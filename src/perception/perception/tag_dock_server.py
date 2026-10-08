"""`/dock_robot`: dock at a dock measured from its AprilTag on the way in.

Started by `perception/launch/docking.launch.py`, which moves opennav_docking's
own docking_server action to `/docking_server/dock_robot` so that THIS node
answers `/dock_robot`. Callers (the mission, `ros2 action send_goal`) keep
using the usual name and goal, and always get the measured, aligned approach:

    ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \\
      "{use_dock_id: true, dock_id: 'E', navigate_to_staging_pose: true}" --feedback

A goal with `use_dock_id: false` (an explicit dock_pose) is passed straight
through to docking_server unchanged.

The process -- Nav2 to the pre-dock pose in the middle of the acceptable box, a
check that the dock bay is free (waiting and checking again, without limit,
while it is not), a measurement of that dock's tag, the measured pose written
back to docks.yaml and docking.yaml, Nav2 turns the robot round, then
docking_server reverses it in at the slow speed set in docking.yaml -- its tolerances and why they are what they
are, are documented in `perception.tag_docking`'s module docstring. Parameters
live in the `tag_dock_server:` block of `perception/config/docking.yaml`.

Feedback reuses DockRobot's states: NAV_TO_STAGING_POSE while driving to the
pre-dock pose, INITIAL_PERCEPTION while checking the bay and measuring the tag
at the pre-dock pose, then whatever docking_server reports for the final drive.
"""

import math
import os
import contextlib
import threading
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from std_srvs.srv import SetBool

from perception.dock_capture import (DOCKING_CHASSIS_OFFSET_M,
                                     DockCaptureError, Pose2D, apply_capture,
                                     normalize_angle,
                                     capture_from_docking_pose,
                                     capture_from_tag_transform,
                                     find_workspace_source_file, tag_tf_frame)
from perception.tag_docking import (PREDOCK_STANDOFF_M, MeasurementWindow,
                                    Tolerance, bay_occupied, compose,
                                    correction, load_dock_database,
                                    occupied_cell_points, points_in_dock_bay,
                                    points_near,
                                    predock_error,
                                    predock_pose_for, yaw_from_quaternion)

Feedback = DockRobot.Feedback
Result = DockRobot.Result

#: Poll period of the execute loop while waiting on child actions.
POLL_SEC = 0.05


class _Abort(Exception):
    """End the goal with this DockRobot error code and message."""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class _Canceled(Exception):
    """The goal was canceled by its caller."""


def _transform_to_pose2d(transform) -> Pose2D:
    t = transform.transform.translation
    q = transform.transform.rotation
    return Pose2D(x=t.x, y=t.y, theta=yaw_from_quaternion(q.x, q.y, q.z, q.w))


def _seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec / 1e9


def _pose_stamped(pose: Pose2D, frame: str, node: Node):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = pose.x
    msg.pose.position.y = pose.y
    msg.pose.orientation.z = math.sin(pose.theta / 2.0)
    msg.pose.orientation.w = math.cos(pose.theta / 2.0)
    return msg


def _fmt(pose: Pose2D) -> str:
    return f'({pose.x:.3f}, {pose.y:.3f}, {math.degrees(pose.theta):.1f} deg)'


class TagDockServer(Node):

    def __init__(self):
        super().__init__('tag_dock_server')

        default_config = os.path.join(
            get_package_share_directory('perception'), 'config', 'docking.yaml')
        declare = self.declare_parameter
        declare('docking_config', default_config)
        declare('detections_topic', '/detections')
        declare('map_frame', 'map')
        declare('odom_frame', 'odom')
        declare('base_frame', 'base_footprint')
        declare('action_name', 'dock_robot')
        declare('navigate_action', 'navigate_to_pose')
        declare('docking_action', 'docking_server/dock_robot')
        declare('undock_action_name', 'undock_robot')
        declare('undocking_action', 'docking_server/undock_robot')
        declare('dock_type', 'arena_dock')
        declare('predock_standoff_m', PREDOCK_STANDOFF_M)
        declare('max_tag_range_m', 2.0)
        declare('max_detection_age_sec', 0.5)
        declare('samples_required', 3)
        declare('sample_spread_m', 0.04)
        declare('sample_spread_rad', 0.06)
        declare('sample_window_sec', 2.0)
        declare('perception_timeout_sec', 3.0)
        declare('gate_service', '/image_gate/enable')
        declare('predock_lateral_tolerance_m', 0.03)
        declare('predock_min_tag_distance_m', 0.5)
        declare('predock_max_tag_distance_m', 1.0)
        declare('predock_heading_tolerance_rad', 0.05)
        declare('max_predock_attempts', 4)
        declare('bay_costmap_topic', 'local_costmap/costmap')
        declare('bay_occupied_value', 100)
        declare('dock_occupied_wait_sec', 10.0)
        declare('bay_min_out_m', 0.15)
        declare('bay_max_out_m', 0.55)
        declare('bay_half_width_m', 0.15)
        declare('bay_min_cells', 2)
        declare('bay_window_sec', 1.5)
        declare('save_measured_pose', True)
        # When something stands on the pre-dock pose: skip it and drive in
        # to the dock pose with Nav2.
        declare('predock_fallback', True)
        declare('predock_clear_radius_m', 0.30)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        config_path = p('docking_config')
        with open(config_path, encoding='utf-8') as stream:
            self._database = load_dock_database(stream.read())
        self._map = p('map_frame')
        self._odom = p('odom_frame')
        self._base = p('base_frame')
        self._dock_type = p('dock_type')
        self._standoff = float(p('predock_standoff_m'))
        self._max_range = float(p('max_tag_range_m'))
        self._max_age = float(p('max_detection_age_sec'))
        self._perception_timeout = float(p('perception_timeout_sec'))
        self._tag_distance_min = float(p('predock_min_tag_distance_m'))
        self._tag_distance_max = float(p('predock_max_tag_distance_m'))
        self._tolerance = Tolerance.from_tag_distance(
            float(p('predock_lateral_tolerance_m')),
            float(p('predock_heading_tolerance_rad')),
            float(p('predock_min_tag_distance_m')),
            float(p('predock_max_tag_distance_m')),
            self._standoff + DOCKING_CHASSIS_OFFSET_M)
        self._max_attempts = int(p('max_predock_attempts'))
        self._occupied_wait = float(p('dock_occupied_wait_sec'))
        self._bay_min_out = float(p('bay_min_out_m'))
        self._bay_max_out = float(p('bay_max_out_m'))
        self._bay_half_width = float(p('bay_half_width_m'))
        self._bay_min_cells = int(p('bay_min_cells'))
        self._bay_occupied_value = int(p('bay_occupied_value'))
        self._bay_window = float(p('bay_window_sec'))
        self._save_measured_pose = bool(p('save_measured_pose'))
        # The dock the last docking reached with Nav2 alone (the fallback), or
        # None. docking_server knows nothing of such a dock, so it cannot undock
        # from it: an undock request just clears this and succeeds, and the
        # next navigation goal drives the robot away.
        self._nav2_docked = None
        self._fallback = bool(p('predock_fallback'))
        self._predock_radius = float(p('predock_clear_radius_m'))
        # Where costmaps are collected while the bay is being checked; None the
        # rest of the time, so the subscription costs nothing while idle.
        self._costmap_sink = None

        self._lock = threading.Lock()
        self._target_tag = None
        self._window = MeasurementWindow(
            int(p('samples_required')), float(p('sample_spread_m')),
            float(p('sample_spread_rad')), float(p('sample_window_sec')))
        self._busy = False

        group = ReentrantCallbackGroup()
        # TF is only listened to while a `_tf_session()` is open (see there).
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self._tf_users = 0
        self._tf_node = None
        self._tf_listener = None
        self._tf_guard = threading.Lock()
        self.create_subscription(AprilTagDetectionArray, p('detections_topic'),
                                 self._on_detections, 10, callback_group=group)
        self._gate_client = self.create_client(SetBool, p('gate_service'),
                                               callback_group=group)
        self._nav_client = ActionClient(self, NavigateToPose, p('navigate_action'),
                                        callback_group=group)
        self._dock_client = ActionClient(self, DockRobot, p('docking_action'),
                                         callback_group=group)
        self._undock_client = ActionClient(self, UndockRobot, p('undocking_action'),
                                           callback_group=group)
        # The local costmap, which is what the lidar and the camera both mark.
        # Transient local, like the costmap publisher, so the last one arrives
        # at once.
        self.create_subscription(
            OccupancyGrid, p('bay_costmap_topic'), self._on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group)
        self._action_name = p('action_name')
        self._server = ActionServer(
            self, DockRobot, self._action_name, self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

        self._undock_server = ActionServer(
            self, UndockRobot, p('undock_action_name'), self._execute_undock,
            goal_callback=lambda _: (GoalResponse.REJECT if self._busy
                                     else GoalResponse.ACCEPT),
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

        self.get_logger().info(
            f'/{self._action_name} ready: {len(self._database)} docks from '
            f'{config_path}; pre-dock {self._standoff:.3f} m out, tolerance lateral '
            f'+-{self._tolerance.lateral_m:.2f} m / '
            f'{self._tag_distance_min:.2f}-{self._tag_distance_max:.2f} m from the tag '
            f'/ heading {math.degrees(self._tolerance.heading_rad):.0f} deg')

    # ------------------------------------------------------------------ TF

    @contextlib.contextmanager
    def _tf_session(self, warmup_sec: float = 2.0):
        """Listen to /tf only inside this block.

        /tf is ~80 msgs/s; decoding them in Python cost 30-50% of a core all the
        time, although this node needs TF only to read the robot pose and to
        measure a tag -- not while Nav2 or a behaviour drives the robot. The
        listener runs in its own node and thread (so its wake-ups never walk
        this node's action clients), on a fresh buffer.
        """
        with self._tf_guard:
            if self._tf_users == 0:
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_node = rclpy.create_node(
                    'tag_dock_server_tf',
                    parameter_overrides=[Parameter(
                        'use_sim_time', value=self._tf_use_sim_time)])
                self._tf_listener = tf2_ros.TransformListener(
                    self._tf_buffer, self._tf_node, spin_thread=True)
            self._tf_users += 1
        try:
            deadline = time.monotonic() + warmup_sec
            while time.monotonic() < deadline:
                if self._tf_buffer.can_transform(self._map, self._base, Time()):
                    break
                time.sleep(0.05)
            yield
        finally:
            with self._tf_guard:
                self._tf_users -= 1
                if self._tf_users == 0:
                    listener, node = self._tf_listener, self._tf_node
                    self._tf_listener = self._tf_node = None
                    listener.executor.shutdown()
                    listener.dedicated_listener_thread.join(timeout=2.0)
                    listener.unregister()
                    node.destroy_node()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _lookup(self, parent, child) -> Pose2D:
        return _transform_to_pose2d(
            self._tf_buffer.lookup_transform(parent, child, Time()))

    def _robot_in_map(self) -> Pose2D:
        try:
            return self._lookup(self._map, self._base)
        except tf2_ros.TransformException as error:
            raise _Abort(Result.UNKNOWN, f'no robot pose in {self._map}: {error}')

    def _odom_to_map(self, pose: Pose2D) -> Pose2D:
        return compose(self._lookup(self._map, self._odom), pose)

    # --------------------------------------------------------- measurement

    def _on_detections(self, msg):
        with self._lock:
            tag = self._target_tag
        if tag is None or self._now() - _seconds(msg.header.stamp) > self._max_age:
            return
        for detection in msg.detections:
            if detection.id != tag:
                continue
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._odom, tag_tf_frame(detection.family, detection.id), Time())
                robot = self._lookup(self._odom, self._base)
            except tf2_ros.TransformException as error:
                self.get_logger().warning(f'tag {tag} seen but no TF: {error}',
                                          throttle_duration_sec=2.0)
                return
            # "Latest available" may be an old sighting still in the buffer.
            if self._now() - _seconds(transform.header.stamp) > self._max_age:
                return
            t = transform.transform.translation
            if math.hypot(t.x - robot.x, t.y - robot.y) > self._max_range:
                return
            q = transform.transform.rotation
            try:
                capture = capture_from_tag_transform(
                    tag, (t.x, t.y), (q.x, q.y, q.z, q.w))
            except DockCaptureError as error:
                self.get_logger().warning(f'tag {tag}: unusable detection: {error}',
                                          throttle_duration_sec=2.0)
                return
            with self._lock:
                if self._target_tag == tag:
                    self._window.add(capture.docking_pose, self._now())
            return

    def _stable_measurement_in_map(self):
        """Agreed-upon dock pose in map from the measurement window, or None."""
        with self._lock:
            in_odom = self._window.stable(self._now())
        if in_odom is None:
            return None
        try:
            return self._odom_to_map(in_odom)
        except tf2_ros.TransformException:
            return None

    # -------------------------------------------------------------- action

    def _on_goal(self, goal):
        if self._busy:
            self.get_logger().warning('rejecting dock goal: already docking')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        self._busy = True
        request = goal_handle.request
        result = Result()
        try:
            if not request.use_dock_id:
                result = self._passthrough(goal_handle, request)
                if result.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            dock = self._database.get(request.dock_id)
            if dock is None:
                raise _Abort(Result.DOCK_NOT_IN_DB,
                             f'dock {request.dock_id!r} is not in docking.yaml')
            self._nav2_docked = None
            with self._lock:
                self._target_tag = dock.tag_id
                self._window.clear()
            result = self._run(goal_handle, dock, request)
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except _Canceled:
            self.get_logger().info('dock goal canceled')
            goal_handle.canceled()
            result = Result(success=False)
        except _Abort as error:
            self.get_logger().error(f'dock goal aborted: {error}')
            goal_handle.abort()
            result = Result(success=False, error_code=error.code,
                            error_msg=str(error))
        finally:
            with self._lock:
                self._target_tag = None
                self._window.clear()
            self._busy = False
        return result

    def _run(self, goal_handle, dock, request) -> Result:
        log = self.get_logger()
        estimate, measured = dock.pose, False
        log.info(f'dock {dock.letter}: stored pose {_fmt(dock.pose)}, '
                 f'looking for tag {dock.tag_id}')

        # Step 1: Nav2 to the pre-dock pose, until the robot is in the box. The
        # detector is off while driving.
        if self._fallback and self._predock_taken(goal_handle, dock, estimate):
            return self._dock_around_blocked_predock(goal_handle, dock, estimate, request)
        try:
            self._reach_predock_box(goal_handle, dock, estimate, request)
        except _Abort as error:
            # Nav2 could not get there: if that is because something stands on
            # the pre-dock pose (now in view), drive in around it instead.
            if not (self._fallback and error.code == Result.FAILED_TO_STAGE
                    and self._predock_taken(goal_handle, dock, estimate)):
                raise
            return self._dock_around_blocked_predock(goal_handle, dock, estimate, request)

        # Steps 2-3, standing in the box: wait for the bay to be free, then
        # measure the tag (detector on only for that) and save the dock pose.
        for attempt in range(1, self._max_attempts + 1):
            self._wait_for_free_bay(goal_handle, dock, estimate)
            goal_handle.publish_feedback(Feedback(state=Feedback.INITIAL_PERCEPTION))
            with self._tf_session():
                final = self._measure_still(goal_handle)
                robot = self._robot_in_map()
            if final is None:
                log.warning(f'dock {dock.letter}: tag {dock.tag_id} not seen, '
                            f'docking against the '
                            f'{"last measured" if measured else "stored"} pose')
                break
            dx, dy, dth = correction(dock.pose, final)
            log.info(f'dock {dock.letter}: tag {dock.tag_id} measured: dock pose '
                     f'{_fmt(final)}, correction vs stored dx {dx:+.3f} '
                     f'dy {dy:+.3f} m, dtheta {math.degrees(dth):+.1f} deg')
            estimate, measured = final, True
            error = predock_error(robot, predock_pose_for(estimate, self._standoff))
            summary = (f'lateral {error.lateral:+.3f} m, along {error.along:+.3f} m, '
                       f'heading {math.degrees(error.heading):+.1f} deg')
            if self._tolerance.accepts(error):
                log.info(f'dock {dock.letter}: in the pre-dock box of the measured '
                         f'dock ({summary})')
                self._save_pose(dock, final)
                break
            if attempt == self._max_attempts:
                raise _Abort(Result.FAILED_TO_STAGE,
                             f'outside the pre-dock box of the measured dock '
                             f'({summary}) after {attempt} measurements')
            # The measurement moved the box (the stored pose was off): let Nav2
            # take the robot to the measured pre-dock pose, then measure again.
            log.info(f'dock {dock.letter}: outside the measured pre-dock box '
                     f'({summary}), navigating there')
            self._reach_predock_box(goal_handle, dock, estimate, request)

        # Something may have arrived while measuring; check once more before
        # driving in.
        self._wait_for_free_bay(goal_handle, dock, estimate)

        # Step 4: turn round (Nav2) onto the staging pose, then Nav2's docking
        # controller reverses in, at the speed limits in docking.yaml.
        self._turn_to_staging(goal_handle, dock, estimate, request)
        return self._dock(goal_handle, dock, estimate)

    def _predock_taken(self, goal_handle, dock, estimate: Pose2D) -> bool:
        """Whether something stands on the pre-dock pose (False when unknown).

        Only asked while the robot is outside the pre-dock box. No costmap, or
        a pre-dock pose outside the local costmap, counts as not taken: the
        normal path is tried and the question asked again if Nav2 fails.
        """
        if self._in_predock_box(estimate)[0]:
            return False
        predock = predock_pose_for(estimate, self._standoff)
        snapshots = self._costmap_snapshots(goal_handle)
        if not snapshots:
            return False
        counts = [points_near(points, predock, self._predock_radius)
                  for points in snapshots]
        taken = bay_occupied(counts, self._bay_min_cells)
        self.get_logger().info(
            f'dock {dock.letter}: occupied costmap cells within '
            f'{self._predock_radius:.2f} m of the pre-dock pose per update {counts}'
            f' -> {"taken" if taken else "free"}')
        return bool(taken)

    def _dock_around_blocked_predock(self, goal_handle, dock, estimate: Pose2D,
                                     request) -> Result:
        """Pre-dock pose taken: check the bay, then Nav2 straight to the dock pose.

        Only Nav2 (NavigateToPose) drives, facing the dock, to the docked pose,
        ending wherever its goal tolerance allows. No pre-dock box, no tag
        measurement, no docking_server.
        """
        self.get_logger().warning(
            f'dock {dock.letter}: obstacle on the pre-dock pose; docking with '
            f'Nav2 only: navigating straight to the dock pose {_fmt(estimate)}')
        self._wait_for_free_bay(goal_handle, dock, estimate)
        deadline = time.monotonic() + float(request.max_staging_time)
        self._navigate(goal_handle, estimate, deadline, what='the dock pose')
        self.get_logger().info(f'dock {dock.letter}: reached the dock pose')
        self._nav2_docked = dock
        return Result(success=True)

    def _turn_to_staging(self, goal_handle, dock, estimate: Pose2D, request):
        """Turn the robot round so it can back into the dock.

        The robot stands in the pre-dock box facing the dock (to see the tag).
        docking_server reverses in, from the staging pose: the pre-dock
        position, facing AWAY from the dock. docking_server only navigates to
        its staging pose when the robot is farther than dock_prestaging_tolerance
        from it IN POSITION, ignoring heading, so a robot already at the spot,
        facing the dock, would be left as it is. Nav2 is sent there instead,
        which also turns it.
        """
        predock = predock_pose_for(estimate, self._standoff)
        staging = Pose2D(predock.x, predock.y,
                         normalize_angle(predock.theta + math.pi))
        self.get_logger().info(
            f'dock {dock.letter}: turning onto the staging pose {_fmt(staging)} '
            f'to back in')
        deadline = time.monotonic() + float(request.max_staging_time)
        self._navigate(goal_handle, staging, deadline)

    def _in_predock_box(self, estimate: Pose2D):
        """(whether the robot is in the pre-dock box of `estimate`, a summary)."""
        with self._tf_session():
            robot = self._robot_in_map()
        error = predock_error(robot, predock_pose_for(estimate, self._standoff))
        summary = (f'lateral {error.lateral:+.3f} m, along {error.along:+.3f} m, '
                   f'heading {math.degrees(error.heading):+.1f} deg')
        return self._tolerance.accepts(error), summary

    def _reach_predock_box(self, goal_handle, dock, estimate: Pose2D, request):
        """Send Nav2 to the pre-dock pose until the robot is inside the box."""
        log = self.get_logger()
        for attempt in range(1, self._max_attempts + 1):
            inside, summary = self._in_predock_box(estimate)
            if inside:
                log.info(f'dock {dock.letter}: in the pre-dock box ({summary})')
                return
            if not request.navigate_to_staging_pose:
                log.warning(f'dock {dock.letter}: not in the pre-dock box ({summary}),'
                            f' navigating there although navigate_to_staging_pose '
                            f'is false')
            log.info(f'dock {dock.letter}: navigating to the pre-dock pose '
                     f'(attempt {attempt}, {summary})')
            deadline = time.monotonic() + float(request.max_staging_time)
            self._navigate(goal_handle, predock_pose_for(estimate, self._standoff),
                           deadline)
        inside, summary = self._in_predock_box(estimate)
        if not inside:
            raise _Abort(Result.FAILED_TO_STAGE,
                         f'could not get into the pre-dock box after '
                         f'{self._max_attempts} navigation goals ({summary})')

    # ------------------------------------------------------------ bay check

    def _on_costmap(self, msg):
        with self._lock:
            sink = self._costmap_sink
            if sink is not None:
                sink.append(msg)

    def _sleep(self, goal_handle, seconds: float):
        """Sleep, but notice a cancel."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise _Canceled()
            if not rclpy.ok():
                raise _Abort(Result.UNKNOWN, 'shutting down')
            time.sleep(POLL_SEC)

    def _costmap_snapshots(self, goal_handle):
        """Occupied cells (map frame) of each local costmap update over the window.

        Reads the local costmap, so what the lidar sees (obstacles in its plane)
        and what the camera sees (low obstacles under it) both count, and so
        does anything Nav2 itself would treat as an obstacle. None when no
        costmap arrived.
        """
        grids = []
        with self._lock:
            self._costmap_sink = grids
        try:
            self._sleep(goal_handle, self._bay_window)
        finally:
            with self._lock:
                self._costmap_sink = None
        if not grids:
            return None
        snapshots = []
        with self._tf_session():
            for grid in grids:
                try:
                    grid_in_map = self._lookup(self._map, grid.header.frame_id)
                except tf2_ros.TransformException as error:
                    self.get_logger().warning(
                        f'costmap check: no {self._map}<-{grid.header.frame_id} '
                        f'transform: {error}', throttle_duration_sec=2.0)
                    continue
                info = grid.info
                cells = occupied_cell_points(
                    grid.data, info.width, info.height, info.resolution,
                    info.origin.position.x, info.origin.position.y,
                    self._bay_occupied_value)
                points = []
                for x, y in cells:
                    point = compose(grid_in_map, Pose2D(x=x, y=y, theta=0.0))
                    points.append((point.x, point.y))
                snapshots.append(points)
        return snapshots or None

    def _bay_is_occupied(self, goal_handle, docking_pose: Pose2D):
        """True / False, or None when no costmap arrived, for the dock bay."""
        snapshots = self._costmap_snapshots(goal_handle)
        if not snapshots:
            return None
        counts = [points_in_dock_bay(
            points, docking_pose, self._bay_min_out, self._bay_max_out,
            self._bay_half_width) for points in snapshots]
        self.get_logger().info(
            f'bay check: occupied costmap cells inside the bay per update {counts} '
            f'(occupied when half the updates have >= {self._bay_min_cells})')
        return bay_occupied(counts, self._bay_min_cells)

    def _wait_for_free_bay(self, goal_handle, dock, estimate: Pose2D):
        """Return once the dock bay is free; while it is not, wait and re-check.

        No limit on the number of checks: a dock another robot is using is
        simply waited for. Cancel the goal to stop. No costmap data counts as not
        free.
        """
        log = self.get_logger()
        checks = 0
        while True:
            occupied = self._bay_is_occupied(goal_handle, estimate)
            if occupied is False:
                if checks:
                    log.info(f'dock {dock.letter}: bay is free again after '
                             f'{checks} waits')
                return
            checks += 1
            reason = ('no costmap received on '
                      f'{self.get_parameter("bay_costmap_topic").value}'
                      if occupied is None else 'obstacle (or another robot) in the bay')
            log.warning(f'dock {dock.letter}: {reason}, not docking; checking '
                        f'again in {self._occupied_wait:.0f} s (wait {checks})')
            goal_handle.publish_feedback(Feedback(state=Feedback.INITIAL_PERCEPTION))
            self._sleep(goal_handle, self._occupied_wait)

    # ------------------------------------------------------- saving the pose

    def _save_pose(self, dock, docking_pose: Pose2D):
        """Write the measured dock pose into docks.yaml and docking.yaml.

        The workspace SOURCE files, as `capture_dock_pose` does (an installed
        copy would be overwritten by the next build), so it takes effect after
        the next `colcon build`; this goal docks against the measurement
        regardless. A failure here is logged and does not stop the docking.
        """
        if not self._save_measured_pose:
            return
        log = self.get_logger()
        try:
            capture = capture_from_docking_pose(dock.tag_id, docking_pose)
            if capture.letter != dock.letter:
                raise DockCaptureError(
                    f'tag {dock.tag_id} belongs to dock {capture.letter}, not '
                    f'{dock.letter}')
            share = get_package_share_directory('perception')
            docks_path = find_workspace_source_file(share, 'mission/config/docks.yaml')
            docking_path = find_workspace_source_file(
                share, 'perception/config/docking.yaml')
            if docks_path is None or docking_path is None:
                log.warning(f'dock {dock.letter}: no workspace source tree next to '
                            f'{share}; measured pose not saved')
                return
            apply_capture(capture, docks_path, docking_path)
            log.info(f'dock {dock.letter}: saved the measured pose to {docks_path} '
                     f'and {docking_path} (applies after the next colcon build)')
        except (DockCaptureError, OSError) as error:
            log.warning(f'dock {dock.letter}: could not save the measured pose: '
                        f'{error}')

    # --------------------------------------------------------- navigation

    def _wait(self, goal_handle, future, child_handle=None):
        while not future.done():
            if goal_handle.is_cancel_requested:
                if child_handle is not None:
                    child_handle.cancel_goal_async()
                raise _Canceled()
            if not rclpy.ok():
                raise _Abort(Result.UNKNOWN, 'shutting down')
            time.sleep(POLL_SEC)
        return future.result()

    def _send_nav(self, goal_handle, predock: Pose2D):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            raise _Abort(Result.FAILED_TO_STAGE, 'navigate_to_pose server not available')
        goal = NavigateToPose.Goal()
        goal.pose = _pose_stamped(predock, self._map, self)
        handle = self._wait(goal_handle, self._nav_client.send_goal_async(goal))
        if not handle.accepted:
            raise _Abort(Result.FAILED_TO_STAGE, 'navigation goal rejected')
        return handle, handle.get_result_async()

    def _navigate(self, goal_handle, target: Pose2D, deadline: float,
                  what: str = 'pre-dock'):
        """Drive (Nav2 NavigateToPose) to `target`, turning to its heading."""
        self.get_logger().info(f'navigating to the {what} {_fmt(target)}')
        handle, result_future = self._send_nav(goal_handle, target)
        last_feedback = 0.0
        while True:
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                raise _Canceled()
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                raise _Abort(Result.FAILED_TO_STAGE, 'max_staging_time exceeded')
            if result_future.done():
                status = result_future.result().status
                if status != GoalStatus.STATUS_SUCCEEDED:
                    raise _Abort(Result.FAILED_TO_STAGE,
                                 f'navigation to {what} ended with status {status}')
                self.get_logger().info(f'reached the {what}')
                return
            if time.monotonic() - last_feedback > 1.0:
                goal_handle.publish_feedback(
                    Feedback(state=Feedback.NAV_TO_STAGING_POSE))
                last_feedback = time.monotonic()
            time.sleep(POLL_SEC)

    def _set_detector(self, goal_handle, on: bool) -> bool:
        """Open or close the AprilTag image gate; False if there is none.

        Without a gate (apriltag ungated, or not running) detections, if any,
        flow all the time, so measuring still works as before.
        """
        if not self._gate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                f'{self._gate_client.srv_name} not available: detector gating '
                f'off', throttle_duration_sec=30.0)
            return False
        request = SetBool.Request(data=on)
        self._wait(goal_handle, self._gate_client.call_async(request))
        return True

    def _measure_still(self, goal_handle):
        """A fresh agreed-upon tag measurement at the pre-dock pose, or None.

        The detector is switched on for just this wait.
        """
        with self._lock:
            self._window.clear()
        gated = self._set_detector(goal_handle, True)
        try:
            deadline = time.monotonic() + self._perception_timeout
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    raise _Canceled()
                measured = self._stable_measurement_in_map()
                if measured is not None:
                    return measured
                time.sleep(POLL_SEC)
            return None
        finally:
            if gated:
                self._close_detector()

    def _close_detector(self):
        """Best-effort close that also works while the goal is being canceled."""
        if self._gate_client.service_is_ready():
            self._gate_client.call_async(SetBool.Request(data=False))

    def _passthrough(self, goal_handle, request) -> Result:
        """Forward an explicit-pose goal to docking_server unchanged."""
        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            raise _Abort(Result.UNKNOWN, 'docking_server not available')

        def forward(feedback_msg):
            goal_handle.publish_feedback(feedback_msg.feedback)

        handle = self._wait(goal_handle, self._dock_client.send_goal_async(
            request, feedback_callback=forward))
        if not handle.accepted:
            raise _Abort(Result.UNKNOWN, 'docking goal rejected by docking_server')
        response = self._wait(goal_handle, handle.get_result_async(), handle)
        return response.result

    # ------------------------------------------------------------ undocking

    def _execute_undock(self, goal_handle):
        """/undock_robot: docking_server backs the robot out to its staging pose."""
        self._busy = True
        result = UndockRobot.Result()
        try:
            if self._nav2_docked is not None:
                self.get_logger().info(
                    f'undock: dock {self._nav2_docked.letter} was reached with Nav2 '
                    f'alone, so docking_server has nothing to undock; now undocked')
                self._nav2_docked = None
                goal_handle.succeed()
                return UndockRobot.Result(success=True)
            if not self._undock_client.wait_for_server(timeout_sec=5.0):
                raise _Abort(Result.UNKNOWN, 'docking_server undock not available')
            handle = self._wait(goal_handle, self._undock_client.send_goal_async(
                goal_handle.request))
            if not handle.accepted:
                raise _Abort(Result.UNKNOWN, 'undock goal rejected by docking_server')
            response = self._wait(goal_handle, handle.get_result_async(), handle)
            result = response.result
            if response.status == GoalStatus.STATUS_SUCCEEDED and result.success:
                goal_handle.succeed()
            else:
                self.get_logger().error(
                    f'undock failed: code {result.error_code} {result.error_msg}')
                goal_handle.abort()
        except _Canceled:
            goal_handle.canceled()
        except _Abort as error:
            self.get_logger().error(f'undock aborted: {error}')
            goal_handle.abort()
            result = UndockRobot.Result(success=False, error_code=error.code,
                                        error_msg=str(error))
        finally:
            self._busy = False
        return result

    # ------------------------------------------------------------- docking

    def _dock(self, goal_handle, dock, estimate: Pose2D) -> Result:
        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            raise _Abort(Result.UNKNOWN, 'docking_server not available')
        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_pose = _pose_stamped(estimate, self._map, self)
        goal.dock_type = self._dock_type
        goal.navigate_to_staging_pose = False
        self.get_logger().info(f'dock {dock.letter}: backing in to {_fmt(estimate)}')

        def forward(feedback_msg):
            goal_handle.publish_feedback(feedback_msg.feedback)

        handle = self._wait(goal_handle, self._dock_client.send_goal_async(
            goal, feedback_callback=forward))
        if not handle.accepted:
            raise _Abort(Result.UNKNOWN, 'docking goal rejected by docking_server')
        response = self._wait(goal_handle, handle.get_result_async(), handle)
        child = response.result
        if response.status == GoalStatus.STATUS_SUCCEEDED and child.success:
            self.get_logger().info(f'dock {dock.letter}: docked')
        else:
            self.get_logger().error(
                f'dock {dock.letter}: docking_server failed: code '
                f'{child.error_code} {child.error_msg}')
        return Result(success=bool(child.success) and
                      response.status == GoalStatus.STATUS_SUCCEEDED,
                      error_code=child.error_code, num_retries=child.num_retries,
                      error_msg=child.error_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TagDockServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

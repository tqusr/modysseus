"""`/dock_robot`: dock at a dock measured from its AprilTag on the way in.

Started by `perception/launch/docking.launch.py`, which moves opennav_docking's
own docking_server action to `/docking_server/dock_robot` so that THIS node
answers `/dock_robot`. Callers (the mission, `ros2 action send_goal`) keep
using the usual name and goal, and always get the measured, aligned approach:

    ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \\
      "{use_dock_id: true, dock_id: 'E', navigate_to_staging_pose: true}" --feedback

A goal with `use_dock_id: false` (an explicit dock_pose) is passed straight
through to docking_server unchanged.

The four-step process, its tolerances and why they are what they are, are
documented in `perception.tag_docking`'s module docstring. Parameters live in
the `tag_dock_server:` block of `perception/config/docking.yaml`.

Feedback reuses DockRobot's states: NAV_TO_STAGING_POSE while driving to the
pre-dock pose (tag measured on the way), INITIAL_PERCEPTION while measuring at
the pre-dock pose, then whatever docking_server reports for the final drive.
"""

import math
import os
import threading
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from nav2_msgs.action import (BackUp, DockRobot, DriveOnHeading,
                              NavigateToPose, Spin)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from perception.dock_capture import (STAGING_STANDOFF_M, DockCaptureError,
                                     Pose2D, capture_from_tag_transform,
                                     tag_tf_frame)
from perception.tag_docking import (FORWARD, SPIN,
                                    MeasurementWindow, Tolerance, compose,
                                    correction, load_dock_database,
                                    needs_replan, plan_alignment,
                                    predock_error, predock_pose_for,
                                    yaw_from_quaternion)

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
        declare('spin_action', 'spin')
        declare('drive_action', 'drive_on_heading')
        declare('backup_action', 'backup')
        declare('dock_type', 'arena_dock')
        declare('predock_standoff_m', STAGING_STANDOFF_M)
        declare('max_tag_range_m', 2.0)
        declare('max_detection_age_sec', 0.5)
        declare('samples_required', 3)
        declare('sample_spread_m', 0.04)
        declare('sample_spread_rad', 0.06)
        declare('sample_window_sec', 2.0)
        declare('perception_timeout_sec', 3.0)
        declare('replan_distance_m', 0.05)
        declare('replan_angle_rad', 0.05)
        declare('align_range_m', 1.0)
        declare('align_speed_mps', 0.08)
        declare('predock_lateral_tolerance_m', 0.03)
        declare('predock_along_tolerance_m', 0.08)
        declare('predock_heading_tolerance_rad', 0.05)
        declare('max_predock_attempts', 4)
        declare('give_up_tolerance_factor', 2.0)

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
        self._replan_m = float(p('replan_distance_m'))
        self._replan_rad = float(p('replan_angle_rad'))
        self._tolerance = Tolerance(float(p('predock_lateral_tolerance_m')),
                                    float(p('predock_along_tolerance_m')),
                                    float(p('predock_heading_tolerance_rad')))
        self._give_up = self._tolerance.scaled(float(p('give_up_tolerance_factor')))
        self._max_attempts = int(p('max_predock_attempts'))
        self._align_range = float(p('align_range_m'))
        self._align_speed = float(p('align_speed_mps'))

        self._lock = threading.Lock()
        self._target_tag = None
        self._window = MeasurementWindow(
            int(p('samples_required')), float(p('sample_spread_m')),
            float(p('sample_spread_rad')), float(p('sample_window_sec')))
        self._busy = False

        group = ReentrantCallbackGroup()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_subscription(AprilTagDetectionArray, p('detections_topic'),
                                 self._on_detections, 10, callback_group=group)
        self._nav_client = ActionClient(self, NavigateToPose, p('navigate_action'),
                                        callback_group=group)
        self._dock_client = ActionClient(self, DockRobot, p('docking_action'),
                                         callback_group=group)
        self._spin_client = ActionClient(self, Spin, p('spin_action'),
                                         callback_group=group)
        self._drive_client = ActionClient(self, DriveOnHeading, p('drive_action'),
                                          callback_group=group)
        self._backup_client = ActionClient(self, BackUp, p('backup_action'),
                                           callback_group=group)
        self._action_name = p('action_name')
        self._server = ActionServer(
            self, DockRobot, self._action_name, self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

        self.get_logger().info(
            f'/{self._action_name} ready: {len(self._database)} docks from '
            f'{config_path}; pre-dock {self._standoff:.2f} m out, aligned within '
            f'{self._align_range:.1f} m by turn-drive-turn, tolerance lateral '
            f'{self._tolerance.lateral_m:.2f} m / along {self._tolerance.along_m:.2f} m '
            f'/ heading {math.degrees(self._tolerance.heading_rad):.0f} deg')

    # ------------------------------------------------------------------ TF

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

        def adopt(new_estimate, where):
            nonlocal estimate, measured
            dx, dy, dth = correction(dock.pose, new_estimate)
            log.info(f'dock {dock.letter}: tag {dock.tag_id} measured {where}: '
                     f'dock pose {_fmt(new_estimate)}, correction vs stored '
                     f'dx {dx:+.3f} dy {dy:+.3f} m, dtheta '
                     f'{math.degrees(dth):+.1f} deg')
            estimate, measured = new_estimate, True

        # Step 1-2: drive near the pre-dock pose, re-aiming at the tag on the
        # way. Skipped when already close: then the robot is typically at a
        # neighbouring dock, and turn-drive-turn below is the exact way over.
        predock = predock_pose_for(estimate, self._standoff)
        robot = self._robot_in_map()
        far = math.hypot(predock.x - robot.x, predock.y - robot.y) > self._align_range
        if far:
            if not request.navigate_to_staging_pose:
                log.warning(f'dock {dock.letter}: robot is more than '
                            f'{self._align_range:.1f} m from the pre-dock pose, '
                            f'navigating there although navigate_to_staging_pose '
                            f'is false')
            deadline = time.monotonic() + float(request.max_staging_time)
            self._navigate(goal_handle, estimate, deadline, on_tag=adopt)
            estimate = self._last_nav_estimate

        # Step 3: measure standing still; align until within tolerance.
        for attempt in range(1, self._max_attempts + 1):
            goal_handle.publish_feedback(Feedback(state=Feedback.INITIAL_PERCEPTION))
            final = self._measure_still(goal_handle)
            if final is not None:
                adopt(final, 'at pre-dock' if attempt > 1 or far else 'before aligning')
            elif not measured and attempt == 1:
                log.warning(f'dock {dock.letter}: tag {dock.tag_id} not seen, '
                            f'docking against the stored pose')
            predock = predock_pose_for(estimate, self._standoff)
            robot = self._robot_in_map()
            error = predock_error(robot, predock)
            summary = (f'lateral {error.lateral:+.3f} m, along {error.along:+.3f} m, '
                       f'heading {math.degrees(error.heading):+.1f} deg')
            if self._tolerance.accepts(error):
                log.info(f'dock {dock.letter}: at pre-dock within tolerance ({summary})')
                break
            if attempt == self._max_attempts:
                if not self._give_up.accepts(error):
                    raise _Abort(Result.FAILED_TO_STAGE,
                                 f'could not reach the pre-dock pose ({summary})')
                log.warning(f'dock {dock.letter}: pre-dock still off ({summary}) '
                            f'after {attempt} alignments; close enough for the '
                            f'docking controller, continuing')
                break
            log.info(f'dock {dock.letter}: aligning to pre-dock ({summary})')
            self._align(goal_handle, robot, predock)

        # Step 4: drive forward into the dock.
        return self._dock(goal_handle, dock, estimate)

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

    def _navigate(self, goal_handle, estimate: Pose2D, deadline: float, on_tag):
        """Drive to estimate's pre-dock pose; with `on_tag`, re-aim at the tag."""
        self._last_nav_estimate = estimate
        target = predock_pose_for(estimate, self._standoff)
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
                                 f'navigation to pre-dock ended with status {status}')
                return
            if on_tag is not None:
                measured = self._stable_measurement_in_map()
                if measured is not None:
                    new_target = predock_pose_for(measured, self._standoff)
                    if needs_replan(target, new_target, self._replan_m, self._replan_rad):
                        on_tag(measured, 'while driving')
                        self._last_nav_estimate = measured
                        target = new_target
                        self.get_logger().info(
                            f'redirecting to measured pre-dock {_fmt(target)}')
                        handle, result_future = self._send_nav(goal_handle, target)
            if time.monotonic() - last_feedback > 1.0:
                goal_handle.publish_feedback(
                    Feedback(state=Feedback.NAV_TO_STAGING_POSE))
                last_feedback = time.monotonic()
            time.sleep(POLL_SEC)

    def _measure_still(self, goal_handle):
        """A fresh agreed-upon tag measurement at the pre-dock pose, or None."""
        with self._lock:
            self._window.clear()
        deadline = time.monotonic() + self._perception_timeout
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                raise _Canceled()
            measured = self._stable_measurement_in_map()
            if measured is not None:
                return measured
            time.sleep(POLL_SEC)
        return None

    # ---------------------------------------------------------- alignment

    def _behavior(self, goal_handle, client, goal, what):
        if not client.wait_for_server(timeout_sec=5.0):
            raise _Abort(Result.FAILED_TO_STAGE, f'{what}: behavior server not available')
        handle = self._wait(goal_handle, client.send_goal_async(goal))
        if not handle.accepted:
            raise _Abort(Result.FAILED_TO_STAGE, f'{what}: goal rejected')
        response = self._wait(goal_handle, handle.get_result_async(), handle)
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise _Abort(Result.FAILED_TO_STAGE,
                         f'{what} failed (status {response.status}, error code '
                         f'{getattr(response.result, "error_code", "?")}) -- '
                         f'blocked by an obstacle?')

    def _align(self, goal_handle, robot: Pose2D, predock: Pose2D):
        """Turn to face the pre-dock pose, drive (or back) to it, turn square."""
        goal_handle.publish_feedback(Feedback(state=Feedback.NAV_TO_STAGING_POSE))
        for kind, amount in plan_alignment(robot, predock):
            if kind == SPIN:
                goal = Spin.Goal(target_yaw=float(amount),
                                 time_allowance=Duration(sec=15))
                self._behavior(goal_handle, self._spin_client, goal,
                               f'spin {math.degrees(amount):+.1f} deg')
                continue
            client, action, label = ((self._drive_client, DriveOnHeading, 'drive')
                                     if kind == FORWARD else
                                     (self._backup_client, BackUp, 'back up'))
            goal = action.Goal(speed=self._align_speed,
                               time_allowance=Duration(
                                   sec=int(10 + amount / self._align_speed * 3)))
            goal.target.x = float(amount)
            self._behavior(goal_handle, client, goal, f'{label} {amount:.3f} m')

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

    # ------------------------------------------------------------- docking

    def _dock(self, goal_handle, dock, estimate: Pose2D) -> Result:
        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            raise _Abort(Result.UNKNOWN, 'docking_server not available')
        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_pose = _pose_stamped(estimate, self._map, self)
        goal.dock_type = self._dock_type
        goal.navigate_to_staging_pose = False
        self.get_logger().info(f'dock {dock.letter}: driving in to {_fmt(estimate)}')

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

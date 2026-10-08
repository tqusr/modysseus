"""tag_dock_server end to end: the whole docking sequence against fake nodes.

The real server runs in this process over real ROS 2 topics, services and
actions, against a rig standing in for the rest of the robot: TF (robot pose and
the dock's tag), the AprilTag detections and image gate, the local costmap (the
dock's wall and wings marked, with inflation), Nav2's `navigate_to_pose` and
opennav_docking's `docking_server`. What is checked is the sequence:

  - Nav2 takes the robot to the pre-dock pose only when it is outside the box;
  - the bay is checked first in the local costmap (lidar and camera both fill
    it), and while another robot or an object is in it nothing else happens (no
    measurement, no docking) however long that takes; cancelling ends the wait;
  - then the tag is measured and the pose written into docks.yaml and
    docking.yaml (temporary copies here; the real source files are checked
    untouched);
  - then Nav2 turns the robot onto the staging pose (the pre-dock position,
    facing away) and docking_server is called with the measured pose; it, not a
    hand-written manoeuvre, reverses the robot in.

It cannot say whether lidar and camera mark a real robot in a real dock, how
Nav2's docking controller behaves at 0.08 m/s on the floor, or how long the real
camera takes to deliver a tag. Those are checks on the robot.

It runs on its own ROS domain (set before rclpy starts), so it neither sees nor
disturbs a robot stack on the same machine.
"""

import math
import os
import shutil
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '93'

import pytest  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

import perception.tag_dock_server as server_module  # noqa: E402
from perception.dock_capture import tag_tf_frame  # noqa: E402
from perception.tag_docking import PREDOCK_STANDOFF_M, predock_pose_for  # noqa: E402
from perception.dock_capture import DOCKING_CHASSIS_OFFSET_M, Pose2D  # noqa: E402
from test_dock_capture import upright_wall_tag_quaternion  # noqa: E402

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCKING_YAML = os.path.join(PKG, 'config', 'docking.yaml')
DOCKS_YAML = os.path.join(PKG, '..', 'mission', 'config', 'docks.yaml')

WAIT_SEC = 0.4          # dock_occupied_wait_sec in these tests
LETTER, TAG_ID = 'B', 1


def _stored_pose():
    with open(DOCKING_YAML) as handle:
        x, y, theta = yaml.safe_load(handle)['docking_server']['ros__parameters'][LETTER]['pose']
    return Pose2D(x, y, theta)


def _wall_pose():
    with open(DOCKS_YAML) as handle:
        dock = yaml.safe_load(handle)['docks'][LETTER]['dock_pose']
    return dock['x'], dock['y'], dock['theta']


def _sha(path):
    with open(path, 'rb') as handle:
        return handle.read()


class Rig(Node):
    """Everything around the server that is not the server."""

    def __init__(self):
        super().__init__('dock_rig')
        group = ReentrantCallbackGroup()
        stored = _stored_pose()
        self.robot = predock_pose_for(stored)              # in the box, facing the dock
        wx, wy, wtheta = _wall_pose()
        self.tag = (wx, wy + 0.02, wtheta)                 # measured 2 cm off the stored one
        self.tag_visible = True
        self.costmap_enabled = True
        self.bay_points = []                                # map-frame obstacle points in the bay
        self.predock_points = []                            # ... and on the pre-dock pose
        self.gate_calls, self.nav_goals, self.dock_goals = [], [], []
        self.undock_calls = 0
        self.events = []                                    # 'nav' / 'dock', in order
        self.nav_moves_robot = True
        self._lock = threading.Lock()

        self._tf = TransformBroadcaster(self)
        self._detections = self.create_publisher(
            AprilTagDetectionArray, '/detections', 10)
        self._costmap = self.create_publisher(
            OccupancyGrid, 'local_costmap/costmap',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.03, self._publish_tf, callback_group=group)
        self.create_timer(0.1, self._publish_detection, callback_group=group)
        self.create_timer(0.25, self._publish_costmap, callback_group=group)
        self.create_service(SetBool, '/image_gate/enable', self._on_gate,
                            callback_group=group)
        self._nav = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self._on_nav,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self._dock = ActionServer(
            self, DockRobot, 'docking_server/dock_robot', self._on_dock,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

        self._undock = ActionServer(
            self, UndockRobot, 'docking_server/undock_robot', self._on_undock,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

    def _transform(self, parent, child, x, y, yaw=0.0, quaternion=None):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.transform.translation.x, msg.transform.translation.y = x, y
        q = quaternion or (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = q
        return msg

    def _publish_tf(self):
        with self._lock:
            robot, tag, visible = self.robot, self.tag, self.tag_visible
        transforms = [self._transform('map', 'odom', 0.0, 0.0),
                      self._transform('odom', 'base_footprint',
                                      robot.x, robot.y, robot.theta),
                      self._transform('base_footprint', 'laser', 0.0, 0.0)]
        if visible:
            transforms.append(self._transform(
                'odom', tag_tf_frame('tag36h11', TAG_ID), tag[0], tag[1],
                quaternion=upright_wall_tag_quaternion(tag[2])))
        self._tf.sendTransform(transforms)

    def _publish_detection(self):
        if not self.tag_visible:
            return
        msg = AprilTagDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        detection = AprilTagDetection()
        detection.family, detection.id = 'tag36h11', TAG_ID
        msg.detections = [detection]
        self._detections.publish(msg)

    def _dock_structure_points(self):
        """The dock's own back wall and flared wings, in the map frame."""
        stored = _stored_pose()
        cos_t, sin_t = math.cos(stored.theta), math.sin(stored.theta)
        local = []                                          # (out from the wall, lateral)
        for i in range(-4, 5):                              # the 0.4 m wide back wall
            local.append((0.0, i * 0.05))
            local.append((0.04, i * 0.05))
        for i in range(0, 4):                               # wings, flared 45 degrees
            local.append((i * 0.05, 0.2 + i * 0.05))
            local.append((i * 0.05, -0.2 - i * 0.05))
        return [(stored.x + (DOCKING_CHASSIS_OFFSET_M - out) * cos_t - lateral * sin_t,
                 stored.y + (DOCKING_CHASSIS_OFFSET_M - out) * sin_t + lateral * cos_t)
                for out, lateral in local]

    def _publish_costmap(self):
        """Returns a 3 x 3 m rolling window around the robot, like the local costmap.

        Obstacle cells are 100 (lethal). Cells within 0.23 m of one are 99
        (inflation), which the bay check must not count. The dock's wall and
        wings are marked and inflated like any obstacle.
        """
        if not self.costmap_enabled:
            return
        size, resolution = 60, 0.05
        with self._lock:
            robot, extra = self.robot, list(self.bay_points) + list(self.predock_points)
        origin_x = round((robot.x - 1.5) / resolution) * resolution
        origin_y = round((robot.y - 1.5) / resolution) * resolution
        lethal = set()
        for x, y in self._dock_structure_points() + extra:
            column = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            if 0 <= column < size and 0 <= row < size:
                lethal.add((row, column))
        data = [0] * (size * size)
        reach = int(0.23 / resolution) + 1
        for row, column in lethal:
            for dr in range(-reach, reach + 1):
                for dc in range(-reach, reach + 1):
                    r, c = row + dr, column + dc
                    if (0 <= r < size and 0 <= c < size
                            and math.hypot(dr, dc) * resolution <= 0.23):
                        data[r * size + c] = max(data[r * size + c], 99)
        for row, column in lethal:
            data[row * size + column] = 100
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = resolution
        msg.info.width = msg.info.height = size
        msg.info.origin.position.x, msg.info.origin.position.y = origin_x, origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        self._costmap.publish(msg)

    def _on_gate(self, request, response):
        self.gate_calls.append(request.data)
        response.success = True
        return response

    def _on_nav(self, goal_handle):
        pose = goal_handle.request.pose.pose
        yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
        self.nav_goals.append(Pose2D(pose.position.x, pose.position.y, yaw))
        self.events.append('nav')
        if self.nav_moves_robot:
            with self._lock:
                self.robot = Pose2D(pose.position.x, pose.position.y, yaw)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _on_dock(self, goal_handle):
        pose = goal_handle.request.dock_pose.pose
        self.events.append('dock')
        self.dock_goals.append((goal_handle.request.use_dock_id, pose.position.x,
                                pose.position.y,
                                2 * math.atan2(pose.orientation.z, pose.orientation.w),
                                goal_handle.request.navigate_to_staging_pose))
        goal_handle.succeed()
        return DockRobot.Result(success=True)

    def _on_undock(self, goal_handle):
        self.undock_calls += 1
        goal_handle.succeed()
        return UndockRobot.Result(success=True)

    def put_obstacle_on_predock(self):
        """Another robot parked on the pre-dock pose."""
        predock = predock_pose_for(_stored_pose())
        with self._lock:
            self.predock_points = [(predock.x + dx, predock.y + dy)
                                   for dx in (-0.1, 0.0, 0.1) for dy in (-0.1, 0.0, 0.1)]

    def _bay_point(self, out, lateral):
        stored = _stored_pose()
        cos_t, sin_t = math.cos(stored.theta), math.sin(stored.theta)
        along = DOCKING_CHASSIS_OFFSET_M - out                                # + = towards the wall
        return (stored.x + along * cos_t - lateral * sin_t,
                stored.y + along * sin_t + lateral * cos_t)

    def put_robot_in_bay(self):
        """A robot docked in the bay: its rear face, 0.26 m out from the wall."""
        with self._lock:
            self.bay_points = [self._bay_point(0.26, lateral) for lateral in
                               (-0.11, -0.07, -0.035, 0.0, 0.035, 0.07, 0.11)]

    def put_low_object_in_bay(self):
        """A small object (a few cells), as the camera would mark one."""
        with self._lock:
            self.bay_points = [self._bay_point(0.35, lateral)
                               for lateral in (-0.02, 0.0, 0.03)]

    def put_one_stray_cell_in_bay(self):
        with self._lock:
            self.bay_points = [self._bay_point(0.35, 0.0)]

    def clear_bay(self):
        with self._lock:
            self.bay_points = []


@pytest.fixture(scope='module')
def ros():
    os.environ['ROS_DOMAIN_ID'] = '93'
    if rclpy.ok():
        rclpy.shutdown()
    rclpy.init(args=[
        '--ros-args', '--params-file', DOCKING_YAML,
        '-p', f'tag_dock_server:docking_config:={DOCKING_YAML}',
        '-p', f'tag_dock_server:dock_occupied_wait_sec:={WAIT_SEC}',
        '-p', 'tag_dock_server:bay_window_sec:=1.0',
        '-p', 'tag_dock_server:perception_timeout_sec:=3.0',
    ])
    yield
    rclpy.shutdown()


@pytest.fixture
def sequence(ros, tmp_path, monkeypatch):
    """A running server + rig, with the YAML saves redirected to copies."""
    docks, docking = tmp_path / 'docks.yaml', tmp_path / 'docking.yaml'
    shutil.copy(DOCKS_YAML, docks)
    shutil.copy(DOCKING_YAML, docking)
    sources = {'mission/config/docks.yaml': str(docks),
               'perception/config/docking.yaml': str(docking)}
    monkeypatch.setattr(server_module, 'find_workspace_source_file',
                        lambda share, relative: sources[relative])
    real_before = (_sha(DOCKS_YAML), _sha(DOCKING_YAML))

    server = server_module.TagDockServer()
    rig = Rig()
    client_node = Node('dock_client')
    client = ActionClient(client_node, DockRobot, 'dock_robot')
    undock_client = ActionClient(client_node, UndockRobot, 'undock_robot')
    executor = MultiThreadedExecutor(num_threads=12)
    for node in (server, rig, client_node):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert client.wait_for_server(timeout_sec=10.0)
    assert undock_client.wait_for_server(timeout_sec=10.0)

    class Sequence:
        pass
    run = Sequence()
    run.rig, run.client, run.docks, run.docking = rig, client, docks, docking
    run.undock_client = undock_client
    yield run

    executor.shutdown()
    thread.join(timeout=5.0)
    for node in (client_node, rig, server):
        node.destroy_node()
    assert (_sha(DOCKS_YAML), _sha(DOCKING_YAML)) == real_before, \
        'the real docks.yaml / docking.yaml were modified by the test'


def send_goal(run):
    goal = DockRobot.Goal(use_dock_id=True, dock_id=LETTER, navigate_to_staging_pose=True)
    future = run.client.send_goal_async(goal)
    deadline = time.monotonic() + 10.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    handle = future.result()
    assert handle is not None and handle.accepted
    return handle, handle.get_result_async()


def wait_for(predicate, timeout, what):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(f'timed out waiting for {what}')


def result_of(result_future, timeout=40.0):
    wait_for(result_future.done, timeout, 'the dock goal to finish')
    return result_future.result()


def assert_is_staging_pose(goal, dock_goal):
    """`goal` (a Nav2 goal) is the staging pose of the dock docking_server was
    given: the pre-dock position, facing away from the dock."""
    _, x, y, theta, _ = dock_goal
    predock = predock_pose_for(Pose2D(x, y, theta))
    assert (goal.x, goal.y) == pytest.approx((predock.x, predock.y), abs=1e-3)
    assert math.cos(goal.theta - (predock.theta + math.pi)) == pytest.approx(
        1.0, abs=1e-3), 'facing away from the dock'


def line_of(path, letter):
    for line in open(path):
        if line.strip().startswith(f'{letter}:'):
            return line.strip()
    raise AssertionError(f'no {letter} line in {path}')


# --------------------------------------------------------------------------

def test_free_bay_in_the_box_measures_saves_and_docks_without_navigating(sequence):
    run = sequence
    before = (line_of(run.docks, LETTER), line_of(run.docking, LETTER))
    _, result_future = send_goal(run)
    response = result_of(result_future)

    assert response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
    # Already in the box: Nav2 is used once, to turn the robot onto the staging
    # pose (pre-dock position, facing away), before docking_server is called.
    assert run.rig.events == ['nav', 'dock']
    assert_is_staging_pose(run.rig.nav_goals[0], run.rig.dock_goals[0])
    assert run.rig.gate_calls and run.rig.gate_calls[0] is True \
        and run.rig.gate_calls[-1] is False, 'detector on for the measurement, then off'

    # docking_server is called once, with the MEASURED pose and no staging of its own.
    assert len(run.rig.dock_goals) == 1
    use_dock_id, x, y, _theta, staging = run.rig.dock_goals[0]
    assert use_dock_id is False and staging is False
    stored = _stored_pose()
    assert x == pytest.approx(stored.x, abs=0.02)
    assert y == pytest.approx(stored.y + 0.02, abs=0.02), 'the 2 cm tag shift is used'

    # ...and the measurement was written to both files.
    after = (line_of(run.docks, LETTER), line_of(run.docking, LETTER))
    assert after[0] != before[0] and after[1] != before[1]
    measured_y = float(after[1].split('pose: [')[1].split(',')[1])
    assert measured_y == pytest.approx(stored.y + 0.02, abs=0.02)


def test_an_occupied_bay_is_waited_out_before_anything_else_happens(sequence):
    run = sequence
    run.rig.put_robot_in_bay()
    _, result_future = send_goal(run)

    time.sleep(WAIT_SEC * 2 + 3.5)       # several checks, all occupied
    assert not result_future.done(), 'must keep waiting while the dock is occupied'
    assert run.rig.dock_goals == [], 'must not dock into an occupied bay'
    assert run.rig.nav_goals == [], 'must not turn round while the bay is occupied'
    assert run.rig.gate_calls == [], 'must not measure the tag until the bay is free'

    run.rig.clear_bay()
    response = result_of(result_future)
    assert response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
    assert run.rig.events == ['nav', 'dock']
    assert run.rig.gate_calls and run.rig.gate_calls[0] is True


def test_a_small_low_object_in_the_bay_stops_the_dock(sequence):
    """What the camera adds: an object the lidar plane would pass over."""
    run = sequence
    run.rig.put_low_object_in_bay()
    _, result_future = send_goal(run)
    time.sleep(WAIT_SEC + 4.0)
    assert not result_future.done() and run.rig.dock_goals == []

    run.rig.clear_bay()
    assert result_of(result_future).result.success


def test_one_stray_cell_in_the_bay_is_not_an_obstacle(sequence):
    run = sequence
    run.rig.put_one_stray_cell_in_bay()
    response = result_of(send_goal(run)[1])
    assert response.result.success and len(run.rig.dock_goals) == 1


def test_cancelling_ends_the_wait(sequence):
    run = sequence
    run.rig.put_robot_in_bay()
    handle, result_future = send_goal(run)
    time.sleep(2.5)
    assert not result_future.done()

    handle.cancel_goal_async()
    response = result_of(result_future, timeout=15.0)
    assert response.status == GoalStatus.STATUS_CANCELED
    assert run.rig.dock_goals == [] and run.rig.nav_goals == []


def test_no_costmap_is_not_treated_as_a_free_bay(sequence):
    run = sequence
    run.rig.costmap_enabled = False
    _, result_future = send_goal(run)
    time.sleep(WAIT_SEC + 4.5)
    assert not result_future.done() and run.rig.dock_goals == []

    run.rig.costmap_enabled = True
    response = result_of(result_future)
    assert response.result.success and len(run.rig.dock_goals) == 1


def test_outside_the_box_nav2_takes_the_robot_to_the_predock_pose(sequence):
    run = sequence
    run.rig.robot = Pose2D(-0.3, 0.9, 1.0)               # nowhere near the box
    _, result_future = send_goal(run)
    response = result_of(result_future)

    assert response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
    # First to the pre-dock pose, facing the dock; then turned onto the staging
    # pose; then docking_server.
    assert run.rig.events == ['nav', 'nav', 'dock']
    goal = run.rig.nav_goals[0]
    expected = predock_pose_for(_stored_pose())
    assert (goal.x, goal.y) == pytest.approx((expected.x, expected.y), abs=1e-3)
    assert math.cos(goal.theta - expected.theta) == pytest.approx(1.0, abs=1e-3)
    assert_is_staging_pose(run.rig.nav_goals[1], run.rig.dock_goals[0])


def test_a_tag_that_is_never_seen_docks_against_the_stored_pose(sequence):
    run = sequence
    run.rig.tag_visible = False
    before = (line_of(run.docks, LETTER), line_of(run.docking, LETTER))
    _, result_future = send_goal(run)
    response = result_of(result_future)

    assert response.result.success and len(run.rig.dock_goals) == 1
    _, x, y, _theta, _staging = run.rig.dock_goals[0]
    stored = _stored_pose()
    assert (x, y) == pytest.approx((stored.x, stored.y), abs=1e-3)
    assert (line_of(run.docks, LETTER), line_of(run.docking, LETTER)) == before


def _park_on_predock_and_start_outside_the_box(run):
    stored = _stored_pose()
    run.rig.robot = predock_pose_for(stored, PREDOCK_STANDOFF_M + 0.7)   # 0.7 m behind the pre-dock pose
    run.rig.put_obstacle_on_predock()


def test_a_taken_predock_pose_sends_nav2_straight_to_the_dock_pose(sequence):
    run = sequence
    _park_on_predock_and_start_outside_the_box(run)
    before = (line_of(run.docks, LETTER), line_of(run.docking, LETTER))
    response = result_of(send_goal(run)[1])

    assert response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
    # One Nav2 goal, to the dock pose itself, facing the dock; docking_server
    # is not called and the tag is not measured.
    assert run.rig.events == ['nav']
    stored = _stored_pose()
    goal = run.rig.nav_goals[0]
    assert (goal.x, goal.y) == pytest.approx((stored.x, stored.y), abs=1e-3)
    assert math.cos(goal.theta - stored.theta) == pytest.approx(1.0, abs=1e-3)
    assert run.rig.dock_goals == []
    assert run.rig.gate_calls == []
    assert (line_of(run.docks, LETTER), line_of(run.docking, LETTER)) == before


def test_a_taken_predock_pose_and_an_occupied_bay_waits(sequence):
    run = sequence
    _park_on_predock_and_start_outside_the_box(run)
    run.rig.put_robot_in_bay()
    _, result_future = send_goal(run)

    time.sleep(WAIT_SEC + 3.5)
    assert not result_future.done()
    assert run.rig.events == [], 'must not drive towards an occupied dock'

    run.rig.clear_bay()
    response = result_of(result_future)
    assert response.result.success
    assert run.rig.events == ['nav']


def test_a_free_predock_pose_still_takes_the_normal_route(sequence):
    run = sequence
    run.rig.robot = predock_pose_for(_stored_pose(), PREDOCK_STANDOFF_M + 0.7)
    response = result_of(send_goal(run)[1])
    assert response.result.success
    assert run.rig.events == ['nav', 'nav', 'dock']



def undock(run):
    future = run.undock_client.send_goal_async(UndockRobot.Goal(dock_type='arena_dock'))
    wait_for(future.done, 10.0, 'the undock goal to be accepted')
    return result_of(future.result().get_result_async())


def test_undocking_after_the_nav2_fallback_just_reports_undocked(sequence):
    run = sequence
    _park_on_predock_and_start_outside_the_box(run)
    assert result_of(send_goal(run)[1]).result.success
    run.rig.events.clear()
    reached = run.rig.robot

    response = undock(run)
    assert response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
    assert run.rig.undock_calls == 0, 'docking_server is not asked to undock'
    assert run.rig.events == [] and run.rig.robot == reached, 'the robot does not move'

    # The flag is spent: a second undock goes to docking_server as usual.
    assert undock(run).result.success
    assert run.rig.undock_calls == 1


def test_undocking_after_a_normal_dock_is_left_to_docking_server(sequence):
    run = sequence
    assert result_of(send_goal(run)[1]).result.success
    assert undock(run).result.success
    assert run.rig.undock_calls == 1


def test_a_new_docking_clears_the_nav2_flag(sequence):
    run = sequence
    _park_on_predock_and_start_outside_the_box(run)
    assert result_of(send_goal(run)[1]).result.success      # fallback
    with run.rig._lock:
        run.rig.predock_points = []
        run.rig.robot = predock_pose_for(_stored_pose())
    assert result_of(send_goal(run)[1]).result.success      # normal
    assert undock(run).result.success
    assert run.rig.undock_calls == 1


def test_undocking_with_another_robot_on_the_predock_pose_just_reports_undocked(sequence):
    run = sequence
    assert result_of(send_goal(run)[1]).result.success      # normal dock
    run.rig.put_obstacle_on_predock()
    run.rig.events.clear()
    parked = run.rig.robot

    response = undock(run)
    assert response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success
    assert run.rig.undock_calls == 0, 'must not back out into the other robot'
    assert run.rig.events == [] and run.rig.robot == parked, 'the robot does not move'

    # The dock is spent: with nothing remembered, a second undock goes to docking_server.
    assert undock(run).result.success
    assert run.rig.undock_calls == 1


def test_undocking_without_a_costmap_does_not_back_out_blind(sequence):
    run = sequence
    assert result_of(send_goal(run)[1]).result.success
    run.rig.costmap_enabled = False
    time.sleep(0.6)                 # lets a costmap already in flight arrive and age out

    assert undock(run).result.success
    assert run.rig.undock_calls == 0

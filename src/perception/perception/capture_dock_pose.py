"""Interactive capture of real-world dock poses from AprilTag detections.

`ros2 run perception capture_dock_pose` -- terminal 3 of the mapping-mission
workflow described at length in
`localization/launch/mapping_mission.launch.py`'s docstring (read that first;
it is the primary documentation for this feature).

WHAT THIS IS FOR. `mission/config/docks.yaml`'s 8 dock poses were computed from
`arena.sdf`'s model origins -- i.e. from the SIMULATED world file. On real
hardware there is no arena.sdf: the docks are wherever somebody put them. This
node closes that gap without anybody doing trigonometry on a tape measure.
Drive the robot (keyboard teleop, terminal 2) until a dock's tag is in the
camera's view, type `i` + Enter here, and this node reads that tag's pose out
of TF in the `map` frame, converts it into docks.yaml's dock_pose/staging_pose
convention and docking.yaml's own (different) schema, and rewrites just those
two lines in the workspace SOURCE files.

WHAT IT DELIBERATELY DOES NOT DO.

- It does not average, filter or re-sample. One keypress is one snapshot of
  TF. Localization noise, a glancing viewing angle or a tag detected at the
  edge of the frame all go straight into the file. Capture from roughly where
  the robot would stage (~0.6 m out, square on to the tag) and re-capture if a
  value looks wrong -- re-capturing is idempotent, it rewrites the same line.
- It does not touch the installed `share/` copies, so a capture only takes
  effect on the next `colcon build`. That is deliberate: the source files are
  what git tracks and what the next build installs, and a capture written into
  `install/` would be silently discarded by that build.
- It does not validate that the captured pose is physically plausible (inside
  the arena, on a wall, 1.5 m from its neighbour). `git diff` is that check,
  and the dock-A sanity check in this feature's report is how you earn the
  right to trust the other 7.

ALL the arithmetic and all the file editing live in `perception.dock_capture`,
which imports no ROS at all, so both are unit-tested without a ROS graph. In
particular the tag-orientation -> `dock_pose.theta` convention (the part most
easily got backwards) is derived and documented in that module's header.
"""

import math
import sys
import threading

import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from perception.dock_capture import (AGE_CLOCK_MISMATCH, AGE_STALE,
                                     CLOCK_MISMATCH_AGE_SEC, ERROR_ORIENTATION,
                                     ERROR_TAG_ID, ERROR_TRANSFORM, MAP_FRAME,
                                     TagTransformUnavailable, apply_capture,
                                     capture_visible_tags,
                                     classify_detection_age,
                                     find_workspace_source_file,
                                     frame_names_from_tf_yaml,
                                     run_command_loop)

#: Reject a detection older (or further in the future) than this. Half a
#: second at the sim's camera rate is a handful of frames: long enough that a
#: capture never fails just because the subscription happened to land between
#: two images, short enough that a tag which left the view, or a robot that has
#: since been driven on, cannot be captured as if it were still there.
DEFAULT_MAX_DETECTION_AGE_SEC = 0.5

#: Source-relative paths of the two files a capture rewrites, used to find the
#: workspace copies from the installed share directories.
DOCKS_YAML_RELATIVE = 'mission/config/docks.yaml'
DOCKING_YAML_RELATIVE = 'perception/config/docking.yaml'


class CaptureDockPoseNode(Node):
    """Keeps the newest /detections message and turns a keypress into a capture.

    The only state is the latest detection array. Captures are driven from the
    stdin thread (see `main`), so that one attribute is the whole thread
    boundary and it is guarded by a lock.
    """

    def __init__(self):
        super().__init__('capture_dock_pose')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('map_frame', MAP_FRAME)
        self.declare_parameter('max_detection_age_sec',
                               DEFAULT_MAX_DETECTION_AGE_SEC)
        # Empty => auto-locate the workspace source copies. Override when the
        # files live somewhere this node cannot infer (a deployment installed
        # from a release, a second workspace).
        self.declare_parameter('docks_file', '')
        self.declare_parameter('docking_file', '')

        self._map_frame = self.get_parameter('map_frame').value
        self._max_age_sec = float(
            self.get_parameter('max_detection_age_sec').value)
        self._docks_path = self._resolve_config('docks_file', DOCKS_YAML_RELATIVE)
        self._docking_path = self._resolve_config(
            'docking_file', DOCKING_YAML_RELATIVE)

        self._lock = threading.Lock()
        self._latest = None
        # Per-capture scratch: tag height and TF age per frame, for the success
        # report only. Not part of the capture itself, which is planar.
        self._detail = {}

        self._tf_buffer = tf2_ros.Buffer()
        # spin_thread=False: this node is spun by main's own executor thread,
        # which is also what delivers TF messages into the buffer -- matching
        # mission_tree.py's shared Buffer/TransformListener idiom.
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=False)

        topic = self.get_parameter('detections_topic').value
        self.create_subscription(
            AprilTagDetectionArray, topic, self._on_detections, 10)
        self._topic = topic

    def _resolve_config(self, parameter_name, relative_path):
        """Absolute path of a config file to rewrite, or raise.

        Failing loudly at startup is the point: the alternative is discovering
        after eight captures that they all went into a share/ copy that the
        next build overwrote.

        Both files are located by walking up to the workspace source tree from
        THIS package's share directory, including `mission/config/docks.yaml` --
        see package.xml for why this must not go through mission's own share
        directory (it would close a dependency cycle).
        """
        override = self.get_parameter(parameter_name).value
        if override:
            return override
        found = find_workspace_source_file(
            get_package_share_directory('perception'), relative_path)
        if found is None:
            raise RuntimeError(
                f'could not find the workspace source copy of {relative_path} '
                f"from perception's installed share directory. Pass it "
                f'explicitly: --ros-args -p {parameter_name}:=/abs/path/to/'
                f'{relative_path.rsplit("/", 1)[-1]}')
        return found

    def _on_detections(self, msg):
        with self._lock:
            self._latest = msg

    def describe_setup(self):
        """Lines summarising what this node will read and write, for startup."""
        return [
            f'  detections topic : {self._topic}',
            f'  map frame        : {self._map_frame}',
            f'  max detection age: {self._max_age_sec:.2f} s',
            f'  docks.yaml       : {self._docks_path}',
            f'  docking.yaml     : {self._docking_path}',
            f'  use_sim_time     : {self.get_parameter("use_sim_time").value}',
        ]

    def capture(self):
        """Capture every tag in the newest /detections message. Never raises.

        Called from the stdin thread. This method owns only the "is there a
        current detection at all" decision; the per-tag work is
        `dock_capture.capture_visible_tags`, which keeps each tag independent
        (an unknown id or a failed TF lookup reports itself and the remaining
        tags in the same message are still captured -- two neighbouring docks
        in one frame is the normal case) and is unit-tested without a node.
        """
        logger = self.get_logger()
        with self._lock:
            msg = self._latest

        if msg is None:
            logger.warning(
                f'no tag currently visible: nothing has been received on '
                f'{self._topic} yet. Is apriltag.launch.py running?')
            return

        age_sec = (self.get_clock().now()
                   - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        verdict = classify_detection_age(age_sec, self._max_age_sec)
        if verdict == AGE_CLOCK_MISMATCH:
            logger.error(
                f'the newest detection is stamped {age_sec:+.1f} s from this '
                f'node\'s clock (>= {CLOCK_MISMATCH_AGE_SEC:.0f} s). That is '
                f'not staleness, it is two clock domains: this node and '
                f'apriltag_ros disagree about use_sim_time. Restart this node '
                f'with "--ros-args -p use_sim_time:=true" when running '
                f'against the simulator.')
            return
        if verdict == AGE_STALE:
            logger.warning(
                f'no tag currently visible: the newest detection is '
                f'{age_sec:+.2f} s old (limit {self._max_age_sec:.2f} s). '
                f'Point the camera at a dock tag and try again.')
            return
        if not msg.detections:
            logger.warning(
                'no tag currently visible: the camera has a current frame but '
                'apriltag_ros found no tag in it.')
            return

        self._detail = {}
        outcomes = capture_visible_tags(
            [(detection.id, detection.family) for detection in msg.detections],
            self._lookup_tag_transform,
            self._write_capture,
        )
        for outcome, detection in zip(outcomes, msg.detections):
            self._report(outcome, detection)

    def _lookup_tag_transform(self, frame):
        """`map` -> tag translation and rotation, for capture_visible_tags.

        Wraps tf2's exception in TagTransformUnavailable so the pure capture
        logic never has to know what tf2 is.
        """
        try:
            # Time() (zero) is tf2's "latest available transform", not a
            # literal timestamp -- the same idiom mission_tree.py uses, and
            # deliberately NOT a lookup at the detection's own stamp:
            # slam_toolbox here publishes map->odom stamped ~0.9 s INTO THE
            # FUTURE (see mission_tree._transform_is_recent's note), so asking
            # for the chain at an older image stamp is an extrapolation
            # request, while "latest available" resolves cleanly. The staleness
            # check on the detection is what guarantees "latest available"
            # means "the tag this keypress is about".
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, frame, Time())
        except tf2_ros.TransformException as error:
            raise TagTransformUnavailable(
                f'{self._map_frame}<-{frame}: {error}') from error

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        # Stashed for the success report only; the capture itself is planar.
        self._detail[frame] = (
            translation.z,
            (self.get_clock().now()
             - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9,
        )
        return ((translation.x, translation.y),
                (rotation.x, rotation.y, rotation.z, rotation.w))

    def _write_capture(self, capture):
        apply_capture(capture, self._docks_path, self._docking_path)

    def _report(self, outcome, detection):
        logger = self.get_logger()
        if outcome.succeeded:
            capture = outcome.capture
            tag_height, tf_age_sec = self._detail.get(
                outcome.frame, (float('nan'), float('nan')))
            logger.info(
                f'Captured tag {outcome.tag_id} -> dock {capture.letter}: '
                f'dock_pose=({_pose(capture.dock_pose)}), '
                f'staging_pose=({_pose(capture.staging_pose)}). '
                f'Updated docks.yaml and docking.yaml.')
            logger.info(
                f'  dock {capture.letter} detail: docking.yaml pose='
                f'({_pose(capture.docking_pose)}), tag height '
                f'{tag_height:+.3f} m, outward normal '
                f'{math.degrees(capture.dock_pose.theta):+.1f} deg, '
                f'TF age {tf_age_sec:+.2f} s, detection hamming '
                f'{detection.hamming}, decision margin '
                f'{detection.decision_margin:.1f}.')
            return

        if outcome.error_kind == ERROR_TRANSFORM:
            logger.error(
                f'tag {outcome.tag_id}: TF lookup failed, skipping this tag '
                f'(any other visible tag is still captured): {outcome.error}')
            known = frame_names_from_tf_yaml(
                self._tf_buffer.all_frames_as_yaml())
            logger.error(
                f'  if the frame NAME looks wrong rather than merely missing, '
                f'these are the frames tf2 currently knows: {known}')
        elif outcome.error_kind == ERROR_TAG_ID:
            logger.warning(
                f'tag {outcome.tag_id}: not an arena dock tag, nothing '
                f'written -- {outcome.error}')
        elif outcome.error_kind == ERROR_ORIENTATION:
            logger.warning(
                f'tag {outcome.tag_id}: skipped -- {outcome.error}')
        else:
            logger.error(
                f'tag {outcome.tag_id}: computed the pose but could not write '
                f'it, NOTHING was changed: {outcome.error}')


def _pose(pose):
    return f'{pose.x:.3f}, {pose.y:.3f}, {pose.theta:.5f}'


def _print_instructions(node):
    print()
    print('=' * 72)
    print('capture_dock_pose -- interactive dock-pose capture')
    print('=' * 72)
    for line in node.describe_setup():
        print(line)
    print()
    print("Drive the robot (terminal 2: teleop_twist_keyboard) until a dock's")
    print('AprilTag is clearly in the camera view, roughly square on and about')
    print('0.6 m out, then:')
    print()
    print("    type 'i' and press Enter  -> capture every tag now visible")
    print('    Ctrl-C                    -> quit')
    print()
    print('Each capture rewrites one line of docks.yaml and one line of')
    print('docking.yaml, in the workspace source tree. Review with `git diff`')
    print('and `colcon build` when done. Capturing dock A (tag 0) first and')
    print('comparing it against its existing known-good values is the')
    print('recommended sanity check before trusting the other seven.')
    print('=' * 72)
    print(flush=True)


def main():
    rclpy.init()
    try:
        node = CaptureDockPoseNode()
    except RuntimeError as error:
        print(f'capture_dock_pose: {error}', file=sys.stderr)
        rclpy.shutdown()
        return 1

    # ROS spins on its own thread so the stdin loop can block on the main
    # thread: that way Ctrl-C lands as a KeyboardInterrupt in the thread that
    # is waiting for input, which is the documented way to quit.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    _print_instructions(node)
    try:
        run_command_loop(sys.stdin, node.capture)
    except KeyboardInterrupt:
        pass
    finally:
        print('\ncapture_dock_pose: exiting.', flush=True)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

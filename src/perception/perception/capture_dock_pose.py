"""Interactive capture of real dock poses from AprilTag detections.

`ros2 run perception capture_dock_pose` is terminal 3 of the mapping-mission
workflow described in the docstring of
`localization/launch/mapping_mission_hardware.launch.py`, which is the primary
documentation. No `use_sim_time` flag is passed: there is no /clock on the robot.

`mission/config/docks.yaml`'s 8 dock poses are wherever the docks were placed.
Drive the robot (teleop, terminal 2) until a dock's tag is in view and type `i`
and Enter here. The node reads that tag's pose from TF in `map`, converts it to
docks.yaml's dock_pose/staging_pose convention and docking.yaml's different
schema, and rewrites just those two lines in the workspace source files.

What it deliberately does not do:

- Average, filter or resample. One keypress is one TF snapshot, so localization
  noise, a glancing angle or a tag at the frame edge go straight into the file.
  Capture from roughly the staging pose, square on, and re-capture if a value
  looks wrong (re-capturing rewrites the same line).
- Touch the installed `share/` copies. Git tracks the source files and the next
  `colcon build` installs them, and a capture written to `install/` would be
  discarded by that build.
- Check that the pose is plausible (inside the arena, on a wall, 1.5 m from its
  neighbour). `git diff` is that check, and a good dock A capture earns trust
  in the other 7.

All arithmetic and file editing lives in `perception.dock_capture` (no ROS,
unit-tested). The tag-orientation to `dock_pose.theta` convention, the part most
easily got backwards, is derived in that module's docstring.
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
                                     CLOCK_MISMATCH_AGE_SEC,
                                     DEFAULT_MAX_DETECTION_AGE_SEC,
                                     ERROR_ORIENTATION,
                                     ERROR_TAG_ID, ERROR_TRANSFORM, MAP_FRAME,
                                     TagTransformUnavailable, apply_capture,
                                     capture_visible_tags,
                                     classify_detection_age,
                                     find_workspace_source_file,
                                     frame_names_from_tf_yaml,
                                     run_command_loop)


# Paths under `src/` of the two files a capture rewrites.
DOCKS_YAML_RELATIVE = 'mission/config/docks.yaml'
DOCKING_YAML_RELATIVE = 'perception/config/docking.yaml'


class CaptureDockPoseNode(Node):
    """Keeps the newest /detections message and turns a keypress into a capture.

    Captures run on the stdin thread (see `main`), so the latest detection array
    is the whole thread boundary and is guarded by a lock.

    Parameters:
        detections_topic: AprilTagDetectionArray topic.
        map_frame: Frame the poses are expressed in.
        max_detection_age_sec: Oldest detection accepted, seconds.
        docks_file, docking_file: Paths to rewrite; empty means locate the
            workspace source copies automatically.
    """

    def __init__(self):
        super().__init__('capture_dock_pose')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('map_frame', MAP_FRAME)
        self.declare_parameter('max_detection_age_sec',
                               DEFAULT_MAX_DETECTION_AGE_SEC)
        # Empty auto-locates the workspace sources; override for a release
        # deployment or a second workspace.
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
        # Per-capture scratch (tag height, TF age per frame) for the report only.
        self._detail = {}

        self._tf_buffer = tf2_ros.Buffer()
        # spin_thread=False: main's executor thread spins this node and
        # delivers TF into the buffer, as in mission_tree.py.
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=False)

        topic = self.get_parameter('detections_topic').value
        self.create_subscription(
            AprilTagDetectionArray, topic, self._on_detections, 10)
        self._topic = topic

    def _resolve_config(self, parameter_name, relative_path):
        """Returns the absolute path of a config file to rewrite.

        Fails at startup, not after eight captures went into a `share/` copy that
        the next build overwrote. Both files are found by walking up from this
        package's share directory, including docks.yaml; package.xml explains why
        mission's own share directory would close a dependency cycle.

        Args:
            parameter_name: Parameter that may override the path.
            relative_path: Path under `src/` to search for.

        Raises:
            RuntimeError: If the file cannot be found and no override is given.
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
        """Stores the newest detection array."""
        with self._lock:
            self._latest = msg

    def describe_setup(self):
        """Returns startup lines summarising what this node reads and writes."""
        return [
            f'  detections topic : {self._topic}',
            f'  map frame        : {self._map_frame}',
            f'  max detection age: {self._max_age_sec:.2f} s',
            f'  docks.yaml       : {self._docks_path}',
            f'  docking.yaml     : {self._docking_path}',
            f'  use_sim_time     : {self.get_parameter("use_sim_time").value}',
        ]

    def capture(self):
        """Captures every tag in the newest /detections message; never raises.

        Called from the stdin thread. It decides only whether a current
        detection exists; the per-tag work is
        `dock_capture.capture_visible_tags`, which keeps tags independent.
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
                f'with no such flag (mapping_mission_hardware.launch.py runs '
                f'apriltag_ros on wall-clock time).')
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
        """Looks up the `map` -> tag transform for `capture_visible_tags`.

        Args:
            frame: The tag's TF frame name.

        Returns:
            `((x, y), (qx, qy, qz, qw))`.

        Raises:
            TagTransformUnavailable: If the lookup fails, so the pure capture
                logic never sees tf2.
        """
        try:
            # Time() (zero) means "latest available" in tf2, not a timestamp, as
            # in mission_tree.py. Not the detection's stamp: slam_toolbox stamps
            # map->odom ~0.9 s into the future, so an older stamp would need
            # extrapolation. The detection staleness check ensures "latest" is
            # the tag this keypress is about.
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, frame, Time())
        except tf2_ros.TransformException as error:
            raise TagTransformUnavailable(
                f'{self._map_frame}<-{frame}: {error}') from error

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        # For the report only; the capture itself is planar.
        self._detail[frame] = (
            translation.z,
            (self.get_clock().now()
             - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9,
        )
        return ((translation.x, translation.y),
                (rotation.x, rotation.y, rotation.z, rotation.w))

    def _write_capture(self, capture):
        """Writes `capture` into both YAML files."""
        apply_capture(capture, self._docks_path, self._docking_path)

    def _report(self, outcome, detection):
        """Logs the result of capturing one tag."""
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
    """Formats a pose as "x, y, theta"."""
    return f'{pose.x:.3f}, {pose.y:.3f}, {pose.theta:.5f}'


def _print_instructions(node):
    """Prints the startup banner and usage."""
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
    """Runs the capture loop; returns the process exit code."""
    rclpy.init()
    try:
        node = CaptureDockPoseNode()
    except RuntimeError as error:
        print(f'capture_dock_pose: {error}', file=sys.stderr)
        rclpy.shutdown()
        return 1

    # ROS spins on its own thread so Ctrl-C reaches the thread waiting on stdin.
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

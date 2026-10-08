"""Pure geometry and YAML-editing logic behind `ros2 run perception capture_dock_pose`.

Imports no rclpy, tf2_ros or apriltag_msgs. Every number written into
`mission/config/docks.yaml` and `perception/config/docking.yaml` comes from a
plain function here, so the whole conversion (tag TF to dock_pose to
staging_pose to opennav_docking pose to the text of one YAML line) is
unit-testable without a ROS graph or camera (see
`perception/test/test_dock_capture.py`). `capture_dock_pose.py` is a thin ROS
shell: subscribe, look up TF, call in here, print.

Orientation convention. `docks.yaml`'s `dock_pose.theta` is the dock tag's
outward (room-facing) normal, pointing from the wall into the arena. Getting
that backwards is a bug class already hit in this repo (see the header of
`perception/config/docking.yaml` for the pi-rotation false start), so the
mapping from the tag's TF orientation to that angle was derived, not assumed:

1. `apriltag_ros` 3.4.0 (installed at /opt/ros/jazzy) estimates each tag pose
   with `pnp()`, which gives `cv::solvePnP` these object points (read out of
   the installed `libAprilTagNode.so`; only the binary ships):

       objectPoints = [(-s,-s,0), (+s,-s,0), (+s,+s,0), (-s,+s,0)],
       s = tagsize/2

   paired index-for-index with the detected corner pixels `det->p[0..3]`.

2. `apriltag` puts those corners (probed offline by rendering `tag36h11` id 0
   with `apriltag_to_image()`, upscaling and detecting), for an upright tag as
   a viewer sees it, at:

       p[0] = bottom-left, p[1] = bottom-right,
       p[2] = top-right,   p[3] = top-left

   This matches `apriltag.h`'s ideal-tag corner order ((-1,1), (1,1), (1,-1),
   (-1,-1); +y of the homography's ideal frame points down the image).

3. Combining 1 and 2, the tag frame has +x to the viewer's right and +y up, so
   (right-handed) +z points out of the tag's face toward the viewer.
   `apriltag_ros` broadcasts that pose unmodified as the TF `<camera optical
   frame> -> <tag frame>` (a plain rotation-matrix to quaternion conversion).

For a wall-mounted tag facing the room, the tag's local +z is the outward
normal, so

    dock_pose.theta = atan2(z_axis.y, z_axis.x)   (z_axis expressed in `map`)

which is `tag_outward_theta()`. It is cross-checked against `cv2.solvePnP` for
dock A's geometry in
`test_dock_capture.py::test_solvepnp_round_trip_reproduces_dock_a_theta`.

STILL UNVERIFIED AGAINST A LIVE TAG. All of this was derived offline from the
installed binaries and libraries and has not been confirmed against a real
`/detections` and TF stream. Capture dock A (tag id 0) first and compare it with
its known-good entry before trusting the rest.
"""

import math
import os
import re
from dataclasses import dataclass


# Distance from a dock's back wall to the pre-dock pose: the middle of the
# acceptable box (`predock_min_tag_distance_m` 0.5 to `predock_max_tag_distance_m`
# 1.0 in docking.yaml), so a Nav2 goal landing near it is inside the box.
PREDOCK_TAG_DISTANCE_M = 0.75

# Standoff for `docks.yaml`'s `staging_pose`: the pre-dock pose, measured from
# the dock's back wall (docks.yaml's dock_pose).
STAGING_STANDOFF_M = PREDOCK_TAG_DISTANCE_M

# Chassis standoff between `docks.yaml`'s `dock_pose` (the back wall) and
# docking.yaml's `pose` (where the robot ends up). Measured live at dock A (see
# the docking.yaml header); only re-applied here.
DOCKING_CHASSIS_OFFSET_M = 0.126

# The arena's fixed tag id to dock letter table: 8 docks, 8 tags, as in
# `docks.yaml`'s `tag_id` fields.
TAG_ID_TO_LETTER = {0: 'A', 1: 'B', 2: 'C', 3: 'D',
                    4: 'E', 5: 'F', 6: 'G', 7: 'H'}

# opennav_docking dock plugin name, from docking.yaml (`dock_plugins`).
DOCKING_PLUGIN_TYPE = 'arena_dock'

# Frame all captured poses are expressed in.
MAP_FRAME = 'map'

# Decimals written back into the two YAML files. Chosen so re-capturing a dock
# at its recorded pose reproduces the existing line byte for byte (see
# test_formatting_reproduces_existing_*): x/y use a fixed 3 decimals (-1.500),
# theta up to 5 with trailing zeros trimmed (3.14159, -1.5708, 0.0). That is
# finer than SLAM or a hand-driven capture can deliver.
POSITION_DECIMALS = 3
THETA_DECIMALS = 5

# Below this the tag's normal is too close to vertical for its XY projection to
# mean anything (1e-3 is within 0.06 deg of straight up or down).
MIN_HORIZONTAL_NORMAL = 1e-3

# `classify_detection_age` verdicts.
AGE_OK = 'ok'
AGE_STALE = 'stale'
AGE_CLOCK_MISMATCH = 'clock_mismatch'

# An age this large is two clock domains (use_sim_time:=true next to a
# wall-clock publisher, or vice versa), a mistake made here before, so it gets
# its own diagnosis instead of "stale tag".
CLOCK_MISMATCH_AGE_SEC = 1000.0

# Default `max_detection_age_sec` for capture_dock_pose: reject a detection
# older (or further in the future) than this. It must cover the end-to-end image
# pipeline latency. On the Jetson (OAK-D 1280x720, rectify_node, apriltag_node)
# detection stamps arrived 0.86-2.4 s old (median 1.4 s), so the original 0.5 s
# rejected every hardware detection. 3 s still stops a tag that has left the view
# being captured, and stays far below CLOCK_MISMATCH_AGE_SEC.
DEFAULT_MAX_DETECTION_AGE_SEC = 3.0


class DockCaptureError(Exception):
    """Base for the per-tag failures a capture reports and then skips past."""


class DegenerateTagOrientation(DockCaptureError):
    """The tag's outward normal is (near) vertical, so it has no map heading."""


class UnknownTagId(DockCaptureError):
    """The tag id is outside the arena's 0-7 dock tags."""


class DockEntryNotFound(DockCaptureError):
    """No (or more than one) line in the YAML file to replace for this dock."""


class TagTransformUnavailable(DockCaptureError):
    """The `map` -> tag TF could not be resolved for this tag.

    Raised by the caller's lookup callable (the node wraps
    `tf2_ros.TransformException`), so `capture_visible_tags` can treat a TF
    failure as a per-tag outcome without importing tf2.
    """


# `CaptureOutcome.error_kind` values: which stage of one tag's capture failed.
# The node words its log differently for each, since a frame-name problem and a
# bad tag id need different things from the operator.
ERROR_TRANSFORM = 'transform'
ERROR_TAG_ID = 'tag_id'
ERROR_ORIENTATION = 'orientation'
ERROR_WRITE = 'write'


@dataclass(frozen=True)
class Pose2D:
    """A planar pose in `map`.

    Attributes:
        x: Metres.
        y: Metres.
        theta: Heading in radians, normalized to (-pi, pi].
    """

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class DockCapture:
    """Everything one captured tag implies, ready to be written out.

    Attributes:
        letter: Dock letter.
        tag_id: AprilTag id.
        dock_pose: Pose of the dock's back wall (docks.yaml).
        staging_pose: Pre-dock pose (docks.yaml).
        docking_pose: Docked robot pose (docking.yaml).
    """

    letter: str
    tag_id: int
    dock_pose: Pose2D
    staging_pose: Pose2D
    docking_pose: Pose2D


def normalize_angle(theta: float) -> float:
    """Normalizes an angle to (-pi, pi], matching `docks.yaml`'s convention.

    `math.remainder` returns the closed range [-pi, pi], so -pi is folded to +pi.
    Otherwise `atan2(-0.0, -1.0)` returning -pi would write some +x-wall
    captures as -3.14159 and others as 3.14159. The `+ 0.0` removes negative
    zero, which formats as "-0.0" and would show up as a spurious diff.
    """
    wrapped = math.remainder(theta, math.tau)
    if wrapped <= -math.pi:
        wrapped = math.pi
    return wrapped + 0.0


def tag_outward_theta(rotation_xyzw) -> float:
    """Returns the map heading of the tag's outward (room-facing) normal.

    Uses the tag frame's local +z axis, which points out of the printed face
    (see the module docstring). For a tag flat on a dock's back wall that is
    away from the wall, into the room, which is what `dock_pose.theta` means.
    Only the third column of the rotation matrix (that axis in `map`) is built.

    Args:
        rotation_xyzw: `(x, y, z, w)` rotation of the `map` -> tag transform,
            i.e. `TransformStamped.transform.rotation`.

    Raises:
        DegenerateTagOrientation: If the quaternion is zero or the axis is
            near vertical (the tag faces the floor or ceiling). A real wall
            dock cannot, so the detection or TF chain is wrong, and projecting
            it would invent a heading from rounding noise.
    """
    x, y, z, w = (float(value) for value in rotation_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise DegenerateTagOrientation(
            'tag orientation quaternion is all zeros (not a rotation)')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    # Third column of the rotation matrix: the tag's local +z axis in `map`.
    axis_x = 2.0 * (x * z + w * y)
    axis_y = 2.0 * (y * z - w * x)

    horizontal = math.hypot(axis_x, axis_y)
    if horizontal < MIN_HORIZONTAL_NORMAL:
        axis_z = 1.0 - 2.0 * (x * x + y * y)
        raise DegenerateTagOrientation(
            "the tag's outward normal is vertical in `map` "
            f'(z component {axis_z:+.3f}), so it has no heading in the XY '
            'plane -- the tag is facing the floor or the ceiling, which no '
            'arena dock does. Check the camera TF chain and the detection.')
    return normalize_angle(math.atan2(axis_y, axis_x))


def staging_pose_for(dock_pose: Pose2D,
                     standoff_m: float = STAGING_STANDOFF_M) -> Pose2D:
    """Returns the staging pose: `standoff_m` out along the outward normal, facing the dock.

        staging.x     = dock.x + d * cos(dock.theta)
        staging.y     = dock.y + d * sin(dock.theta)
        staging.theta = dock.theta + pi

    Args:
        dock_pose: The dock's back-wall pose.
        standoff_m: Distance out from the wall in metres.
    """
    return Pose2D(
        x=dock_pose.x + standoff_m * math.cos(dock_pose.theta),
        y=dock_pose.y + standoff_m * math.sin(dock_pose.theta),
        theta=normalize_angle(dock_pose.theta + math.pi),
    )


def docking_server_pose_for(dock_pose: Pose2D,
                            offset_m: float = DOCKING_CHASSIS_OFFSET_M) -> Pose2D:
    """Returns docking.yaml's pose for the same dock.

    Two differences from `dock_pose` (see that file's header):

    - Position moves `offset_m` outward along the normal, because `dock_pose` is
      the back wall while opennav_docking's pose is where `base_frame` ends up,
      and the nose cannot get closer than ~0.123 m to the wall.
    - Theta rotates by pi, because opennav_docking's theta is the robot's
      nose-forward heading once docked (into the wall), not the outward normal.

    Args:
        dock_pose: The dock's back-wall pose.
        offset_m: Chassis standoff from the wall in metres.
    """
    return Pose2D(
        x=dock_pose.x + offset_m * math.cos(dock_pose.theta),
        y=dock_pose.y + offset_m * math.sin(dock_pose.theta),
        theta=normalize_angle(dock_pose.theta + math.pi),
    )


def letter_for_tag_id(tag_id: int) -> str:
    """Returns the dock letter for an arena tag id.

    Raises:
        UnknownTagId: If `tag_id` is outside 0-7.
    """
    try:
        return TAG_ID_TO_LETTER[int(tag_id)]
    except (KeyError, TypeError, ValueError):
        raise UnknownTagId(
            f'tag id {tag_id} is not one of the arena dock tags 0-7') from None


def capture_from_tag_transform(tag_id: int, translation_xy, rotation_xyzw,
                               standoff_m: float = STAGING_STANDOFF_M,
                               docking_offset_m: float = DOCKING_CHASSIS_OFFSET_M,
                               ) -> DockCapture:
    """Returns everything one visible tag implies.

    Args:
        tag_id: AprilTag id.
        translation_xy: `(x, y)` of the tag in `map`; height is irrelevant as
            all three poses are planar.
        rotation_xyzw: Quaternion of the `map` -> tag transform.
        standoff_m: Staging standoff in metres.
        docking_offset_m: Chassis standoff from the wall in metres.
    """
    letter = letter_for_tag_id(tag_id)
    theta = tag_outward_theta(rotation_xyzw)
    dock_pose = Pose2D(x=float(translation_xy[0]),
                       y=float(translation_xy[1]),
                       theta=theta)
    return DockCapture(
        letter=letter,
        tag_id=int(tag_id),
        dock_pose=dock_pose,
        staging_pose=staging_pose_for(dock_pose, standoff_m),
        docking_pose=docking_server_pose_for(dock_pose, docking_offset_m),
    )


def capture_from_docking_pose(tag_id: int, docking_pose: Pose2D,
                              standoff_m: float = STAGING_STANDOFF_M,
                              docking_offset_m: float = DOCKING_CHASSIS_OFFSET_M,
                              ) -> DockCapture:
    """Returns everything implied by a measured docking pose.

    The inverse of `docking_server_pose_for`. `docking_pose` is where
    base_footprint ends up (heading into the wall, `docking_offset_m` out from
    it), so the wall is that far further along the heading and its outward
    normal points the opposite way.

    Args:
        tag_id: AprilTag id.
        docking_pose: The docked robot pose.
        standoff_m: Staging standoff in metres.
        docking_offset_m: Chassis standoff from the wall in metres.
    """
    wall = Pose2D(
        x=docking_pose.x + docking_offset_m * math.cos(docking_pose.theta),
        y=docking_pose.y + docking_offset_m * math.sin(docking_pose.theta),
        theta=normalize_angle(docking_pose.theta - math.pi),
    )
    return DockCapture(
        letter=letter_for_tag_id(tag_id),
        tag_id=int(tag_id),
        dock_pose=wall,
        staging_pose=staging_pose_for(wall, standoff_m),
        docking_pose=docking_pose,
    )


@dataclass(frozen=True)
class CaptureOutcome:
    """What happened to one tag in one capture.

    Attributes:
        tag_id: AprilTag id.
        frame: TF frame name of the tag.
        capture: The `DockCapture`, or None on failure.
        error_kind: One of the `ERROR_*` constants, or None on success.
        error: Error message, or None on success.
    """

    tag_id: int
    frame: str
    capture: DockCapture = None
    error_kind: str = None
    error: str = None

    @property
    def succeeded(self) -> bool:
        """Returns True if the tag was captured."""
        return self.capture is not None


def capture_visible_tags(tags, lookup_transform, write_capture):
    """Captures every tag in one detection message, independently.

    Every expected per-tag failure (broken TF, id outside 0-7, no horizontal
    normal, a missing YAML entry) becomes an outcome instead of an exception,
    so it cannot abort the other tags in the message. Two neighbouring docks in
    one frame is normal, and losing the good one would mean re-driving. Other
    errors, such as a bug or a vanished disk, still propagate. The caller does
    the reporting; this does not log.

    Args:
        tags: Iterable of `(tag_id, family)` pairs from the detection array.
        lookup_transform: Callable taking a frame name and returning
            `((x, y), (qx, qy, qz, qw))` in `map`, or raising
            `TagTransformUnavailable`.
        write_capture: Callable persisting one `DockCapture` (normally
            `apply_capture` bound to the two file paths).

    Returns:
        One `CaptureOutcome` per tag, in order.
    """
    outcomes = []
    for tag_id, family in tags:
        frame = tag_tf_frame(family, tag_id)
        try:
            translation_xy, rotation_xyzw = lookup_transform(frame)
        except TagTransformUnavailable as error:
            outcomes.append(CaptureOutcome(
                tag_id, frame, error_kind=ERROR_TRANSFORM, error=str(error)))
            continue

        try:
            capture = capture_from_tag_transform(
                tag_id, translation_xy, rotation_xyzw)
        except UnknownTagId as error:
            outcomes.append(CaptureOutcome(
                tag_id, frame, error_kind=ERROR_TAG_ID, error=str(error)))
            continue
        except DegenerateTagOrientation as error:
            outcomes.append(CaptureOutcome(
                tag_id, frame, error_kind=ERROR_ORIENTATION, error=str(error)))
            continue

        try:
            write_capture(capture)
        except (DockCaptureError, OSError) as error:
            outcomes.append(CaptureOutcome(
                tag_id, frame, error_kind=ERROR_WRITE, error=str(error)))
            continue

        outcomes.append(CaptureOutcome(tag_id, frame, capture=capture))
    return outcomes


def classify_detection_age(age_sec: float, max_age_sec: float) -> str:
    """Classifies a detection message's age.

    Both directions are bounded: an old stamp means the tag left the view or the
    robot has moved (a stale capture would be silently wrong), and a wildly off
    stamp either way means the detection and this node use different clocks.

    Args:
        age_sec: Now minus the message stamp; negative for a future stamp.
        max_age_sec: Largest acceptable absolute age.

    Returns:
        `AGE_OK`, `AGE_STALE` or `AGE_CLOCK_MISMATCH`.
    """
    if abs(age_sec) >= CLOCK_MISMATCH_AGE_SEC:
        return AGE_CLOCK_MISMATCH
    if not -max_age_sec <= age_sec <= max_age_sec:
        return AGE_STALE
    return AGE_OK


def tag_tf_frame(family: str, tag_id: int) -> str:
    """Returns the TF frame name apriltag_ros broadcasts for one detected tag.

    `family` must be the detection message's own `family` field, not the
    `family` parameter in apriltag.yaml ("36h11"). apriltag_ros fills the field
    and the TF `child_frame_id` from the same `det->family->name`
    ("tag36h11" here), joined to the id with ":" (verified in the installed
    libAprilTagNode.so, where both load offset 0x38 of the family struct).
    This is apriltag_ros's default naming; apriltag.yaml sets no `tag.frames`.

    Args:
        family: The message's `family` field.
        tag_id: AprilTag id.
    """
    return f'{family}:{int(tag_id)}'


def _fixed(value: float, decimals: int) -> str:
    """Formats `value` with fixed decimals, turning "-0.000" into "0.000"."""
    text = f'{value:.{decimals}f}'
    if text.startswith('-') and float(text) == 0.0:
        text = text[1:]
    return text


def _theta(value: float) -> str:
    """Formats a theta with up to THETA_DECIMALS decimals, trimmed, at least one.

    Applied to pi, -pi/2 and 0 it gives 3.14159, -1.5708 and 0.0, which is what
    makes re-capturing an unchanged dock byte-identical.
    """
    text = _fixed(value, THETA_DECIMALS)
    trimmed = text.rstrip('0')
    if trimmed.endswith('.'):
        trimmed += '0'
    return trimmed


def _pose_fields(pose: Pose2D) -> str:
    """Formats a pose as the `x: .., y: .., theta: ..` fields of a flow mapping."""
    return (f'x: {_fixed(pose.x, POSITION_DECIMALS)}, '
            f'y: {_fixed(pose.y, POSITION_DECIMALS)}, '
            f'theta: {_theta(pose.theta)}')


def format_docks_entry(capture: DockCapture) -> str:
    """Returns one `mission/config/docks.yaml` dock line without its indent.

    Matches the file's flow-style one-line-per-dock format; the caller reuses the
    indentation of the line being replaced.
    """
    return (f'{capture.letter}: {{tag_id: {capture.tag_id}, '
            f'dock_pose: {{{_pose_fields(capture.dock_pose)}}}, '
            f'staging_pose: {{{_pose_fields(capture.staging_pose)}}}}}')


def format_docking_entry(capture: DockCapture) -> str:
    """Returns one `perception/config/docking.yaml` dock line without its indent.

    A different schema from docks.yaml: opennav_docking's dock database format
    (`{type, frame, pose: [x, y, theta], id}`, parsed by `parseDockParams`),
    with `id` the AprilTag id as a string and the pose a 3-element sequence.
    """
    pose = capture.docking_pose
    return (f'{capture.letter}: {{type: "{DOCKING_PLUGIN_TYPE}", '
            f'frame: "{MAP_FRAME}", '
            f'pose: [{_fixed(pose.x, POSITION_DECIMALS)}, '
            f'{_fixed(pose.y, POSITION_DECIMALS)}, '
            f'{_theta(pose.theta)}], '
            f'id: "{capture.tag_id}"}}')


def replace_entry_line(text: str, letter: str, new_body: str) -> str:
    """Replaces exactly the one line defining dock `letter`.

    Edits the line directly instead of a YAML load/dump round trip, which would
    delete the files' header comments and reflow the other entries. The line's
    indentation is kept, so one function serves docks.yaml (2 spaces) and
    docking.yaml (8 spaces).

    Args:
        text: The YAML file's contents.
        letter: Dock letter to replace.
        new_body: The new line without indentation.

    Returns:
        The updated text.

    Raises:
        DockEntryNotFound: If not exactly one line matches.
    """
    pattern = re.compile(
        rf'^(?P<indent>[ \t]*){re.escape(letter)}:[ \t]*\{{.*$', re.MULTILINE)
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise DockEntryNotFound(
            f'expected exactly 1 line starting with "{letter}: {{", '
            f'found {len(matches)}')
    match = matches[0]
    return (text[:match.start()]
            + match.group('indent') + new_body
            + text[match.end():])


def apply_capture(capture: DockCapture, docks_path: str,
                  docking_path: str) -> None:
    """Writes one capture into both YAML files, or into neither.

    Both replacements are computed before either file is written. The files
    describe one dock in two schemas and are only useful in agreement
    (docks.yaml is the source of truth), so a line missing or duplicated in one
    must not leave the other half-updated.

    Args:
        capture: The capture to write.
        docks_path: Path to docks.yaml.
        docking_path: Path to docking.yaml.

    Raises:
        DockEntryNotFound: If a dock line is missing or duplicated.
    """
    with open(docks_path) as handle:
        docks_text = handle.read()
    with open(docking_path) as handle:
        docking_text = handle.read()

    new_docks_text = replace_entry_line(
        docks_text, capture.letter, format_docks_entry(capture))
    new_docking_text = replace_entry_line(
        docking_text, capture.letter, format_docking_entry(capture))

    with open(docks_path, 'w') as handle:
        handle.write(new_docks_text)
    with open(docking_path, 'w') as handle:
        handle.write(new_docking_text)


# `interpret_command` verdicts.
COMMAND_CAPTURE = 'capture'
COMMAND_EMPTY = 'empty'
COMMAND_UNKNOWN = 'unknown'

# What the operator types to capture, and the prompt they see.
CAPTURE_KEY = 'i'
PROMPT = "[capture_dock_pose] ready -- 'i' + Enter to capture, Ctrl-C to quit"


def interpret_command(line: str) -> str:
    """Classifies one line typed by the operator.

    Trimmed and case-insensitive, so "I", " i " and "i\\n" all capture. A bare
    Enter is `COMMAND_EMPTY`, not unknown: people press it to check the prompt
    is alive, so the loop just re-prints the prompt.

    Returns:
        `COMMAND_CAPTURE`, `COMMAND_EMPTY` or `COMMAND_UNKNOWN`.
    """
    stripped = line.strip()
    if not stripped:
        return COMMAND_EMPTY
    if stripped.lower() == CAPTURE_KEY:
        return COMMAND_CAPTURE
    return COMMAND_UNKNOWN


def run_command_loop(stream, on_capture, emit=print, prompt: str = PROMPT) -> None:
    """Reads lines off `stream` and dispatches captures until EOF or Ctrl-C.

    Uses plain blocking line reads, not raw single-keypress mode, so it works in
    any terminal or over ssh and cannot leave the tty half-configured; the cost
    is one extra Enter. It is ROS-free so it is testable with a StringIO. Ctrl-C
    propagates as KeyboardInterrupt for `main`. The prompt is a whole line,
    re-printed each iteration, because ROS log output shares the terminal and
    would leave an inline "> " dangling.

    Args:
        stream: Object with `readline()`.
        on_capture: Callable run for a capture command.
        emit: One-argument writer for prompts and messages.
        prompt: The prompt line.
    """
    while True:
        emit(prompt)
        line = stream.readline()
        if line == '':
            return
        command = interpret_command(line)
        if command == COMMAND_CAPTURE:
            on_capture()
        elif command == COMMAND_UNKNOWN:
            emit(f'unrecognized input {line.strip()!r} -- '
                 f"type '{CAPTURE_KEY}' and press Enter to capture, "
                 f'or Ctrl-C to quit.')


def frame_names_from_tf_yaml(tf_yaml: str):
    """Returns the sorted frame names in `tf2_ros.Buffer.all_frames_as_yaml()` output.

    Makes a failed `map` -> tag lookup diagnosable: if apriltag_ros names its
    frames differently from `tag_tf_frame`, the frames tf2 knows show it. Parsed
    by hand because the buffer returns "{}" when empty and only the top-level
    keys are wanted.
    """
    names = []
    for line in tf_yaml.splitlines():
        if not line or line[0].isspace():
            continue
        # rsplit: tag frame names contain a colon ("tag36h11:0").
        name = line.rsplit(':', 1)[0].strip()
        if name and name != '{}':
            names.append(name)
    return sorted(names)


def find_workspace_source_file(share_dir: str, relative_path: str,
                               max_levels: int = 8):
    """Locates the source copy of a config file from a package's share dir.

    Captures must edit `<workspace>/src/<relative_path>`, not the installed
    `share/` copy, which the next `colcon build` overwrites. Walks up from the
    share directory looking for a `src/<relative_path>` sibling, which works for
    merged, isolated and `--symlink-install` layouts.

    Args:
        share_dir: The package's installed share directory.
        relative_path: Path under `src/` to find.
        max_levels: Directory levels to climb.

    Returns:
        The absolute path, or None if no workspace source tree is found.
    """
    current = os.path.abspath(share_dir)
    for _ in range(max_levels):
        current = os.path.dirname(current)
        if not current or current == os.path.dirname(current):
            break
        candidate = os.path.join(current, 'src', relative_path)
        if os.path.isfile(candidate):
            return candidate
    return None

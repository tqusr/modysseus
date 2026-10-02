"""Pure geometry + YAML-editing logic behind `ros2 run perception capture_dock_pose`.

NOTHING HERE IMPORTS rclpy, tf2_ros OR apriltag_msgs, on purpose. Every number
this feature writes into `mission/config/docks.yaml` and
`perception/config/docking.yaml` is computed by a plain function in this
module, so the whole conversion -- tag TF -> dock_pose -> staging_pose ->
opennav_docking pose -> the exact text of one YAML line -- is unit-testable
without a ROS graph, a TF broadcaster, a camera or Gazebo (see
`perception/test/test_dock_capture.py`). `capture_dock_pose.py` is then a thin
ROS shell: subscribe, look up TF, call in here, print.

THE ORIENTATION CONVENTION, AND WHERE IT COMES FROM.

`docks.yaml`'s `dock_pose.theta` is the dock/tag's own OUTWARD (room-facing)
normal: the direction pointing away from the wall the dock is mounted on, out
into the arena. Getting that backwards is a real, already-hit bug class in
this repo (see `perception/config/docking.yaml`'s header comment for the
pi-rotation false start it cost), so the mapping from an AprilTag's TF
orientation to that angle was DERIVED, not assumed:

1. `apriltag_ros` 3.4.0 (the version installed at /opt/ros/jazzy) estimates
   each tag's pose with `pnp()`, which hands `cv::solvePnP` these object
   points, in this order (read out of the installed `libAprilTagNode.so`,
   since only the binary ships -- no sources in the share directory):

       objectPoints = [(-s,-s,0), (+s,-s,0), (+s,+s,0), (-s,+s,0)],
       s = tagsize/2

   paired index-for-index with `det->p[0..3]`, the detected corner pixels.

2. `apriltag` itself (probed offline: render `tag36h11` id 0 with the
   library's own `apriltag_to_image()`, upscale, detect) puts those corners
   at, for an upright tag as a human looking at it sees it:

       p[0] = bottom-left, p[1] = bottom-right,
       p[2] = top-right,   p[3] = top-left

   which also matches `apriltag.h`'s documented ideal-tag corner order
   ((-1,1), (1,1), (1,-1), (-1,-1), i.e. +y of the homography's ideal frame
   points DOWN the image).

3. Composing 1 and 2: the tag's own frame has +x to the viewer's RIGHT and +y
   UP as seen facing the tag, hence (right-handed) +z OUT OF THE TAG'S FACE,
   toward whoever is looking at it. `apriltag_ros` broadcasts that pose
   unmodified as the TF `<camera optical frame> -> <tag frame>` (its
   `tf2::toMsg<apriltag_pose_t, Transform>` is a plain rotation-matrix ->
   quaternion conversion, no axis remapping).

So for a wall-mounted dock tag facing into the room, the tag frame's local +z
axis IS the outward/room-facing normal, and

    dock_pose.theta = atan2(z_axis.y, z_axis.x)   (z_axis expressed in `map`)

which is what `tag_outward_theta()` computes. Cross-checked end-to-end against
`cv2.solvePnP` for dock A's real geometry -- see
`test_dock_capture.py::test_solvepnp_round_trip_reproduces_dock_a_theta`.

!!! STILL UNVERIFIED AGAINST A LIVE TAG !!! Everything above is derived from
the installed binaries and libraries, offline. It has NOT been confirmed
against a real `/detections` + TF stream. Capture dock A (tag id 0) first and
compare against its known-good sim-derived entry before trusting the rest --
see this feature's report for the step-by-step procedure.
"""

import math
import os
import re
from dataclasses import dataclass


#: Standoff used for `docks.yaml`'s `staging_pose`, per that file's header
#: comment (within the task spec's 0.5-0.7 m range).
STAGING_STANDOFF_M = 0.6

#: Chassis standoff separating `docks.yaml`'s `dock_pose` (the dock's back
#: wall) from `perception/config/docking.yaml`'s `pose` (where the ROBOT must
#: end up). Measured live at dock A -- see docking.yaml's own header comment;
#: this module only re-applies that measurement, it does not re-derive it.
DOCKING_CHASSIS_OFFSET_M = 0.126

#: The arena's fixed tag-id -> dock-letter table. 8 docks, 8 tags, as placed
#: in `arena.sdf` and recorded in `docks.yaml`'s `tag_id` fields.
TAG_ID_TO_LETTER = {0: 'A', 1: 'B', 2: 'C', 3: 'D',
                    4: 'E', 5: 'F', 6: 'G', 7: 'H'}

#: `opennav_docking` dock-plugin instance name, as configured in
#: `perception/config/docking.yaml` (`dock_plugins: ["arena_dock"]`).
DOCKING_PLUGIN_TYPE = 'arena_dock'

#: Frame all captured poses are expressed in.
MAP_FRAME = 'map'

#: Decimals used when writing numbers back into the two YAML files. Chosen so
#: that re-capturing a dock at its currently-recorded pose reproduces the
#: existing line BYTE FOR BYTE (see test_formatting_reproduces_existing_*):
#: x/y are written with a fixed 3 decimals (the files use -1.500, 2.992),
#: theta with up to 5 and trailing zeros trimmed (the files use 3.14159,
#: -1.5708, 0.0). Sub-millimetre / sub-0.001-rad precision is well below what
#: SLAM localization and a hand-driven capture can deliver anyway.
POSITION_DECIMALS = 3
THETA_DECIMALS = 5

#: Below this, the tag's normal is too close to vertical for its projection
#: into the map XY-plane to mean anything (1e-3 ~ within 0.06 deg of straight
#: up or straight down).
MIN_HORIZONTAL_NORMAL = 1e-3

#: `classify_detection_age` verdicts.
AGE_OK = 'ok'
AGE_STALE = 'stale'
AGE_CLOCK_MISMATCH = 'clock_mismatch'

#: An age this absurd is not staleness, it is two different clock domains
#: (a node without `use_sim_time:=true` next to a sim publishing /clock from
#: ~0, or vice versa) -- a mistake this stack has made on itself before, so it
#: gets its own diagnosis instead of being reported as a stale tag.
CLOCK_MISMATCH_AGE_SEC = 1000.0


class DockCaptureError(Exception):
    """Base for the per-tag failures a capture reports and then skips past."""


class DegenerateTagOrientation(DockCaptureError):
    """The tag's outward normal is (near) vertical, so it has no map heading."""


class UnknownTagId(DockCaptureError):
    """The tag id is outside the arena's 0-7 dock tags."""


class DockEntryNotFound(DockCaptureError):
    """No (or more than one) line in the YAML file to replace for this dock."""


class TagTransformUnavailable(DockCaptureError):
    """`map` -> tag TF could not be resolved for this tag.

    Raised by the caller's own lookup callable (the node wraps
    `tf2_ros.TransformException` in it), so that `capture_visible_tags` can
    treat a TF failure as just another per-tag outcome without this module
    importing tf2.
    """


#: `CaptureOutcome.error_kind` values -- which stage of one tag's capture
#: failed. The node words its log message differently for each, so this is not
#: decoration: a frame-name problem and a bad tag id need completely different
#: things from the operator.
ERROR_TRANSFORM = 'transform'
ERROR_TAG_ID = 'tag_id'
ERROR_ORIENTATION = 'orientation'
ERROR_WRITE = 'write'


@dataclass(frozen=True)
class Pose2D:
    """A planar pose in `map`. `theta` is always normalized to (-pi, pi]."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class DockCapture:
    """Everything one captured tag implies, ready to be written out."""

    letter: str
    tag_id: int
    dock_pose: Pose2D
    staging_pose: Pose2D
    docking_pose: Pose2D


def normalize_angle(theta: float) -> float:
    """Normalize an angle to (-pi, pi], matching `docks.yaml`'s convention.

    `math.remainder` lands in the CLOSED range [-pi, pi]; the explicit -pi ->
    +pi fold is what makes the range half-open the way docks.yaml's header
    comment specifies. It matters in practice: a dock on the +x wall has
    theta = pi exactly, and `atan2(-0.0, -1.0)` returns -pi, so without this
    fold half the +x-wall captures would be written as -3.14159 and half as
    3.14159 depending on a sign of zero.

    The `+ 0.0` kills negative zero for the same reason: `-0.0` formats as
    "-0.0", which would make an otherwise byte-identical re-capture show up as
    a spurious diff.
    """
    wrapped = math.remainder(theta, math.tau)
    if wrapped <= -math.pi:
        wrapped = math.pi
    return wrapped + 0.0


def tag_outward_theta(rotation_xyzw) -> float:
    """Map heading of the tag's outward (room-facing) normal, from its TF quaternion.

    `rotation_xyzw` is the (x, y, z, w) of a `map` -> tag-frame transform's
    rotation, i.e. `TransformStamped.transform.rotation`.

    WHICH LOCAL AXIS AND WHY: the tag frame's local **+z** axis, because
    apriltag_ros's tag frame has +x right / +y up as seen facing the tag,
    hence +z out of the tag's printed face toward the observer -- see this
    module's header for the full derivation out of the installed
    libAprilTagNode.so and the apriltag library's own corner order. For a tag
    mounted flat on a dock's back wall, "out of the printed face" is exactly
    "away from the wall, into the room", which is what docks.yaml means by
    `dock_pose.theta`.

    Only the third COLUMN of the rotation matrix is needed (the local +z axis
    expressed in `map`); the full matrix is not built.

    Raises DegenerateTagOrientation if that axis is (near) vertical in `map`,
    i.e. the tag is facing the floor or the ceiling rather than into the room.
    A real wall dock cannot do that, so it means the detection or the TF chain
    is wrong, and silently projecting it would invent a heading out of
    rounding noise.
    """
    x, y, z, w = (float(value) for value in rotation_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise DegenerateTagOrientation(
            'tag orientation quaternion is all zeros (not a rotation)')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    # Third column of the rotation matrix = the tag's local +z axis in `map`.
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
    """`docks.yaml`'s documented staging formula: stand `standoff_m` out along
    the dock's own outward normal and look back at it.

        staging.x     = dock.x + d * cos(dock.theta)
        staging.y     = dock.y + d * sin(dock.theta)
        staging.theta = dock.theta + pi
    """
    return Pose2D(
        x=dock_pose.x + standoff_m * math.cos(dock_pose.theta),
        y=dock_pose.y + standoff_m * math.sin(dock_pose.theta),
        theta=normalize_angle(dock_pose.theta + math.pi),
    )


def docking_server_pose_for(dock_pose: Pose2D,
                            offset_m: float = DOCKING_CHASSIS_OFFSET_M) -> Pose2D:
    """`perception/config/docking.yaml`'s pose for the same dock.

    Two differences from `dock_pose`, both documented at length in that file's
    own header comment and only re-applied here:

    - position moved `offset_m` OUTWARD along the dock's outward normal,
      because docks.yaml's dock_pose is the dock's back WALL while
      opennav_docking's pose is where the robot's `base_frame` must end up,
      and the chassis nose physically cannot get closer than ~0.123 m to that
      wall;
    - theta rotated by pi, because opennav_docking's dock pose.theta is the
      robot's own final NOSE-FORWARD heading once docked (into the wall), not
      the tag's outward normal.
    """
    return Pose2D(
        x=dock_pose.x + offset_m * math.cos(dock_pose.theta),
        y=dock_pose.y + offset_m * math.sin(dock_pose.theta),
        theta=normalize_angle(dock_pose.theta + math.pi),
    )


def letter_for_tag_id(tag_id: int) -> str:
    """Dock letter for an arena tag id. Raises UnknownTagId outside 0-7."""
    try:
        return TAG_ID_TO_LETTER[int(tag_id)]
    except (KeyError, TypeError, ValueError):
        raise UnknownTagId(
            f'tag id {tag_id} is not one of the arena dock tags 0-7') from None


def capture_from_tag_transform(tag_id: int, translation_xy, rotation_xyzw,
                               standoff_m: float = STAGING_STANDOFF_M,
                               docking_offset_m: float = DOCKING_CHASSIS_OFFSET_M,
                               ) -> DockCapture:
    """Everything one visible tag implies, from its `map` -> tag transform.

    `translation_xy` is (x, y) in `map` (the tag's own height is irrelevant --
    all three poses are planar); `rotation_xyzw` is that transform's
    quaternion.
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


@dataclass(frozen=True)
class CaptureOutcome:
    """What happened to ONE tag in one capture."""

    tag_id: int
    frame: str
    capture: DockCapture = None
    error_kind: str = None
    error: str = None

    @property
    def succeeded(self) -> bool:
        return self.capture is not None


def capture_visible_tags(tags, lookup_transform, write_capture):
    """Capture every tag in one detection message, independently of the others.

    `tags` is an iterable of (tag_id, family) pairs -- whatever was in the
    detection array. `lookup_transform(frame)` returns
    ((x, y), (qx, qy, qz, qw)) in `map` or raises TagTransformUnavailable.
    `write_capture(capture)` persists one capture (normally `apply_capture`
    bound to the two file paths).

    Returns one CaptureOutcome per tag, in order. Every EXPECTED per-tag
    failure -- a broken TF, an id outside 0-7, a tag with no horizontal normal,
    an entry missing from a YAML file -- becomes an outcome rather than an
    exception, so it cannot abort the capture of the other tags in the same
    message. Two neighbouring docks in one camera frame is the normal case
    here, not an edge case, and losing the good one because the other failed
    would mean re-driving the robot for nothing. Anything else (a genuine bug
    in here, a disk that vanished mid-write) still propagates: this is not a
    blanket `except Exception`.

    The caller does the reporting. This function does not log, so the whole
    per-tag success/failure behaviour is testable with two callables and no
    ROS node (see test_dock_capture.py).
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
    """AGE_OK / AGE_STALE / AGE_CLOCK_MISMATCH for a detection message's age.

    `age_sec` is (now - message stamp), so it is NEGATIVE for a stamp in the
    future. Both directions are bounded, because both are real failures here:
    an old stamp means the tag left the camera's view (or the robot has since
    moved, which is exactly what a stale capture would silently mis-record),
    and a wildly off stamp in either direction means the detection and this
    node are on different clocks.
    """
    if abs(age_sec) >= CLOCK_MISMATCH_AGE_SEC:
        return AGE_CLOCK_MISMATCH
    if not -max_age_sec <= age_sec <= max_age_sec:
        return AGE_STALE
    return AGE_OK


def tag_tf_frame(family: str, tag_id: int) -> str:
    """The TF frame name apriltag_ros broadcasts for one detected tag.

    `family` is the detection message's own `family` field, NOT the `family`
    parameter from apriltag.yaml ("36h11"): apriltag_ros fills both the
    message field and the TF `child_frame_id` from the same
    `det->family->name` string ("tag36h11" for this arena's tags), joined to
    the id with a single ":" -- verified by reading the installed
    libAprilTagNode.so, where both assignments load offset 0x38 of the
    apriltag family struct (its `name`). Deriving the frame name from the
    message's own field rather than hardcoding it is what makes this correct
    by construction instead of by assumption.

    (This is apriltag_ros's DEFAULT naming, which is what
    `perception/config/apriltag.yaml` uses -- it deliberately sets no
    `tag.frames` overrides.)
    """
    return f'{family}:{int(tag_id)}'


def _fixed(value: float, decimals: int) -> str:
    """Fixed-decimal formatting with "-0.000" normalized to "0.000"."""
    text = f'{value:.{decimals}f}'
    if text.startswith('-') and float(text) == 0.0:
        text = text[1:]
    return text


def _theta(value: float) -> str:
    """Theta formatting: up to THETA_DECIMALS decimals, trailing zeros
    trimmed, always at least one decimal place.

    This is what makes a re-capture of an unchanged dock produce a
    byte-identical line: the existing files write 3.14159, -1.5708 and 0.0,
    which is exactly this rule applied to pi, -pi/2 and 0.
    """
    text = _fixed(value, THETA_DECIMALS)
    trimmed = text.rstrip('0')
    if trimmed.endswith('.'):
        trimmed += '0'
    return trimmed


def _pose_fields(pose: Pose2D) -> str:
    return (f'x: {_fixed(pose.x, POSITION_DECIMALS)}, '
            f'y: {_fixed(pose.y, POSITION_DECIMALS)}, '
            f'theta: {_theta(pose.theta)}')


def format_docks_entry(capture: DockCapture) -> str:
    """One `mission/config/docks.yaml` dock line, WITHOUT its leading indent.

    Matches that file's existing flow-style one-line-per-dock format exactly;
    the caller re-uses the indentation of the line being replaced.
    """
    return (f'{capture.letter}: {{tag_id: {capture.tag_id}, '
            f'dock_pose: {{{_pose_fields(capture.dock_pose)}}}, '
            f'staging_pose: {{{_pose_fields(capture.staging_pose)}}}}}')


def format_docking_entry(capture: DockCapture) -> str:
    """One `perception/config/docking.yaml` dock line, WITHOUT its indent.

    A DIFFERENT schema from docks.yaml's -- this is opennav_docking's own
    dock-database format (`{type, frame, pose: [x, y, theta], id}`, parsed by
    its `parseDockParams`), with `id` the AprilTag id as a STRING and the pose
    a 3-element flow sequence rather than a mapping.
    """
    pose = capture.docking_pose
    return (f'{capture.letter}: {{type: "{DOCKING_PLUGIN_TYPE}", '
            f'frame: "{MAP_FRAME}", '
            f'pose: [{_fixed(pose.x, POSITION_DECIMALS)}, '
            f'{_fixed(pose.y, POSITION_DECIMALS)}, '
            f'{_theta(pose.theta)}], '
            f'id: "{capture.tag_id}"}}')


def replace_entry_line(text: str, letter: str, new_body: str) -> str:
    """Replace exactly the one line defining dock `letter`, nothing else.

    Targeted line surgery rather than a YAML load/dump round trip, because
    both files are ~60-350 lines of load-bearing header comments explaining
    measurements, false starts and safety exceptions, and `yaml.safe_dump`
    would silently delete every one of them (and re-flow the surviving
    entries' formatting for good measure).

    The matched line's own indentation is preserved, so the same function
    serves docks.yaml's 2-space `docks:` mapping and docking.yaml's 8-space
    `ros__parameters:` block. Raises DockEntryNotFound unless exactly one line
    matches, rather than guessing which of several to rewrite.
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
    """Write one capture into both YAML files, or into neither.

    BOTH replacements are computed in memory before EITHER file is written. The
    two files describe the same physical dock through two different schemas and
    are only useful in agreement (docks.yaml is the source of truth; docking
    .yaml is derived from it), so a dock whose line is missing or duplicated in
    one of them must not leave the other half-updated.
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


#: `interpret_command` verdicts.
COMMAND_CAPTURE = 'capture'
COMMAND_EMPTY = 'empty'
COMMAND_UNKNOWN = 'unknown'

#: What the operator types to capture, and the prompt they see.
CAPTURE_KEY = 'i'
PROMPT = "[capture_dock_pose] ready -- 'i' + Enter to capture, Ctrl-C to quit"


def interpret_command(line: str) -> str:
    """Classify one line typed by the operator.

    Whitespace-trimmed and case-insensitive, so "I", " i " and "i\\n" all
    capture. A bare Enter is COMMAND_EMPTY rather than COMMAND_UNKNOWN: it is
    how people check that an interactive prompt is still alive, and scolding
    them for it would be noise, not feedback (the loop just re-prints the
    prompt).
    """
    stripped = line.strip()
    if not stripped:
        return COMMAND_EMPTY
    if stripped.lower() == CAPTURE_KEY:
        return COMMAND_CAPTURE
    return COMMAND_UNKNOWN


def run_command_loop(stream, on_capture, emit=print, prompt: str = PROMPT) -> None:
    """Read lines off `stream` and dispatch captures until EOF or Ctrl-C.

    Plain blocking line reads, NOT raw/termios single-keypress mode: this has
    to work in whatever terminal, multiplexer or IDE console the operator
    happens to be in, and over ssh, without leaving a half-configured tty
    behind if it dies. The cost is one extra Enter per capture, which is
    nothing next to driving the robot to the next dock.

    This function is deliberately ROS-free -- `on_capture` is just a callable
    and `emit` a one-argument writer -- so the dispatch behaviour is
    unit-testable with a StringIO and a list. Ctrl-C is left to propagate as
    KeyboardInterrupt for `main` to handle; it is the documented way out.

    The prompt is a whole line of its own, re-printed every iteration, rather
    than an inline "> " cursor: this node shares its terminal with its own ROS
    log output (every capture result and every per-tag failure), and an inline
    prompt would be left dangling mid-line by the next log message.
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
    """Frame names out of `tf2_ros.Buffer.all_frames_as_yaml()`'s output.

    Used only to make a failed `map` -> tag lookup diagnosable live: if
    apriltag_ros ever names its tag frames differently from what
    `tag_tf_frame` builds, the list of frames tf2 actually knows is the one
    piece of evidence that says so immediately. Parsed by hand rather than with
    yaml.safe_load because the buffer returns "{}" when empty and because the
    only thing wanted is the top-level keys.
    """
    names = []
    for line in tf_yaml.splitlines():
        if not line or line[0].isspace():
            continue
        # rsplit, not split: a tag frame name CONTAINS a colon
        # ("tag36h11:0"), so only the last one separates key from value.
        name = line.rsplit(':', 1)[0].strip()
        if name and name != '{}':
            names.append(name)
    return sorted(names)


def find_workspace_source_file(share_dir: str, relative_path: str,
                               max_levels: int = 8):
    """Locate the SOURCE copy of a config file, given a package's share dir.

    Captures have to edit `<workspace>/src/<relative_path>`, not the installed
    `share/` copy: the installed copy is overwritten by the next `colcon
    build`, so a capture written there would silently evaporate (and never
    reach git). Walk up from the installed share directory looking for a
    `src/<relative_path>` sibling, which finds the workspace for both the
    merged and the isolated install layouts, and for `--symlink-install`.

    Returns the absolute path, or None if no workspace source tree is in sight
    (an installed-from-a-release deployment, say) -- the caller decides what
    to do about that rather than having a guess forced on it.
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

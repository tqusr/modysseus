"""Unit tests for perception.dock_capture -- the pure logic behind
`ros2 run perception capture_dock_pose`.

No rclpy, no tf2, no ROS graph, no camera, no Gazebo: every test here is plain
Python against plain functions, which is the reason that module exists
separately from the node at all.

The ONE thing these tests cannot do is confirm that a real AprilTag detection
produces the orientation this code expects. What they do instead, and the
closest available substitute:

  - reproduce dock A's existing known-good `docks.yaml` entry from a synthetic
    tag TF built under the derived convention, and
  - independently re-derive that same convention through `cv2.solvePnP`, using
    apriltag_ros's own object points and apriltag's own corner order (both
    read out of the installed binaries -- see dock_capture's module docstring),
    so the convention is checked against the actual pose-estimation maths and
    not just against itself.

See this feature's report for the live-validation procedure that is still
owed.
"""

import io
import math

import pytest

from perception.dock_capture import (AGE_CLOCK_MISMATCH, AGE_OK, AGE_STALE,
                                     COMMAND_CAPTURE, COMMAND_EMPTY,
                                     COMMAND_UNKNOWN, DOCKING_CHASSIS_OFFSET_M,
                                     ERROR_ORIENTATION, ERROR_TAG_ID,
                                     ERROR_TRANSFORM, ERROR_WRITE,
                                     STAGING_STANDOFF_M, DegenerateTagOrientation,
                                     DockEntryNotFound, Pose2D,
                                     TagTransformUnavailable, UnknownTagId,
                                     apply_capture, capture_from_tag_transform,
                                     capture_visible_tags,
                                     classify_detection_age,
                                     docking_server_pose_for,
                                     find_workspace_source_file,
                                     format_docking_entry, format_docks_entry,
                                     frame_names_from_tf_yaml,
                                     interpret_command, letter_for_tag_id,
                                     normalize_angle, replace_entry_line,
                                     run_command_loop, staging_pose_for,
                                     tag_outward_theta, tag_tf_frame)


def test_perception_package_importable():
    import perception  # noqa: F401


# --------------------------------------------------------------------------
# Synthetic tag transforms
# --------------------------------------------------------------------------

def rotation_matrix_to_quaternion(matrix):
    """(x, y, z, w) for a 3x3 rotation matrix, as a TF message would carry it.

    Test-side helper only: the production code never needs to go this
    direction. Shepherd's/standard trace method, branching on the largest
    diagonal term for numerical stability -- the same approach tf2's own
    Matrix3x3::getRotation uses.
    """
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def upright_wall_tag_quaternion(outward_theta):
    """Quaternion of an upright wall-mounted tag whose outward normal is `outward_theta`.

    Built from the convention under test, stated as axes rather than as a
    formula so a reader can check it by eye:

      tag local +z = the outward (room-facing) normal, horizontal
      tag local +y = straight up in `map` (the tag is not tilted)
      tag local +x = +y cross +z, i.e. the viewer's right facing the tag

    which is exactly apriltag_ros's tag frame (x right, y up, z out of the
    printed face) for a tag hanging flat and upright on a wall.
    """
    z_axis = (math.cos(outward_theta), math.sin(outward_theta), 0.0)
    y_axis = (0.0, 0.0, 1.0)
    x_axis = (y_axis[1] * z_axis[2] - y_axis[2] * z_axis[1],
              y_axis[2] * z_axis[0] - y_axis[0] * z_axis[2],
              y_axis[0] * z_axis[1] - y_axis[1] * z_axis[0])
    matrix = [[x_axis[i], y_axis[i], z_axis[i]] for i in range(3)]
    return rotation_matrix_to_quaternion(matrix)


# --------------------------------------------------------------------------
# normalize_angle
# --------------------------------------------------------------------------

@pytest.mark.parametrize('given, expected', [
    (0.0, 0.0),
    (math.pi, math.pi),
    (-math.pi, math.pi),          # (-pi, pi] is half-open: -pi folds to +pi
    (2 * math.pi, 0.0),
    (3 * math.pi, math.pi),
    (math.pi / 2, math.pi / 2),
    (-math.pi / 2, -math.pi / 2),
    (3 * math.pi / 2, -math.pi / 2),
    (-3 * math.pi / 2, math.pi / 2),
])
def test_normalize_angle(given, expected):
    assert normalize_angle(given) == pytest.approx(expected, abs=1e-12)


def test_normalize_angle_never_returns_negative_zero():
    # -0.0 would format as "-0.0" and show up as a spurious diff against a
    # file that writes 0.0.
    assert math.copysign(1.0, normalize_angle(-0.0)) > 0
    assert math.copysign(1.0, normalize_angle(2 * math.pi)) > 0


# --------------------------------------------------------------------------
# tag_outward_theta: the convention itself
# --------------------------------------------------------------------------

@pytest.mark.parametrize('outward_theta', [
    0.0, math.pi / 2, math.pi, -math.pi / 2, 0.7, -2.3,
])
def test_tag_outward_theta_recovers_the_normal_it_was_built_from(outward_theta):
    quaternion = upright_wall_tag_quaternion(outward_theta)
    assert tag_outward_theta(quaternion) == pytest.approx(
        normalize_angle(outward_theta), abs=1e-9)


def test_tag_outward_theta_uses_local_z_not_local_x():
    """A tag whose +z and +x point in different directions must follow +z.

    Guards the specific mistake this convention is most likely to be got
    wrong by: reading the tag frame's "forward" as +x (the ROS body-frame
    habit) instead of +z (the camera/optical habit apriltag_ros actually
    follows).
    """
    quaternion = upright_wall_tag_quaternion(math.pi)
    # For outward_theta = pi: +z is -x of map, while +x of the tag is +y of map.
    assert tag_outward_theta(quaternion) == pytest.approx(math.pi, abs=1e-9)


def test_tag_outward_theta_is_insensitive_to_quaternion_scale():
    x, y, z, w = upright_wall_tag_quaternion(0.7)
    scaled = (2 * x, 2 * y, 2 * z, 2 * w)
    assert tag_outward_theta(scaled) == pytest.approx(0.7, abs=1e-9)


#: Columns are the tag's local x/y/z axes in `map`. The identity is a tag lying
#: flat on the floor facing the CEILING (local +z == map +z); the second is the
#: same tag face-down. Neither has a heading in the XY plane.
CEILING_FACING_TAG = [[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]]
FLOOR_FACING_TAG = [[1.0, 0.0, 0.0],
                    [0.0, -1.0, 0.0],
                    [0.0, 0.0, -1.0]]


@pytest.mark.parametrize('matrix', [CEILING_FACING_TAG, FLOOR_FACING_TAG])
def test_tag_outward_theta_rejects_a_tag_with_no_horizontal_normal(matrix):
    with pytest.raises(DegenerateTagOrientation):
        tag_outward_theta(rotation_matrix_to_quaternion(matrix))


def test_tag_outward_theta_rejects_a_zero_quaternion():
    with pytest.raises(DegenerateTagOrientation):
        tag_outward_theta((0.0, 0.0, 0.0, 0.0))


# --------------------------------------------------------------------------
# The derived poses
# --------------------------------------------------------------------------

def test_staging_pose_matches_docks_yaml_formula():
    dock = Pose2D(x=2.992, y=-1.500, theta=math.pi)
    staging = staging_pose_for(dock)
    assert staging.x == pytest.approx(2.992 - STAGING_STANDOFF_M)
    assert staging.y == pytest.approx(-1.500)
    assert staging.theta == pytest.approx(0.0)


def test_staging_pose_walks_out_along_the_normal_not_into_the_wall():
    # Dock G sits on the -y wall; its outward normal is +y, so staging must be
    # at a LARGER y than the dock, further into the room.
    dock = Pose2D(x=-1.500, y=-2.992, theta=math.pi / 2)
    staging = staging_pose_for(dock)
    assert staging.y > dock.y
    assert staging.y == pytest.approx(-2.992 + STAGING_STANDOFF_M)
    assert staging.theta == pytest.approx(-math.pi / 2)


def test_docking_server_pose_matches_docking_yaml_derivation():
    dock = Pose2D(x=2.992, y=-1.500, theta=math.pi)
    pose = docking_server_pose_for(dock)
    assert pose.x == pytest.approx(2.992 - DOCKING_CHASSIS_OFFSET_M)
    assert pose.y == pytest.approx(-1.500)
    assert pose.theta == pytest.approx(0.0)


def test_docking_pose_sits_between_dock_and_staging():
    """The robot's docked pose must be inside the dock's staging standoff.

    Sanity check on the two offsets' signs relative to each other: both walk
    outward along the same normal, and the chassis standoff is much the
    smaller of the two.
    """
    dock = Pose2D(x=1.500, y=2.992, theta=-math.pi / 2)
    docking = docking_server_pose_for(dock)
    staging = staging_pose_for(dock)
    assert dock.y > docking.y > staging.y
    assert docking.theta == pytest.approx(staging.theta)


def test_letter_for_tag_id_covers_all_eight_docks():
    assert [letter_for_tag_id(i) for i in range(8)] == list('ABCDEFGH')


@pytest.mark.parametrize('tag_id', [-1, 8, 42, 999])
def test_letter_for_tag_id_rejects_non_dock_tags(tag_id):
    with pytest.raises(UnknownTagId):
        letter_for_tag_id(tag_id)


def test_capture_from_tag_transform_reproduces_dock_a(dock_a_capture):
    """The anchor test: dock A's KNOWN-GOOD entry, from a synthetic tag TF.

    `docks.yaml` currently records, from arena.sdf's model origins:
        A: {tag_id: 0, dock_pose: {x: 2.992, y: -1.500, theta: 3.14159},
                       staging_pose: {x: 2.392, y: -1.500, theta: 0.0}}

    Dock A is on the arena's +x wall, so its tag faces -x (into the room): an
    upright tag at (2.992, -1.500) whose outward normal is -x. Feed exactly
    that through the real code path and the recorded numbers must come back
    out. This is the best available stand-in for live validation.
    """
    capture = dock_a_capture
    assert capture.letter == 'A'
    assert capture.tag_id == 0
    assert capture.dock_pose.x == pytest.approx(2.992)
    assert capture.dock_pose.y == pytest.approx(-1.500)
    assert capture.dock_pose.theta == pytest.approx(3.14159, abs=1e-5)
    assert capture.staging_pose.x == pytest.approx(2.392)
    assert capture.staging_pose.y == pytest.approx(-1.500)
    assert capture.staging_pose.theta == pytest.approx(0.0, abs=1e-9)
    # And docking.yaml's own derived entry for the same dock.
    assert capture.docking_pose.x == pytest.approx(2.866)
    assert capture.docking_pose.y == pytest.approx(-1.500)
    assert capture.docking_pose.theta == pytest.approx(0.0, abs=1e-9)


def test_capture_from_tag_transform_rejects_unknown_tag():
    with pytest.raises(UnknownTagId):
        capture_from_tag_transform(
            11, (1.0, 2.0), upright_wall_tag_quaternion(0.0))


# --------------------------------------------------------------------------
# Line formatting -- against the real files' current contents
# --------------------------------------------------------------------------

#: The arena's 8 docks as the two config files currently record them, with
#: theta as the EXACT geometry (0, +-pi/2, pi) the files' rounded constants
#: stand for. `expected_docks_line` / `expected_docking_line` are verbatim from
#: the files, minus their leading indentation.
ARENA_DOCKS = [
    # letter, tag_id, dock x, dock y, dock theta
    ('A', 0, 2.992, -1.500, math.pi),
    ('B', 1, 2.992, 1.500, math.pi),
    ('C', 2, 1.500, 2.992, -math.pi / 2),
    ('D', 3, -1.500, 2.992, -math.pi / 2),
    ('E', 4, -2.992, 1.500, 0.0),
    ('F', 5, -2.992, -1.500, 0.0),
    ('G', 6, -1.500, -2.992, math.pi / 2),
    ('H', 7, 1.500, -2.992, math.pi / 2),
]

EXPECTED_DOCKS_LINES = {
    'A': 'A: {tag_id: 0, dock_pose: {x: 2.992, y: -1.500, theta: 3.14159}, staging_pose: {x: 2.392, y: -1.500, theta: 0.0}}',
    'B': 'B: {tag_id: 1, dock_pose: {x: 2.992, y: 1.500, theta: 3.14159}, staging_pose: {x: 2.392, y: 1.500, theta: 0.0}}',
    'C': 'C: {tag_id: 2, dock_pose: {x: 1.500, y: 2.992, theta: -1.5708}, staging_pose: {x: 1.500, y: 2.392, theta: 1.5708}}',
    'D': 'D: {tag_id: 3, dock_pose: {x: -1.500, y: 2.992, theta: -1.5708}, staging_pose: {x: -1.500, y: 2.392, theta: 1.5708}}',
    'E': 'E: {tag_id: 4, dock_pose: {x: -2.992, y: 1.500, theta: 0.0}, staging_pose: {x: -2.392, y: 1.500, theta: 3.14159}}',
    'F': 'F: {tag_id: 5, dock_pose: {x: -2.992, y: -1.500, theta: 0.0}, staging_pose: {x: -2.392, y: -1.500, theta: 3.14159}}',
    'G': 'G: {tag_id: 6, dock_pose: {x: -1.500, y: -2.992, theta: 1.5708}, staging_pose: {x: -1.500, y: -2.392, theta: -1.5708}}',
    'H': 'H: {tag_id: 7, dock_pose: {x: 1.500, y: -2.992, theta: 1.5708}, staging_pose: {x: 1.500, y: -2.392, theta: -1.5708}}',
}

EXPECTED_DOCKING_LINES = {
    'A': 'A: {type: "arena_dock", frame: "map", pose: [2.866, -1.500, 0.0], id: "0"}',
    'B': 'B: {type: "arena_dock", frame: "map", pose: [2.866, 1.500, 0.0], id: "1"}',
    'C': 'C: {type: "arena_dock", frame: "map", pose: [1.500, 2.866, 1.5708], id: "2"}',
    'D': 'D: {type: "arena_dock", frame: "map", pose: [-1.500, 2.866, 1.5708], id: "3"}',
    'E': 'E: {type: "arena_dock", frame: "map", pose: [-2.866, 1.500, 3.14159], id: "4"}',
    'F': 'F: {type: "arena_dock", frame: "map", pose: [-2.866, -1.500, 3.14159], id: "5"}',
    'G': 'G: {type: "arena_dock", frame: "map", pose: [-1.500, -2.866, -1.5708], id: "6"}',
    'H': 'H: {type: "arena_dock", frame: "map", pose: [1.500, -2.866, -1.5708], id: "7"}',
}


def capture_for(letter, tag_id, x, y, theta):
    return capture_from_tag_transform(
        tag_id, (x, y), upright_wall_tag_quaternion(theta))


@pytest.mark.parametrize('letter, tag_id, x, y, theta', ARENA_DOCKS)
def test_formatting_reproduces_existing_docks_yaml_lines(letter, tag_id, x, y, theta):
    """Re-capturing an unchanged dock must produce a BYTE-IDENTICAL line.

    Not cosmetic: it is what makes the live dock-A sanity check readable as
    `git diff` showing nothing at all, and it pins the number formatting
    (fixed 3 decimals on x/y, up to 5 with trailing zeros trimmed on theta)
    against the files as they actually are rather than against a description
    of them.
    """
    capture = capture_for(letter, tag_id, x, y, theta)
    assert format_docks_entry(capture) == EXPECTED_DOCKS_LINES[letter]


@pytest.mark.parametrize('letter, tag_id, x, y, theta', ARENA_DOCKS)
def test_formatting_reproduces_existing_docking_yaml_lines(letter, tag_id, x, y, theta):
    capture = capture_for(letter, tag_id, x, y, theta)
    assert format_docking_entry(capture) == EXPECTED_DOCKING_LINES[letter]


def test_docks_and_docking_entries_use_their_own_schemas():
    """The two files are not transcriptions of each other."""
    capture = capture_for(*ARENA_DOCKS[0])
    docks_line = format_docks_entry(capture)
    docking_line = format_docking_entry(capture)
    assert 'staging_pose' in docks_line and 'staging_pose' not in docking_line
    assert 'type:' in docking_line and 'type:' not in docks_line
    assert 'pose: [' in docking_line and 'pose: [' not in docks_line


# --------------------------------------------------------------------------
# Targeted line replacement
# --------------------------------------------------------------------------

#: Mimics the real files: comment-heavy, one flow-style line per dock, and a
#: `docks:` list whose quoted letters must NOT be mistaken for entries.
FIXTURE_DOCKS_YAML = """\
# Real arena dock poses (world == `map` frame).
#
# staging_pose formula:
#   staging_pose.x = dock_pose.x + d * cos(dock_pose.theta)
#
# A: this comment mentions dock A and must survive untouched.
docks:
  A: {tag_id: 0, dock_pose: {x: 2.992, y: -1.500, theta: 3.14159}, staging_pose: {x: 2.392, y: -1.500, theta: 0.0}}
  B: {tag_id: 1, dock_pose: {x: 2.992, y: 1.500, theta: 3.14159}, staging_pose: {x: 2.392, y: 1.500, theta: 0.0}}
# trailing comment
"""

FIXTURE_DOCKING_YAML = """\
### opennav_docking's docking_server parameters.
#
# theta here is DELIBERATELY ROTATED BY PI from docks.yaml's theta.
docking_server:
    ros__parameters:
        use_sim_time: true
        docks: ["A", "B"]
        A: {type: "arena_dock", frame: "map", pose: [2.866, -1.500, 0.0], id: "0"}
        B: {type: "arena_dock", frame: "map", pose: [2.866, 1.500, 0.0], id: "1"}
        # a trailing comment about dock A
"""


def test_replace_entry_line_preserves_every_other_line_byte_for_byte():
    updated = replace_entry_line(FIXTURE_DOCKS_YAML, 'A', 'A: {replaced}')
    original_lines = FIXTURE_DOCKS_YAML.splitlines(keepends=True)
    updated_lines = updated.splitlines(keepends=True)

    assert len(updated_lines) == len(original_lines)
    for index, (before, after) in enumerate(zip(original_lines, updated_lines)):
        if index == 7:  # the A: line
            assert after == '  A: {replaced}\n'
        else:
            assert after == before


def test_replace_entry_line_preserves_the_matched_lines_indentation():
    updated = replace_entry_line(FIXTURE_DOCKING_YAML, 'B', 'B: {new}')
    assert '        B: {new}\n' in updated
    # 8-space block indent, not docks.yaml's 2.
    assert '  B: {new}\n' not in updated.replace('        B: {new}\n', '')


def test_replace_entry_line_ignores_comments_and_quoted_letters():
    updated = replace_entry_line(FIXTURE_DOCKING_YAML, 'A', 'A: {new}')
    assert 'docks: ["A", "B"]' in updated
    assert '# a trailing comment about dock A' in updated
    assert "# theta here is DELIBERATELY ROTATED BY PI" in updated


def test_replace_entry_line_rejects_a_missing_dock():
    with pytest.raises(DockEntryNotFound):
        replace_entry_line(FIXTURE_DOCKS_YAML, 'H', 'H: {new}')


def test_replace_entry_line_rejects_a_duplicated_dock():
    duplicated = FIXTURE_DOCKS_YAML + '  A: {tag_id: 0}\n'
    with pytest.raises(DockEntryNotFound):
        replace_entry_line(duplicated, 'A', 'A: {new}')


def test_replace_entry_line_keeps_a_file_without_a_trailing_newline_intact():
    text = 'docks:\n  A: {tag_id: 0}'
    assert replace_entry_line(text, 'A', 'A: {new}') == 'docks:\n  A: {new}'


# --------------------------------------------------------------------------
# apply_capture: both files, or neither
# --------------------------------------------------------------------------

@pytest.fixture
def dock_a_capture():
    return capture_from_tag_transform(
        0, (2.992, -1.500), upright_wall_tag_quaternion(math.pi))


@pytest.fixture
def yaml_pair(tmp_path):
    docks = tmp_path / 'docks.yaml'
    docking = tmp_path / 'docking.yaml'
    docks.write_text(FIXTURE_DOCKS_YAML)
    docking.write_text(FIXTURE_DOCKING_YAML)
    return docks, docking


def test_apply_capture_writes_both_files(yaml_pair):
    docks, docking = yaml_pair
    capture = capture_from_tag_transform(
        1, (1.234, -5.678), upright_wall_tag_quaternion(0.0))

    apply_capture(capture, str(docks), str(docking))

    assert ('  B: {tag_id: 1, dock_pose: {x: 1.234, y: -5.678, theta: 0.0}, '
            'staging_pose: {x: 1.834, y: -5.678, theta: 3.14159}}'
            in docks.read_text())
    assert ('        B: {type: "arena_dock", frame: "map", '
            'pose: [1.360, -5.678, 3.14159], id: "1"}'
            in docking.read_text())
    # The other dock, and the comments, are untouched in both files.
    assert 'A: {tag_id: 0, dock_pose: {x: 2.992' in docks.read_text()
    assert 'docks: ["A", "B"]' in docking.read_text()


def test_apply_capture_is_idempotent_for_an_unchanged_dock(yaml_pair, dock_a_capture):
    docks, docking = yaml_pair
    before = (docks.read_text(), docking.read_text())

    apply_capture(dock_a_capture, str(docks), str(docking))

    assert (docks.read_text(), docking.read_text()) == before


def test_apply_capture_writes_nothing_when_one_file_lacks_the_dock(tmp_path):
    """A dock missing from docking.yaml must not half-update docks.yaml."""
    docks = tmp_path / 'docks.yaml'
    docking = tmp_path / 'docking.yaml'
    docks.write_text(FIXTURE_DOCKS_YAML)
    docking.write_text(FIXTURE_DOCKING_YAML.replace(
        '        B: {type: "arena_dock"', '        Z: {type: "arena_dock"'))
    before = (docks.read_text(), docking.read_text())

    capture = capture_from_tag_transform(
        1, (1.0, 2.0), upright_wall_tag_quaternion(0.0))
    with pytest.raises(DockEntryNotFound):
        apply_capture(capture, str(docks), str(docking))

    assert (docks.read_text(), docking.read_text()) == before


# --------------------------------------------------------------------------
# capture_visible_tags: one tag's failure must not cost the others
# --------------------------------------------------------------------------

def upright_lookup(poses):
    """A lookup_transform stand-in: frame -> ((x, y), quaternion), or failure.

    `poses` maps a TF frame name to (x, y, outward_theta). Any frame not in it
    raises TagTransformUnavailable, which is what the node's real lookup does
    when tf2 cannot resolve the chain.
    """
    def lookup(frame):
        if frame not in poses:
            raise TagTransformUnavailable(f'map<-{frame}: frame does not exist')
        x, y, theta = poses[frame]
        return ((x, y), upright_wall_tag_quaternion(theta))
    return lookup


def test_capture_visible_tags_captures_every_tag_in_one_message():
    written = []
    outcomes = capture_visible_tags(
        [(0, 'tag36h11'), (1, 'tag36h11')],
        upright_lookup({'tag36h11:0': (2.992, -1.500, math.pi),
                        'tag36h11:1': (2.992, 1.500, math.pi)}),
        written.append,
    )

    assert [outcome.succeeded for outcome in outcomes] == [True, True]
    assert [capture.letter for capture in written] == ['A', 'B']


def test_capture_visible_tags_skips_only_the_tag_whose_tf_failed():
    """The edge case this function exists for.

    Three tags visible, the middle one's TF unresolvable: the other two are
    still captured and written, and the failure is reported against that tag
    alone with the frame name that failed.
    """
    written = []
    outcomes = capture_visible_tags(
        [(0, 'tag36h11'), (1, 'tag36h11'), (2, 'tag36h11')],
        upright_lookup({'tag36h11:0': (2.992, -1.500, math.pi),
                        'tag36h11:2': (1.500, 2.992, -math.pi / 2)}),
        written.append,
    )

    assert [outcome.succeeded for outcome in outcomes] == [True, False, True]
    assert outcomes[1].error_kind == ERROR_TRANSFORM
    assert outcomes[1].frame == 'tag36h11:1'
    assert 'tag36h11:1' in outcomes[1].error
    assert [capture.letter for capture in written] == ['A', 'C']


def test_capture_visible_tags_skips_a_tag_id_outside_the_arena():
    written = []
    outcomes = capture_visible_tags(
        [(42, 'tag36h11'), (3, 'tag36h11')],
        upright_lookup({'tag36h11:42': (0.0, 0.0, 0.0),
                        'tag36h11:3': (-1.500, 2.992, -math.pi / 2)}),
        written.append,
    )

    assert [outcome.succeeded for outcome in outcomes] == [False, True]
    assert outcomes[0].error_kind == ERROR_TAG_ID
    assert [capture.letter for capture in written] == ['D']


def test_capture_visible_tags_skips_a_tag_with_no_horizontal_normal():
    def lookup(frame):
        return ((1.0, 2.0),
                rotation_matrix_to_quaternion(CEILING_FACING_TAG))

    outcomes = capture_visible_tags([(0, 'tag36h11')], lookup, lambda c: None)

    assert not outcomes[0].succeeded
    assert outcomes[0].error_kind == ERROR_ORIENTATION


def test_capture_visible_tags_reports_a_write_failure_per_tag():
    def failing_write(capture):
        if capture.letter == 'A':
            raise DockEntryNotFound('expected exactly 1 line, found 0')

    written = []

    def write(capture):
        failing_write(capture)
        written.append(capture)

    outcomes = capture_visible_tags(
        [(0, 'tag36h11'), (1, 'tag36h11')],
        upright_lookup({'tag36h11:0': (2.992, -1.500, math.pi),
                        'tag36h11:1': (2.992, 1.500, math.pi)}),
        write,
    )

    assert [outcome.succeeded for outcome in outcomes] == [False, True]
    assert outcomes[0].error_kind == ERROR_WRITE
    assert [capture.letter for capture in written] == ['B']


def test_capture_visible_tags_on_an_empty_message_does_nothing():
    written = []
    assert capture_visible_tags([], upright_lookup({}), written.append) == []
    assert written == []


# --------------------------------------------------------------------------
# Detection staleness
# --------------------------------------------------------------------------

@pytest.mark.parametrize('age_sec, expected', [
    (0.0, AGE_OK),
    (0.3, AGE_OK),
    (-0.3, AGE_OK),          # a slightly-future stamp is fine
    (0.5, AGE_OK),           # the bound itself is inclusive
    (0.51, AGE_STALE),
    (-0.51, AGE_STALE),
    (12.0, AGE_STALE),
    (1000.0, AGE_CLOCK_MISMATCH),
    (1.7e9, AGE_CLOCK_MISMATCH),     # wall clock vs sim /clock from ~0
    (-1.7e9, AGE_CLOCK_MISMATCH),
])
def test_classify_detection_age(age_sec, expected):
    assert classify_detection_age(age_sec, 0.5) == expected


# --------------------------------------------------------------------------
# TF frame naming
# --------------------------------------------------------------------------

def test_tag_tf_frame_matches_apriltag_ros_default_naming():
    # apriltag_ros joins det->family->name and the id with ":"; the detection
    # message's own `family` field is that same string.
    assert tag_tf_frame('tag36h11', 0) == 'tag36h11:0'
    assert tag_tf_frame('tag36h11', 7) == 'tag36h11:7'


def test_frame_names_from_tf_yaml():
    tf_yaml = (
        "odom: \n"
        "  parent: 'map'\n"
        "  broadcaster: '/slam_toolbox'\n"
        "tag36h11:0: \n"
        "  parent: 'oak_rgb_camera_optical_frame'\n"
        "  broadcaster: '/apriltag_node'\n"
    )
    assert frame_names_from_tf_yaml(tf_yaml) == ['odom', 'tag36h11:0']


def test_frame_names_from_tf_yaml_handles_an_empty_buffer():
    assert frame_names_from_tf_yaml('{}') == []
    assert frame_names_from_tf_yaml('') == []


# --------------------------------------------------------------------------
# The stdin command loop
# --------------------------------------------------------------------------

@pytest.mark.parametrize('line, expected', [
    ('i\n', COMMAND_CAPTURE),
    ('I\n', COMMAND_CAPTURE),
    ('  i  \n', COMMAND_CAPTURE),
    ('i', COMMAND_CAPTURE),
    ('\n', COMMAND_EMPTY),
    ('   \n', COMMAND_EMPTY),
    ('q\n', COMMAND_UNKNOWN),
    ('ii\n', COMMAND_UNKNOWN),
    ('capture\n', COMMAND_UNKNOWN),
])
def test_interpret_command(line, expected):
    assert interpret_command(line) == expected


def test_run_command_loop_captures_once_per_i_and_stops_at_eof():
    captures = []
    emitted = []
    run_command_loop(io.StringIO('i\nI\n\nnope\ni\n'),
                     lambda: captures.append(1), emit=emitted.append)

    assert len(captures) == 3
    assert any('unrecognized input' in line for line in emitted)
    # One prompt per read, including the one that hit EOF.
    assert sum(1 for line in emitted if 'ready' in line) == 6


def test_run_command_loop_does_not_scold_a_bare_enter():
    emitted = []
    run_command_loop(io.StringIO('\n'), lambda: None, emit=emitted.append)
    assert not any('unrecognized' in line for line in emitted)


# --------------------------------------------------------------------------
# Locating the workspace source files
# --------------------------------------------------------------------------

def test_find_workspace_source_file_walks_up_to_the_src_tree(tmp_path):
    share = tmp_path / 'install' / 'perception' / 'share' / 'perception'
    share.mkdir(parents=True)
    target = tmp_path / 'src' / 'mission' / 'config' / 'docks.yaml'
    target.parent.mkdir(parents=True)
    target.write_text('docks:\n')

    found = find_workspace_source_file(str(share), 'mission/config/docks.yaml')
    assert found is not None
    assert str(target) == found


def test_find_workspace_source_file_returns_none_when_there_is_no_source_tree(tmp_path):
    share = tmp_path / 'opt' / 'ros' / 'share' / 'perception'
    share.mkdir(parents=True)
    assert find_workspace_source_file(
        str(share), 'mission/config/docks.yaml') is None


def test_find_workspace_source_file_ignores_a_directory_of_the_same_name(tmp_path):
    share = tmp_path / 'install' / 'perception' / 'share' / 'perception'
    share.mkdir(parents=True)
    (tmp_path / 'src' / 'mission' / 'config' / 'docks.yaml').mkdir(parents=True)
    assert find_workspace_source_file(
        str(share), 'mission/config/docks.yaml') is None


# --------------------------------------------------------------------------
# Independent re-derivation of the orientation convention
# --------------------------------------------------------------------------

def test_solvepnp_round_trip_reproduces_dock_a_theta():
    """Re-derive the convention through the actual pose-estimation maths.

    Every other test in this file takes the convention as given (they build a
    synthetic quaternion with `upright_wall_tag_quaternion` and check it comes
    back out). This one does not: it starts from a physical tag's four corners
    in the `map` frame, projects them into a synthetic camera, and hands
    `cv2.solvePnP` exactly what apriltag_ros's `pnp()` hands it --

      objectPoints = [(-s,-s,0), (+s,-s,0), (+s,+s,0), (-s,+s,0)]
      imagePoints  = det->p[0..3] = bottom-left, bottom-right, top-right,
                     top-left of the upright tag as a viewer sees it

    -- the first read out of the installed libAprilTagNode.so, the second
    probed from the apriltag library itself by rendering tag36h11 id 0 with
    `apriltag_to_image()` and detecting it (and consistent with apriltag.h's
    documented ideal-corner order). It then checks that the recovered tag
    frame's +z axis is the outward/room-facing normal and that
    `tag_outward_theta` on the recovered quaternion gives dock A's recorded
    theta.

    Skipped where cv2 is unavailable: this is a verification test, not a
    gate on the feature's own logic.
    """
    cv2 = pytest.importorskip('cv2', reason='opencv-python not installed')
    numpy = pytest.importorskip('numpy')

    tag_size = 0.100           # perception/config/apriltag.yaml
    half = tag_size / 2.0
    tag_centre = numpy.array([2.992, -1.500, 0.200])     # dock A, tag ~0.2 m up
    tag_up = numpy.array([0.0, 0.0, 1.0])
    tag_normal = numpy.array([-1.0, 0.0, 0.0])           # faces -x, into the room
    viewer_right = numpy.cross(tag_up, tag_normal)

    corners_map = [
        tag_centre - viewer_right * half - tag_up * half,   # bottom-left
        tag_centre + viewer_right * half - tag_up * half,   # bottom-right
        tag_centre + viewer_right * half + tag_up * half,   # top-right
        tag_centre - viewer_right * half + tag_up * half,   # top-left
    ]

    # A camera at dock A's staging pose, looking straight at the tag. Optical
    # frame: +x right, +y down, +z forward.
    camera_position = numpy.array([2.392, -1.500, 0.200])
    camera_forward = numpy.array([1.0, 0.0, 0.0])
    camera_down = numpy.array([0.0, 0.0, -1.0])
    camera_right = numpy.cross(camera_down, camera_forward)
    map_from_camera = numpy.column_stack(
        [camera_right, camera_down, camera_forward])
    intrinsics = numpy.array([[600.0, 0.0, 640.0],
                              [0.0, 600.0, 360.0],
                              [0.0, 0.0, 1.0]])

    def project(point_map):
        in_camera = map_from_camera.T @ (point_map - camera_position)
        homogeneous = intrinsics @ in_camera
        return homogeneous[:2] / homogeneous[2]

    image_points = numpy.array([project(corner) for corner in corners_map])
    object_points = numpy.array([[-half, -half, 0.0],
                                 [+half, -half, 0.0],
                                 [+half, +half, 0.0],
                                 [-half, +half, 0.0]])

    # flags=0 is SOLVEPNP_ITERATIVE, which is what apriltag_ros passes.
    ok, rotation_vector, translation = cv2.solvePnP(
        object_points, image_points, intrinsics, None, flags=0)
    assert ok
    camera_from_tag, _ = cv2.Rodrigues(rotation_vector)
    map_from_tag = map_from_camera @ camera_from_tag
    tag_in_map = map_from_camera @ translation.reshape(3) + camera_position

    # The tag is where it was put, and its local +z is the outward normal.
    assert tag_in_map == pytest.approx(tag_centre, abs=1e-6)
    assert map_from_tag[:, 2] == pytest.approx(tag_normal, abs=1e-6)

    # And the production code, fed the recovered orientation, yields dock A's
    # recorded theta.
    quaternion = rotation_matrix_to_quaternion(
        [[map_from_tag[row][col] for col in range(3)] for row in range(3)])
    assert tag_outward_theta(quaternion) == pytest.approx(3.14159, abs=1e-5)

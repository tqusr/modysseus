import math

from visualization_msgs.msg import Marker

from mission.dock_markers import build_dock_markers
from mission.docks import Dock


def _docks():
    return {
        "B": Dock("B", 1, {"x": 3.0, "y": 2.0, "theta": math.pi / 2},
                  {"x": 3.0, "y": 2.6, "theta": -math.pi / 2}),
        "A": Dock("A", 0, {"x": 1.0, "y": 2.0, "theta": 0.0},
                  {"x": 1.6, "y": 2.0, "theta": math.pi}),
    }


def test_three_markers_per_dock_in_map_frame():
    markers = build_dock_markers(_docks()).markers

    assert len(markers) == 6
    assert all(m.header.frame_id == "map" for m in markers)
    # (ns, id) must be unique or later markers replace earlier ones.
    assert len({(m.ns, m.id) for m in markers}) == 6


def test_dock_arrow_matches_dock_pose():
    markers = build_dock_markers(_docks()).markers
    arrow_b = next(m for m in markers if m.ns == "dock_pose" and m.id == 1)

    assert arrow_b.type == Marker.ARROW
    assert (arrow_b.pose.position.x, arrow_b.pose.position.y) == (3.0, 2.0)
    yaw = 2.0 * math.atan2(arrow_b.pose.orientation.z, arrow_b.pose.orientation.w)
    assert math.isclose(yaw, math.pi / 2)


def test_labels_name_dock_and_tag():
    labels = [m.text for m in build_dock_markers(_docks()).markers if m.ns == "label"]

    assert labels == ["A (tag 0)", "B (tag 1)"]

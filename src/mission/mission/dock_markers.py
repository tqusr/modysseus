"""Dock poses from docks.yaml as a MarkerArray, for viewing on the map in
Foxglove or RViz.

Per dock: a green arrow at `dock_pose` pointing along the tag's outward
normal, a blue arrow at `staging_pose` pointing the way the robot faces
there, and a text label with the dock letter and tag id.
"""
import math

from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

DOCK_COLOR = (0.1, 0.8, 0.2, 1.0)
STAGING_COLOR = (0.2, 0.4, 1.0, 1.0)
LABEL_COLOR = (1.0, 1.0, 1.0, 1.0)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _marker(frame_id, ns, marker_id, marker_type, pose, color) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.position.x = float(pose["x"])
    marker.pose.position.y = float(pose["y"])
    marker.pose.orientation = _yaw_to_quaternion(float(pose["theta"]))
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    return marker


def _arrow(frame_id, ns, marker_id, pose, color) -> Marker:
    marker = _marker(frame_id, ns, marker_id, Marker.ARROW, pose, color)
    marker.scale.x = 0.25
    marker.scale.y = 0.04
    marker.scale.z = 0.04
    return marker


def build_dock_markers(docks: dict, frame_id: str = "map") -> MarkerArray:
    array = MarkerArray()
    for index, letter in enumerate(sorted(docks)):
        dock = docks[letter]
        array.markers.append(
            _arrow(frame_id, "dock_pose", index, dock.dock_pose, DOCK_COLOR))
        array.markers.append(
            _arrow(frame_id, "staging_pose", index, dock.staging_pose, STAGING_COLOR))
        label = _marker(frame_id, "label", index, Marker.TEXT_VIEW_FACING,
                        dock.dock_pose, LABEL_COLOR)
        label.pose.position.z = 0.25
        label.scale.z = 0.12
        label.text = f"{letter} (tag {dock.tag_id})"
        array.markers.append(label)
    return array

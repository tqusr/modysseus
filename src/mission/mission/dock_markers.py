"""Dock poses from docks.yaml as a MarkerArray for Foxglove or RViz.

Each dock gets a green arrow at `dock_pose` along the tag's outward normal,
a blue arrow at `staging_pose` along the robot's heading there, and a text
label with the dock letter and tag id.
"""
import math

from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray

DOCK_COLOR = (0.1, 0.8, 0.2, 1.0)
STAGING_COLOR = (0.2, 0.4, 1.0, 1.0)
LABEL_COLOR = (1.0, 1.0, 1.0, 1.0)


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Returns the quaternion for a rotation of `yaw` radians about z."""
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _marker(frame_id, ns, marker_id, marker_type, pose, color) -> Marker:
    """Returns a marker of `marker_type` at `pose` (x, y, theta) in `color` (RGBA)."""
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
    """Returns an arrow marker; see `_marker` for the arguments."""
    marker = _marker(frame_id, ns, marker_id, Marker.ARROW, pose, color)
    marker.scale.x = 0.25
    marker.scale.y = 0.04
    marker.scale.z = 0.04
    return marker


def build_dock_markers(docks: dict, frame_id: str = "map") -> MarkerArray:
    """Builds the markers for all docks.

    Args:
        docks: Dock letter to `mission.docks.Dock`.
        frame_id: Frame the poses are expressed in.

    Returns:
        Three markers per dock: dock arrow, staging arrow and label.
    """
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


def main():
    """Publishes the dock markers without the mission (e.g. `mission:=false`)."""
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile

    from mission.docks import load_docks

    rclpy.init()
    node = Node('dock_markers')
    docks_file = node.declare_parameter(
        'docks_file',
        get_package_share_directory('mission') + '/config/docks.yaml').value
    publisher = node.create_publisher(
        MarkerArray, '~/markers',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(build_dock_markers(load_docks(docks_file)))
    node.get_logger().info(f'Published dock markers from {docks_file}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

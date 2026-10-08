"""Runs apriltag_ros's detector against the OAK-D RGB stream.

Publishes `/detections` (apriltag_msgs/AprilTagDetectionArray), and apriltag_ros
broadcasts a TF per detected tag. `tag_dock_server` measures each dock's tag
from them while docking, and `capture_dock_pose` records dock poses from them.
Localization does not use them: it is the EKF and slam_toolbox only.

Arguments (all nodes run on wall-clock time; there is no /clock on the robot):

  image_topic  /oak/rgb/image_raw
  rectify      true   start image_proc's rectify_node and detect on its output
  gated        false  put perception's image_gate in front of the detector, so
                      it processes frames only while `/image_gate/enable` is
                      true (tag_dock_server opens it for each measurement).
                      Ungated, the detector took ~60% of a CPU core all the
                      time. Leave false where detections are wanted
                      continuously (the mapping mission's capture).

There is no camera_info argument: both nodes subscribe through
image_transport::CameraSubscriber, which always reads the sibling topic
`<image_topic's namespace>/camera_info` (`/oak/rgb/camera_info` by default), so
a remapping would have no effect.

Rectify: apriltag_ros takes intrinsics from the projection matrix P of
camera_info (fx, fy, cx, cy = p[0], p[5], p[2], p[6]) and ignores distortion, so
a distorted lens needs rectifying first. The real OAK-D RGB camera is
distorted, so by default rectify_node turns `image_topic` into
`<same namespace>/image_rect` and the detector reads that. Because it reads P,
no K/P relay is needed. On the robot, check that P in `/oak/rgb/camera_info` is
non-zero and sane.

On-robot QoS risk: rectify_node subscribes lazily, adapts to the publisher's QoS
and publishes image_rect reliable, so the camera's QoS cannot silence it.
apriltag_node's camera_info subscription is reliable (`qos_profile: default` in
apriltag.yaml). If the driver's `/oak/rgb/camera_info` is best-effort,
`/oak/rgb/image_rect` stays alive but `/detections` is silent. Check
`ros2 topic info -v /oak/rgb/camera_info`; the fix is `qos_profile: sensor_data`
in apriltag.yaml.
"""

import posixpath

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    """Returns True for "true" or "1"; launch arguments are text but parameters must be typed."""
    return text.strip().lower() in ('true', '1')


def _rectified_topic(image_topic):
    """Returns the rectified topic, e.g. `/oak/rgb/image_raw` -> `/oak/rgb/image_rect`."""
    return posixpath.join(posixpath.dirname(image_topic), 'image_rect')


def _launch_nodes(context):
    """Returns the optional rectify and gate nodes plus the detector."""
    # A bool, not text: a 'false' string would stop the node starting.
    use_sim_time = False
    image_topic = LaunchConfiguration('image_topic').perform(context)
    rectify = _as_bool(LaunchConfiguration('rectify').perform(context))
    gated = _as_bool(LaunchConfiguration('gated').perform(context))
    config = get_package_share_directory('perception') + '/config/apriltag.yaml'

    nodes = []
    detector_image = image_topic
    if rectify:
        detector_image = _rectified_topic(image_topic)
        nodes.append(Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('image', image_topic),
                ('image_rect', detector_image),
            ],
        ))
    if gated:
        source = detector_image
        detector_image = posixpath.join(posixpath.dirname(image_topic), 'image_gated')
        nodes.append(Node(
            package='perception',
            executable='image_gate',
            name='image_gate',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'source_topic': source,
                         'output_topic': detector_image}],
        ))
    nodes.append(Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', detector_image),
        ],
    ))
    return nodes


def generate_launch_description():
    """Declares `image_topic`, `rectify` and `gated`, then launches the nodes."""
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument('rectify', default_value='true'),
        DeclareLaunchArgument('gated', default_value='false'),
        OpaqueFunction(function=_launch_nodes),
    ])

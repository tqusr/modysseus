"""Run apriltag_ros's detector against the OAK-D RGB stream.

NOTHING IN THIS STACK CONSUMES THE DETECTIONS. Stated plainly because the
opposite was implied in a couple of places: this node publishes
`/detections` (apriltag_msgs/AprilTagDetectionArray) and apriltag_ros
broadcasts a TF per detected tag, and that is where it ends.

  - Docking is fixed-pose, not vision-corrected:
    perception/config/docking.yaml sets
    `use_external_detection_pose: false`, so docking_server never looks at
    a detected tag (that file's header comment explains the choice and
    what wiring the vision path up would take).
  - Localization is EKF + slam_toolbox only. There is no AprilTag pose
    correction anywhere in this branch -- localization/package.xml used to
    claim otherwise in its <description>; that claim was false and has been
    corrected.
  - mission/config/docks.yaml carries a `tag_id` per dock. It is loaded
    into mission.docks.Dock and never read by any behaviour.

So this node is, today, a verification aid: it is what proved the dock
staging poses put the right tag in the camera's view (see docks.yaml's
empirical-verification note), and it is the hook a future implementer would
start from -- subscribe to `/detections`, filter by the dock's `tag_id`, and
either sanity-check the map pose against the tag's known world pose or
republish a corrected `detected_dock_pose` for SimpleChargingDock. Keeping
it running costs one node; removing it would delete that hook and the
verification path with it. It was never wired up in this branch.

ARGUMENTS. The defaults are the simulator's, so `mapping_mission.launch.py`
includes this file with none and gets exactly the graph it always had.

  use_sim_time        true   applied to every node started here
  image_topic         /oak/rgb/image_raw
  camera_info_topic   /oak/rgb/camera_info
  rectify             false  also start image_proc's rectify_node and detect
                             on its output

RECTIFY, AND WHY ONLY ON HARDWARE. apriltag_ros uses just the camera matrix
from camera_info and ignores the distortion coefficients, so on a lens with
real distortion the tag pose is wrong by an uncorrected amount. The simulated
camera has none, so simulation feeds the raw image straight in
(`rectify:=false`). The real OAK-D RGB camera is distorted, so
`mapping_mission_hardware.launch.py` passes `rectify:=true`: rectify_node
turns `image_topic` into `<same namespace>/image_rect`
(`/oak/rgb/image_rect` by default) and the detector reads that, together with
the unmodified `camera_info_topic`.
"""

import posixpath

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    """Launch arguments are text; node parameters must be typed."""
    return text.strip().lower() in ('true', '1')


def _rectified_topic(image_topic):
    """`/oak/rgb/image_raw` -> `/oak/rgb/image_rect`."""
    return posixpath.join(posixpath.dirname(image_topic), 'image_rect')


def _launch_nodes(context):
    # A bool, not the argument's text: rclpy has already declared
    # `use_sim_time` as a bool, so a 'false' string would stop the node
    # starting (mapping_mission_hardware.launch.py documents the same trap).
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    image_topic = LaunchConfiguration('image_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    rectify = _as_bool(LaunchConfiguration('rectify').perform(context))
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
                ('camera_info', camera_info_topic),
                ('image_rect', detector_image),
            ],
        ))
    nodes.append(Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', detector_image),
            ('camera_info', camera_info_topic),
        ],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/oak/rgb/camera_info'),
        DeclareLaunchArgument('rectify', default_value='false'),
        OpaqueFunction(function=_launch_nodes),
    ])

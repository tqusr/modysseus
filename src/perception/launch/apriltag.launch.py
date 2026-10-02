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
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory('perception') + '/config/apriltag.yaml'
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[config, {'use_sim_time': True}],
            remappings=[
                ('image_rect', '/oak/rgb/image_raw'),
                ('camera_info', '/oak/rgb/camera_info'),
            ],
        ),
    ])

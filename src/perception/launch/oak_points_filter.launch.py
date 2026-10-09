"""Small obstacle cloud from the OAK-D depth image, for Nav2's camera layers.

    /oak/stereo/image_raw --> XYZ cloud --> /oak/points_filtered

navigation's nav2_params.yaml points its camera costmap layers and the
collision_monitor's camera source at /oak/points_filtered.

The camera config (einride-mini-truck's oak_d_lite.yaml) publishes depth unaligned
at 320x240 (76,800 points, 1.2 MB as XYZ), so no resize or voxel step is needed.
Why not the vendor's /oak/points: that is a colour cloud, which needs depth aligned
to colour, and the camera is on a USB 2 link that cannot carry aligned 1280x720
depth (5 Hz, 200 ms). The depth image is 150 kB, so only it crosses DDS (UDP only,
see /etc/default/einride-mini-truck) to this node. Once nothing subscribes to
/oak/points the vendor's colour-cloud node stops computing it.

Empty depth pixels become NaN points; Nav2 drops them at its height checks. The
cloud is in the depth image's frame (oak_right_camera_optical_frame), which Nav2
transforms. There is no crop box: the real camera does not see the robot. Every
node runs on wall-clock time.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _depth_container(config):
    """Returns one container: depth image to XYZ cloud.

    Args:
        config: Path to oak_points_filter.yaml.
    """
    # A bool, not text: rclpy declares `use_sim_time` as a bool.
    use_sim_time = False
    parameters = [config, {'use_sim_time': use_sim_time}]
    return ComposableNodeContainer(
        name='oak_cloud_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='oak_depth_points',
                parameters=parameters,
                remappings=[
                    ('image_rect', '/oak/stereo/image_raw'),
                    ('camera_info', '/oak/stereo/camera_info'),
                    ('points', '/oak/points_filtered'),
                ],
            ),
        ],
    )


def generate_launch_description():
    """Returns the OAK-D point cloud filter container."""
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'oak_points_filter.yaml')
    return LaunchDescription([_depth_container(config)])

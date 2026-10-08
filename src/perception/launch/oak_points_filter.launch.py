"""Small obstacle cloud from the OAK-D depth image, for Nav2's camera layers.

    /oak/stereo/image_raw --> resize 4x per side --> XYZ cloud --> voxel
                          --> /oak/points_filtered

navigation's nav2_params.yaml points its camera costmap layers and the
collision_monitor's camera source at /oak/points_filtered.

Why not filter the vendor's /oak/points: the vendor's camera container builds it
(921,600 points, ~29.5 MB) and it crosses DDS over UDP only
(/etc/default/einride-mini-truck sets FASTDDS_BUILTIN_TRANSPORTS=UDPv4) to reach
a filter, which stalled the collision monitor's camera source and stopped the
robot. Here only the 1.8 MB depth image crosses DDS. It is shrunk 4x per side
(nearest neighbour, so no depth is invented at object edges), turned into a
320x180 XYZ cloud (57,600 points, no colour since nav does not read it) and
voxelled, all in one container with intra-process transport. Once nothing
subscribes to /oak/points the vendor's colour-cloud node stops computing it.
config/oak_points_filter.yaml has the sizing.

There is no crop box: the real camera does not see the robot. Every node runs
on wall-clock time.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _depth_container(config):
    """Returns one container: depth image to small XYZ cloud to voxel.

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
                package='image_proc',
                plugin='image_proc::ResizeNode',
                name='oak_depth_resize',
                parameters=parameters,
                remappings=[
                    ('image/image_raw', '/oak/stereo/image_raw'),
                    ('image/camera_info', '/oak/stereo/camera_info'),
                    ('resize/image_raw', '/oak/depth_small/image_raw'),
                    ('resize/camera_info', '/oak/depth_small/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='oak_depth_points',
                parameters=parameters,
                remappings=[
                    ('image_rect', '/oak/depth_small/image_raw'),
                    ('camera_info', '/oak/depth_small/camera_info'),
                    ('points', '/oak/points_small'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::VoxelGrid',
                name='filter_voxel_grid_node',
                parameters=parameters,
                remappings=[
                    ('input', '/oak/points_small'),
                    ('output', '/oak/points_filtered'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )


def generate_launch_description():
    """Returns the OAK-D point cloud filter container."""
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'oak_points_filter.yaml')
    return LaunchDescription([_depth_container(config)])

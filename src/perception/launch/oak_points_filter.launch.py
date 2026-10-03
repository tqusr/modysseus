"""Crop the OAK-D Lite sim camera's own self-detection noise out of
/oak/points, at the source, before any costmap or safety-monitor ever sees
it.

Why this exists: Task 6's navigation package found (empirically, during its
own NavigateToPose testing) that /oak/points carries self-detection/
near-field noise within a few cm of the robot's own origin. The first fix
attempt raised Nav2's own min-height/min-range thresholds globally in
nav2_params.yaml, which worked but was wrong: it also blinded the camera
obstacle layer to any real low obstacle, or anything within ~20cm,
anywhere in its field of view -- the exact cases that layer exists for.

The real fix belongs here, in `perception` (the design spec's own home for
"camera obstacle cloud relay/filter" responsibility), using an off-the-
shelf pcl_ros filter rather than hand-rolled logic: a CropBox filter
(`config/oak_points_filter.yaml` for the exact box, scoped to the robot's
own known geometry, not a blanket scene-wide cut) removes only points
inside a small region immediately around the robot's own body, republishing
the rest unchanged on /oak/points_filtered. navigation's nav2_params.yaml
points its camera costmap layers and collision_monitor's camera source at
this filtered topic instead of raw /oak/points, with its own height/range
thresholds reverted back down near stock values now that self-returns are
handled here instead.

Not lifecycle nodes (pcl_ros's filter_* executables are plain nodes,
confirmed via its own package contents - no change_state/get_state services
to manage), so these are plain Nodes, no watchdog needed.

A voxel downsample now runs AHEAD of the crop box. That stage is a latency
fix, not a filtering one - the raw cloud's 29.5 MB/message volume was
starving nav2_collision_monitor's camera source of data fresh enough to pass
its `source_timeout`. config/oak_points_filter.yaml has the measurements.

The `use_sim_time` launch argument (default true) exists for the real robot.
The YAML sets `use_sim_time: true` on both filters; the inline dict listed
after it wins.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    """Launch arguments are text; node parameters must be typed."""
    return text.strip().lower() in ('true', '1')


def _launch_nodes(context):
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'oak_points_filter.yaml')

    return [
        # Stage 1: voxel downsample. /oak/points is a 1280x720 ORGANIZED
        # cloud -- 921,600 points, 29.5 MB per message at a nominal 30 Hz
        # (measured off the topic). Carrying that volume all the way to
        # collision_monitor and both costmap layers starved the camera source
        # of fresh data: nav2_collision_monitor was discarding `oak_cloud`
        # against its own `source_timeout: 1.0` several times a second and
        # falling back to lidar-only safety stopping. See
        # config/oak_points_filter.yaml for the full measurement and why a
        # 0.03 m leaf discards nothing any consumer could represent.
        Node(
            package='pcl_ros',
            executable='filter_voxel_grid_node',
            name='filter_voxel_grid_node',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('input', '/oak/points'),
                ('output', '/oak/points_voxel'),
            ],
        ),
        # Stage 2: the self-filter crop box. Reads the downsampled cloud, so
        # its base_footprint TF transform runs over a few thousand points
        # rather than 921,600. Still publishes /oak/points_filtered, which is
        # what navigation/config/nav2_params.yaml points at.
        Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name='filter_crop_box_node',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('input', '/oak/points_voxel'),
                ('output', '/oak/points_filtered'),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_launch_nodes),
    ])

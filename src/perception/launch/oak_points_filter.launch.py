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

Not a lifecycle node (pcl_ros::CropBox / filter_crop_box_node is a plain
node, confirmed via its own package contents - no change_state/get_state
services to manage), so this is a single plain Node, no watchdog needed.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception'), 'config', 'oak_points_filter.yaml')

    return LaunchDescription([
        Node(
            package='pcl_ros',
            executable='filter_crop_box_node',
            name='filter_crop_box_node',
            output='screen',
            parameters=[config],
            remappings=[
                ('input', '/oak/points'),
                ('output', '/oak/points_filtered'),
            ],
        ),
    ])

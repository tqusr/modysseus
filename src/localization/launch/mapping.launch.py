"""Run slam_toolbox in mapping mode against the sim's /scan topic.

One-time data-collection launch: drive the robot around the arena while this
runs, then save the resulting map with nav2_map_server's map_saver_cli. Task
3's localization launch is the long-running counterpart that consumes the
saved map instead of building one.

async_slam_toolbox_node (this ROS 2 Jazzy install) is a managed lifecycle
node: `ros2 node info /slam_toolbox` shows only change_state/get_state
services and no /scan subscription until it is configured and activated. A
plain Node action never drives those transitions, so it would sit idle
forever. Rather than hand-roll the configure/activate event wiring (tried
first - the launch-side EmitEvent/OnStateTransition handler proved flaky:
`change_state` would complete the configure transition but the chained
activate event sometimes never fired, leaving the node stuck "inactive"),
this reuses slam_toolbox's own shipped
`online_async_launch.py` (/opt/ros/jazzy/share/slam_toolbox/launch/), which
contains that exact, tested lifecycle-management logic, and just points its
`slam_params_file` argument at our config instead of the stock one.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    config = get_package_share_directory('localization') + '/config/mapper_params_online_async.yaml'

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': config,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        slam_toolbox,
    ])

"""Wheel-speed drive for the real robot (opt-in).

    ros2 launch localization wheel_drive.launch.py

Starts `wheel_drive_node`, which turns `cmd_vel_ctrl` into `T:1` left/right
wheel speeds. Nav2 must publish to `cmd_vel_ctrl` for it to be driven:
navigation.launch.py `cmd_vel_out:=cmd_vel_ctrl`. Read
`scripts/wheel_drive_node.py` before using it: it is a second writer on the
serial port next to the vendor driver.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('localization')
    return LaunchDescription([
        Node(
            package='localization',
            executable='wheel_drive_node.py',
            name='wheel_drive_node',
            output='screen',
            parameters=[os.path.join(share, 'config', 'hardware.yaml'),
                        {'use_sim_time': False}],
        ),
    ])

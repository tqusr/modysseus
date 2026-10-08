"""Real-robot odometry: wheel odometry plus gyro/EKF fusion.

    ros2 launch localization hardware_odometry.launch.py

Shared by `mapping_mission_hardware.launch.py` and `mission`'s
`hardware.launch.py`, so the chain that owns `odom -> base_footprint` is
defined once. Takes no arguments. See `mapping_mission_hardware.launch.py` for
why the EKF and not the wheels own the TF edge, and for the calibrations in
`config/hardware.yaml`.

The robot's drivers (`/wheel_encoders`, `/imu`) come from the vendor's
`einride-mini-truck` service; nothing here starts them.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Returns wheel odometry plus the gyro/EKF fusion."""
    share = get_package_share_directory('localization')
    hardware_params = os.path.join(share, 'config', 'hardware.yaml')

    # Calibration (wheel_separation, encoder_scale) comes from
    # config/hardware.yaml. TF is off: the EKF below owns odom -> base_footprint.
    wheel_odometry = Node(
        package='localization',
        executable='wheel_odometry_node.py',
        name='wheel_odometry_node',
        output='screen',
        parameters=[hardware_params, {
            'use_sim_time': False,
            'publish_tf': False,
        }],
    )

    # gyro_bias_node and the EKF. Started before SLAM so the TF edge exists
    # at slam_toolbox's first scan lookup.
    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'localization.launch.py')))

    return LaunchDescription([wheel_odometry, fusion])

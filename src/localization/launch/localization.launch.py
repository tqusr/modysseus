"""Wheel + gyro fusion: gyro_bias_node and the robot_localization EKF.

    ros2 launch localization localization.launch.py

What is fused, and why only that, is written down in `config/ekf.yaml`.

Runs on wall-clock time (there is no /clock on the robot) and the EKF
broadcasts odom -> base_footprint: `hardware_odometry.launch.py` turns
wheel_odometry_node's own TF off, since two publishers of one edge fight.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory('localization') + '/config/ekf.yaml'
    return LaunchDescription([
        # Removes the gyro's bias before the EKF integrates it into heading.
        Node(
            package='localization',
            executable='gyro_bias_node.py',
            name='gyro_bias_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config, {'use_sim_time': False, 'publish_tf': True}],
        ),
    ])

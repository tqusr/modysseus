"""Wheel + gyro fusion: gyro_bias_node and the robot_localization EKF.

    ros2 launch localization localization.launch.py                 # simulation
    ros2 launch localization localization.launch.py \\
        use_sim_time:=false publish_tf:=true                        # robot

What is fused, and why only that, is written down in `config/ekf.yaml`.

Defaults are simulation's, so every existing caller (full_stack.launch.py)
keeps behaving as before. Both arguments override ekf.yaml's values: the dict
after the config file wins, because launch applies parameter sources in order.
A bare LaunchConfiguration is parsed by YAML rules, so 'true'/'false' arrive
as bools - see mapping.launch.py's use_sim_time note.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory('localization') + '/config/ekf.yaml'
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Take time from /clock. True in '
                                          'simulation, false on hardware.'),
        DeclareLaunchArgument('publish_tf', default_value='false',
                              description='Broadcast odom -> base_footprint '
                                          'from the EKF. Only when nothing '
                                          'else does: false in simulation '
                                          '(Gazebo owns it), true on the '
                                          'robot with wheel_odometry_node\'s '
                                          'publish_tf off.'),

        # Removes the gyro's bias before the EKF integrates it into heading.
        Node(
            package='localization',
            executable='gyro_bias_node.py',
            name='gyro_bias_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config, {
                'use_sim_time': use_sim_time,
                'publish_tf': LaunchConfiguration('publish_tf'),
            }],
        ),
    ])

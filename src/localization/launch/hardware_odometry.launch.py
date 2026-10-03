"""Real-robot odometry: wheel odometry plus gyro/EKF fusion.

    ros2 launch localization hardware_odometry.launch.py

Shared by `mapping_mission_hardware.launch.py` and (in `mission`)
`full_stack_hardware.launch.py`, so the chain that owns `odom -> base_footprint`
on the robot is defined in exactly one place. It takes no arguments: it is
hardware only. See `mapping_mission_hardware.launch.py` for the full story
(why the EKF and not the wheels own the TF edge, the turn and encoder
calibrations in `config/hardware.yaml`, the two-of-four-wheels limitation).

The robot's own drivers (`/wheel_encoders`, `/imu`) come from the vendor's
`einride-mini-truck` service or `hardware.launch.py`; nothing here starts them.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

#: There is no /clock on a real robot.
#:
#: Spelled twice, because the two places it goes want different TYPES and
#: getting that wrong does not degrade anything - it stops the node starting.
#: A launch ARGUMENT is text, so the include below gets the string; a node
#: PARAMETER is typed, and launch_ros passes a plain Python `str` straight
#: through as a string parameter rather than applying YAML rules to it (only
#: an unannotated *substitution* gets those). rclpy's own TimeSource has
#: already declared `use_sim_time` as a bool by then, so a string override is
#: rejected as a type mismatch and the node dies on construction. The vendor
#: repo's hardware.launch.py makes the same distinction - a `'false'` string
#: in its `launch_arguments`, a `False` bool in every `parameters`.
USE_SIM_TIME = False
USE_SIM_TIME_ARG = 'false'


def generate_launch_description():
    share = get_package_share_directory('localization')
    hardware_params = os.path.join(share, 'config', 'hardware.yaml')

    # The real odometry source, and the reason mapping can work at all.
    # Calibration (wheel_separation, encoder_scale) comes from
    # config/hardware.yaml. See mapping_mission_hardware.launch.py's docstring
    # for the caveats and the two-of-four-wheels limitation, and
    # localization/localization/wheel_odometry.py for the maths and the input
    # contract. Its TF broadcast is OFF: the EKF below owns
    # odom -> base_footprint, and two publishers of one edge fight.
    wheel_odometry = Node(
        package='localization',
        executable='wheel_odometry_node.py',
        name='wheel_odometry_node',
        output='screen',
        parameters=[hardware_params, {
            'use_sim_time': USE_SIM_TIME,
            'publish_tf': False,
        }],
    )

    # gyro_bias_node + EKF, fusing /odom with the bias-corrected gyro and
    # broadcasting odom -> base_footprint. Started before SLAM so the edge is
    # already in TF when slam_toolbox's first scan lookup happens.
    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': USE_SIM_TIME_ARG,
                          'publish_tf': 'true'}.items())

    return LaunchDescription([wheel_odometry, fusion])

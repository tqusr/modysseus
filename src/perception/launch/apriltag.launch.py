from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = get_package_share_directory('perception') + '/config/apriltag.yaml'
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[config, {'use_sim_time': True}],
            remappings=[
                ('image_rect', '/oak/rgb/image_raw'),
                ('camera_info', '/oak/rgb/camera_info'),
            ],
        ),
    ])

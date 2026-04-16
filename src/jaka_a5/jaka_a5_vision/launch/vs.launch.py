from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('jaka_a5_vision')
    apriltag_config = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[apriltag_config],
        output='screen'
    )

    apriltag_sub = Node(
        package='jaka_a5_vision',
        executable='apriltag_subscriber',
        name='apriltag_subscriber',
        output='screen'
    )

    vs_controller = Node(
        package='jaka_a5_vision',
        executable='vs_controller',
        name='visual_servo_controller',
        output='screen'
    )

    return LaunchDescription([apriltag, apriltag_sub, vs_controller])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('jaka_a5_vision')
    apriltag_config = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    enable_servo_controller = LaunchConfiguration('enable_servo_controller')

    camera_qos_relay = Node(
        package='jaka_a5_vision',
        executable='camera_qos_relay',
        name='camera_qos_relay',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[apriltag_config, {'use_sim_time': True}],
        remappings=[
            ('image_rect', '/relay/image_raw'),
            ('camera_info', '/relay/camera_info'),
            ('detections', '/tag_detections'),
        ],
        output='screen'
    )

    apriltag_sub = Node(
        package='jaka_a5_vision',
        executable='apriltag_subscriber',
        name='apriltag_subscriber',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    vs_controller = Node(
        package='jaka_a5_vision',
        executable='vs_controller',
        name='visual_servo_controller',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(enable_servo_controller),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_servo_controller',
            default_value='false',
            description='Start the automatic visual servo controller'
        ),
        camera_qos_relay,
        apriltag,
        apriltag_sub,
        vs_controller,
    ])

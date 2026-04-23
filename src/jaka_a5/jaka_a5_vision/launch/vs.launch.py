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
    enable_moveit_coordinator = LaunchConfiguration('enable_moveit_coordinator')
    debug_pause_on_state_transition = LaunchConfiguration('debug_pause_on_state_transition')
    event_log_path = LaunchConfiguration('event_log_path')
    servo_enable_topic = '/visual_servo/enable'
    debug_step_topic = '/visual_servo/debug_step'

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
        parameters=[{
            'use_sim_time': True,
            'servo_enable_topic': servo_enable_topic,
            'debug_pause_on_state_transition': debug_pause_on_state_transition,
            'debug_step_topic': debug_step_topic,
            'event_log_path': event_log_path,
        }],
        condition=IfCondition(enable_servo_controller),
        output='screen'
    )

    moveit_coordinator = Node(
        package='jaka_a5_vision',
        executable='pregrasp_coordinator',
        name='pregrasp_coordinator',
        parameters=[{'use_sim_time': True, 'servo_enable_topic': servo_enable_topic, 'event_log_path': event_log_path}],
        condition=IfCondition(enable_moveit_coordinator),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_servo_controller',
            default_value='false',
            description='Start the automatic visual servo controller'
        ),
        DeclareLaunchArgument(
            'enable_moveit_coordinator',
            default_value='false',
            description='Use MoveIt to reach pre_grasp before enabling visual servo'
        ),
        DeclareLaunchArgument(
            'debug_pause_on_state_transition',
            default_value='false',
            description='Pause the visual-servo state machine after each state transition until a debug step command arrives'
        ),
        DeclareLaunchArgument(
            'event_log_path',
            default_value='/tmp/jaka_a5_experiment_log.csv',
            description='Shared CSV event log for coordinator and visual servo controller'
        ),
        camera_qos_relay,
        apriltag,
        apriltag_sub,
        vs_controller,
        moveit_coordinator,
    ])

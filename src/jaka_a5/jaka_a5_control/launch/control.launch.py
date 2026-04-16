from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('jaka_a5_control')
    controllers_file = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    # Check file exists before reading
    if not os.path.exists(controllers_file):
        raise FileNotFoundError(f'Controller file not found: {controllers_file}')

    # Load controller yaml
    with open(controllers_file, 'r') as f:
        controllers_yaml = f.read()

    # Controller manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Joint Trajectory Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['jaka_a5_arm_controller'],
        output='screen'
    )

    # Event handler: wait for controller_manager to start before spawning controllers
    joint_state_broadcaster_spawner_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    arm_controller_spawner_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager,
            on_start=[arm_controller_spawner]
        )
    )

    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner_handler,
        arm_controller_spawner_handler
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    gazebo_pkg = FindPackageShare('jaka_a5_gazebo').find('jaka_a5_gazebo')
    control_pkg = FindPackageShare('jaka_a5_control').find('jaka_a5_control')
    planning_pkg = FindPackageShare('jaka_a5_planning').find('jaka_a5_planning')
    vision_pkg = FindPackageShare('jaka_a5_vision').find('jaka_a5_vision')

    enable_vision = LaunchConfiguration('enable_vision')
    enable_servo_controller = LaunchConfiguration('enable_servo_controller')
    enable_moveit_coordinator = LaunchConfiguration('enable_moveit_coordinator')
    enable_camera_monitor = LaunchConfiguration('enable_camera_monitor')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'sim.launch.py')
        )
    )

    control = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(control_pkg, 'launch', 'control.launch.py')
                )
            )
        ]
    )

    moveit = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(planning_pkg, 'launch', 'move_group.launch.py')
                )
            )
        ]
    )

    vision = TimerAction(
        period=8.0,
        condition=IfCondition(enable_vision),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vision_pkg, 'launch', 'vs.launch.py')
                ),
                launch_arguments={
                    'enable_servo_controller': enable_servo_controller,
                    'enable_moveit_coordinator': enable_moveit_coordinator,
                }.items()
            )
        ]
    )

    camera_monitor = TimerAction(
        period=9.0,
        condition=IfCondition(enable_camera_monitor),
        actions=[
            Node(
                package='image_tools',
                executable='showimage',
                name='camera_monitor',
                remappings=[
                    ('image', '/camera/image_raw'),
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_vision',
            default_value='true',
            description='Start AprilTag detection and target pose publisher'
        ),
        DeclareLaunchArgument(
            'enable_servo_controller',
            default_value='false',
            description='Start automatic visual servo controller'
        ),
        DeclareLaunchArgument(
            'enable_moveit_coordinator',
            default_value='false',
            description='Use MoveIt to reach pre_grasp before enabling visual servo'
        ),
        DeclareLaunchArgument(
            'enable_camera_monitor',
            default_value='false',
            description='Open a separate /camera/image_raw window'
        ),
        gazebo,
        control,
        moveit,
        vision,
        camera_monitor,
    ])

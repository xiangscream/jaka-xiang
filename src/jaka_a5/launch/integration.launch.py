from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # Find package shares
    gazebo_pkg = FindPackageShare('jaka_a5_gazebo').find('jaka_a5_gazebo')
    control_pkg = FindPackageShare('jaka_a5_control').find('jaka_a5_control')
    planning_pkg = FindPackageShare('jaka_a5_planning').find('jaka_a5_planning')
    vision_pkg = FindPackageShare('jaka_a5_vision').find('jaka_a5_vision')

    # Integration launch file directory
    launch_dir = os.path.dirname(os.path.abspath(__file__))

    # 1. Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_pkg, 'launch', 'sim.launch.py')
        ])
    )

    # 2. ros2_control (after small delay to let Gazebo initialize)
    control = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(control_pkg, 'launch', 'control.launch.py')
                ])
            )
        ]
    )

    # 3. MoveIt2 (after control)
    moveit = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(planning_pkg, 'launch', 'move_group.launch.py')
                ])
            )
        ]
    )

    # 4. Vision (last - depends on camera topics)
    vision = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(vision_pkg, 'launch', 'vs.launch.py')
                ])
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        control,
        moveit,
        vision
    ])
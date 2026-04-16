from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('jaka_a5_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'jaka_a5.urdf.xacro')
    world_share = get_package_share_directory('jaka_a5_gazebo')
    world_file = os.path.join(world_share, 'worlds', 'battery_station.world')

    # Process xacro to get URDF XML for robot_state_publisher
    result = subprocess.run(
        ['xacro', xacro_file],
        capture_output=True,
        text=True,
        check=True
    )
    robot_desc_xml = result.stdout

    # Start Gazebo with the world file (with --wait for readiness)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--wait', world_file],
        output='screen'
    )

    # Robot state publisher (must start before spawn so TF is available)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc_xml}],
        output='screen'
    )

    # Spawn robot via ros2 service call (delayed to ensure Gazebo is ready)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'URDF=$(xacro {xacro_file}) && '
                    'ros2 service call /spawn_entity ros_gz_sim/srv/SpawnEntity '
                    '"{name: jaka_a5, xml: $URDF}"'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        spawn_robot
    ])

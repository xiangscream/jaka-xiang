from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('jaka_a5_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'jaka_a5.urdf.xacro')
    world_share = get_package_share_directory('jaka_a5_gazebo')
    world_file = os.path.join(world_share, 'worlds', 'battery_station.world')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Process xacro to get URDF XML for robot_state_publisher
    result = subprocess.run(
        ['xacro', xacro_file],
        capture_output=True,
        text=True,
        check=True
    )
    robot_desc_xml = result.stdout

    set_gz_resource_path_description = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.abspath(os.path.join(pkg_share, '..'))
    )

    set_gz_resource_path_worlds = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(world_share, 'models')
    )

    set_gz_resource_path_package = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        world_share
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc_xml},
            {'use_sim_time': True},
        ],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'jaka_a5',
                    '-topic', 'robot_description',
                ],
                output='screen'
            )
        ]
    )

    common_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path_description,
        set_gz_resource_path_worlds,
        set_gz_resource_path_package,
        gazebo,
        robot_state_pub,
        common_bridge,
        camera_image_bridge,
        spawn_robot
    ])

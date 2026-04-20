from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import yaml

def generate_launch_description():
    pkg_share = get_package_share_directory('jaka_a5_planning')
    desc_pkg_share = get_package_share_directory('jaka_a5_description')

    # Load SRDF
    srdf_file = os.path.join(pkg_share, 'config', 'jaka_a5.srdf')
    with open(srdf_file, 'r') as f:
        srdf_content = f.read()

    # Load robot description from xacro
    xacro_file = os.path.join(desc_pkg_share, 'urdf', 'jaka_a5.urdf.xacro')
    try:
        result = subprocess.run(['xacro', xacro_file], capture_output=True, text=True, check=True)
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f'xacro failed: {e.stderr}') from e
    robot_desc_xml = result.stdout

    joint_limits_file = os.path.join(pkg_share, 'config', 'joint_limits.yaml')
    ompl_config_file = os.path.join(pkg_share, 'config', 'ompl_planning.yaml')
    kinematics_file = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    moveit_controllers_file = os.path.join(pkg_share, 'config', 'moveit_controllers.yaml')

    with open(joint_limits_file, 'r') as f:
        joint_limits = yaml.safe_load(f)

    with open(ompl_config_file, 'r') as f:
        ompl_config = yaml.safe_load(f)

    with open(kinematics_file, 'r') as f:
        kinematics = yaml.safe_load(f)

    with open(moveit_controllers_file, 'r') as f:
        moveit_controllers = yaml.safe_load(f)

    move_group_params = [
        {'robot_description': robot_desc_xml},
        {'robot_description_semantic': srdf_content},
        {'robot_description_planning': joint_limits},
        {'robot_description_kinematics': kinematics},
        {'planning_pipelines': ['ompl']},
        {'default_planning_pipeline': 'ompl'},
        ompl_config,
        moveit_controllers,
        {'moveit_manage_controllers': False},
        {'use_sim_time': True},
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
        name='move_group'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'moveit.rviz')],
        output='screen',
        parameters=move_group_params
    )

    return LaunchDescription([
        move_group_node,
        rviz_node
    ])

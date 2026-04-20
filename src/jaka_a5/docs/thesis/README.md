# ROS2 机械臂视觉闭环路径规划 — 毕业设计教程

> 本教程详细记录了从零搭建 JAKA A5 机械臂视觉闭环系统的完整步骤
> 环境：Ubuntu 22.04 + ROS2 Humble + Gazebo Fortress
> GitHub: https://github.com/xiangscream/jaka-xiang

---

## 📋 目录

1. [README — 项目概述与快速导航](./README.md)
2. [step-by-step-tutorial.md — 从零搭建项目的详细步骤](./step-by-step-tutorial.md)
3. [knowledge-modules.md — 各模块知识整理](./knowledge-modules.md)
4. [literature-review.md — 文献综述与技术调研](./literature-review.md)
5. [paper-references.md — 论文推荐清单](./paper-references.md)

---

## 📚 文档分类

### 教程类
| 文档 | 内容 | 适合 |
|------|------|------|
| step-by-step-tutorial.md | 从环境配置到完整系统的10阶段搭建教程 | 跟着做 |
| knowledge-modules.md | ROS2、机械臂、Gazebo、视觉伺服知识整理 | 学习理论 |

### 论文类
| 文档 | 内容 | 适合 |
|------|------|------|
| literature-review.md | 视觉伺服、Eye-in-Hand、AprilTag、深度学习综述 | 写文献综述 |
| paper-references.md | 20+经典论文推荐清单，含引用格式 | 写参考文献 |

---

## 1. 项目概述

### 1.1 项目目标

实现一个**换电池工作站**中的机械臂视觉闭环路径规划系统：

```
相机采集图像 → AprilTag检测 → 视觉伺服控制 → 机械臂运动 → 循环反馈
```

### 1.2 技术栈

| 组件 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 22.04 | 操作系统 |
| ROS2 | Humble | 机器人操作系统 |
| Gazebo | Fortress | 仿真引擎 |
| MoveIt2 | Humble | 运动规划 |
| apriltag_ros | - | AprilTag 检测 |

### 1.3 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Gazebo Sim (Ignition)                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐  │
│  │ jaka_a5     │    │ 电池+标签    │    │ camera_link (Eye-in-Hand)│  │
│  │ 机械臂模型   │    │ AprilTag    │    │ 30Hz 图像发布            │  │
│  └─────────────┘    └─────────────┘    └───────────┬─────────────┘  │
└────────────────────────────┬───────────────────────┼────────────────┘
                             │                       │
                    ┌────────┴────────┐    ┌────────┴────────┐
                    │ ros_gz_bridge   │    │ ros_gz_image    │
                    │ /clock 等话题   │    │ /camera/image   │
                    └────────┬────────┘    └────────┬────────┘
                             │                       │
                             ▼                       ▼
                    ┌─────────────────────────────────────────┐
                    │        ROS2 话题 (/camera/image_raw)     │
                    └────────────────────┬────────────────────┘
                                         │
                    ┌────────────────────┴────────────────────┐
                    │              apriltag_ros                │
                    │     /tag_detections (3D位姿)            │
                    └────────────────────┬────────────────────┘
                                         │
                    ┌────────────────────┴────────────────────┐
                    │         apriltag_subscriber              │
                    │    提取 tag_id=1 → /target_pose        │
                    └────────────────────┬────────────────────┘
                                         │
                    ┌────────────────────┴────────────────────┐
                    │           vs_controller                  │
                    │   计算误差 → /jaka_a5_arm_controller/    │
                    │           joint_trajectory              │
                    └────────────────────┬────────────────────┘
                                         │
                    ┌────────────────────┴────────────────────┐
                    │        JointTrajectoryController          │
                    │    (ros2_control + gz_ros2_control)       │
                    └─────────────────────────────────────────┘
```

---

## 2. 环境配置

### 2.1 安装 ROS2 Humble

```bash
# 设置 locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 apt 源
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-moveit2 ros-humble-gazebo-ros-pkgs

# 安装 Gazebo Fortress
sudo apt install gz-fortress

# 安装其他依赖
sudo apt install python3-colcon-common-extensions python3-vcstool python3-colcon-ros python3-rosdep

# 初始化 rosdep
sudo rosdep init
rosdep update

# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 2.2 验证安装

```bash
# 检查 ROS2
ros2 doctor

# 检查 Gazebo
gz sim --version

# 检查 colcon
colcon version
```

---

## 3. 创建工作空间和包结构

### 3.1 创建工作空间目录

```bash
cd ~/ros2_ws/src
mkdir -p jaka_a5/{jaka_a5_description,jaka_a5_gazebo,jaka_a5_control,jaka_a5_planning,jaka_a5_vision,jaka_a5_bringup}
```

### 3.2 创建 jaka_a5_description 包

机器人描述包，包含 URDF/xacro 文件。

```bash
cd ~/ros2_ws/src/jaka_a5/jaka_a5_description
mkdir -p {urdf,launch,meshes,rviz}

# 创建 package.xml
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>jaka_a5_description</name>
  <version>1.0.0</version>
  <description>JAKA A5 robot description package</description>
  <maintainer email="xiang@example.com">Xiang</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>robot_state_publisher</depend>
  <depend>xacro</depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# 创建 CMakeLists.txt
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(jaka_a5_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY urdf launch meshes rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
EOF
```

### 3.3 创建其他包（概要）

```
jaka_a5_description/   — URDF/xacro 机器人模型
jaka_a5_gazebo/        — Gazebo 仿真环境
jaka_a5_control/       — ros2_control 控制器
jaka_a5_planning/       — MoveIt2 运动规划
jaka_a5_vision/         — AprilTag 视觉检测
jaka_a5_bringup/        — 统一启动入口
```

每个包的结构：

```bash
# 目录结构示例
jaka_a5_xxx/
├── package.xml
├── CMakeLists.txt (ament_cmake) 或 setup.py (ament_python)
├── launch/
├── config/
└── src/ (C++) 或 jaka_a5_xxx/ (Python)
```

---

## 4. 机器人描述（URDF/xacro）

### 4.1 创建基础 URDF 文件

`jaka_a5_description/urdf/jaka_a5.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="jaka_a5">

  <!-- 包含机器人模型 + Gazebo 配置 -->
  <xacro:include filename="jaka_a5.robot.xacro" />
  <xacro:include filename="gazebo.urdf.xacro" />

</robot>
```

### 4.2 创建机器人模型 xacro

`jaka_a5_description/urdf/jaka_a5.robot.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="jaka_a5">

  <!-- 常量定义 -->
  <xacro:property name="PI" value="3.1415926535897931" />
  <xacro:property name="VELOCITY" value="3.665" />

  <!-- world link（Gazebo 需要） -->
  <link name="world" />

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <origin xyz="0.0028167 -0.00019454 0.019544" rpy="0 0 0" />
      <mass value="0.52663" />
      <inertia ixx="0.0006297" ixy="6.7091E-08" ixz="3.4682E-06"
               iyy="0.00095727" iyz="2.4476E-07" izz="0.0014514" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://jaka_a5_description/meshes/jaka_a5_meshes/base_link.STL" />
      </geometry>
      <material name="">
        <color rgba="0.86667 0.86667 0.8902 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://jaka_a5_description/meshes/jaka_a5_meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>

  <!-- World to Base Joint -->
  <joint name="fixed" type="fixed">
    <parent link="world" />
    <child link="base_link" />
  </joint>

  <!-- Joint 1 -->
  <link name="J1">
    <inertial>
      <origin xyz="-2.5186E-07 0.0033226 -0.001509" rpy="0 0 0" />
      <mass value="15.135" />
      <inertia ixx="0.044302" ixy="1.5349E-07" ixz="-6.1966E-07"
               iyy="0.043091" iyz="1.4326E-05" izz="0.030523" />
    </inertial>
    <visual>...</visual>
    <collision>...</collision>
  </link>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 -0.00022535 0.12015" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="J1" />
    <axis xyz="0 0 1" />
    <limit lower="-6.28" upper="6.28" effort="2000" velocity="3.665" />
  </joint>

  <!-- Joint 2 ~ Joint 6 结构类似 -->

  <!-- Eye-in-Hand Camera Mount -->
  <link name="camera_link" />

  <joint name="camera_joint" type="fixed">
    <origin xyz="0 0 0.02" rpy="0 0 0" />
    <parent link="J6" />
    <child link="camera_link" />
  </joint>

  <!-- ros2_control 硬件接口声明 -->
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="joint_1">
      <command_interface name="position" />
      <state_interface name="position" />
      <state_interface name="velocity" />
    </joint>
    <!-- joint_2 ~ joint_6 同样结构 -->
  </ros2_control>

</robot>
```

### 4.3 关键概念解释

| 元素 | 说明 |
|------|------|
| `<link>` | 机械臂的连杆，包含惯性、外观、碰撞体 |
| `<joint>` | 关节，定义两个 link 之间的运动关系 |
| `<sensor>` | 传感器（如相机），挂在 link 上 |
| `<gazebo>` | Gazebo 特定配置（Gazebo 扩展） |
| `<ros2_control>` | ros2_control 硬件接口声明 |

### 4.4 相机挂载点说明

相机安装在 J6（末端法兰）上：

```xml
<origin xyz="0 0 0.02" rpy="0 0 0" />
```

| 参数 | 值 | 含义 |
|------|-----|------|
| xyz | 0 0 0.02 | 相机在 J6 法兰上方 2cm |
| rpy | 0 0 0 | 相机朝向与法兰一致（垂直向上） |

---

## 5. Gazebo 仿真环境

### 5.1 创建 Gazebo world 文件

`jaka_a5_gazebo/worlds/battery_station.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="battery_station">

    <!-- 物理引擎 -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- 光照 -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
    </light>

    <!-- 地面 -->
    <ground_plane>
      <geometry>
        <plane><normal>0 0 1</normal><size>10 10</size></plane>
      </geometry>
      <material>
        <diffuse>0.5 0.5 0.5 1</diffuse>
      </material>
    </ground_plane>

    <!-- 工作台 -->
    <model name="workbench">
      <pose>0 0 0.8 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual>
          <geometry><box><size>1.2 0.8 0.05</size></box></geometry>
          <material><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
        <collision>
          <geometry><box><size>1.2 0.8 0.05</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- 电池模型 + AprilTag -->
    <model name="battery_with_tag">
      <pose>0.5 0 0.825 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="battery_visual">
          <pose>0 0 0 0 0 0</pose>
          <geometry><box><size>0.1 0.05 0.025</size></box></geometry>
          <material><diffuse>1 1 1 1</diffuse></material>
        </visual>
        <visual name="tag_visual">
          <pose>0 0 0.011 0 0 0</pose>
          <geometry><box><size>0.05 0.05 0.001</size></box></geometry>
          <material><diffuse>1 1 1 1</diffuse></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### 5.2 创建 Gazebo xacro 配置

`jaka_a5_description/urdf/gazebo.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Gazebo 材质 -->
  <gazebo reference="base_link"><material>Gazebo/DarkGrey</material></gazebo>
  <gazebo reference="J1"><material>Gazebo/Blue</material></gazebo>
  <!-- ... 其他连杆材质 ... -->

  <!-- ros2_control 插件（Gazebo Sim） -->
  <gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find jaka_a5_control)/config/ros2_controllers.yaml</parameters>
      <ros>
        <remapping>/controller_manager/robot_description:=/robot_description</remapping>
      </ros>
    </plugin>
  </gazebo>

  <!-- Eye-in-Hand 相机传感器 -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30.0</update_rate>
      <topic>/camera/image_raw</topic>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>  <!-- 60度 -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.01</near>
          <far>100</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>

</robot>
```

### 5.3 创建 sim.launch.py

`jaka_a5_gazebo/launch/sim.launch.py`:

```python
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

    # 处理 xacro 生成 URDF
    result = subprocess.run(['xacro', xacro_file], capture_output=True, text=True, check=True)
    robot_desc_xml = result.stdout

    # 设置 Gazebo 资源路径
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.abspath(os.path.join(pkg_share, '..'))
    )

    # 启动 Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # Robot State Publisher（发布 TF）
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc_xml}, {'use_sim_time': True}],
        output='screen'
    )

    # 延迟 3 秒后 spawn 机器人
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'jaka_a5', '-topic', 'robot_description'],
                output='screen'
            )
        ]
    )

    # /clock 时钟桥接
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 相机图像桥接（Gazebo → ROS2）
    camera_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen'
    )

    # 相机内参桥接
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_pub,
        clock_bridge,
        camera_image_bridge,
        camera_info_bridge,
        spawn_robot
    ])
```

### 5.4 桥接话题说明

| Gazebo 话题 | ROS2 话题 | 消息类型 |
|------------|----------|----------|
| `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` |
| `/camera/image_raw` | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

---

## 6. ros2_control 控制器配置

### 6.1 创建 ros2_controllers.yaml

`jaka_a5_control/config/ros2_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    jaka_a5_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

jaka_a5_arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      joint_1: {trajectory: 0.2, goal: 0.1}
      joint_2: {trajectory: 0.2, goal: 0.1}
      joint_3: {trajectory: 0.2, goal: 0.1}
      joint_4: {trajectory: 0.2, goal: 0.1}
      joint_5: {trajectory: 0.2, goal: 0.1}
      joint_6: {trajectory: 0.2, goal: 0.1}
```

### 6.2 创建 control.launch.py

`jaka_a5_control/launch/control.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    control_pkg = get_package_share_directory('jaka_a5_control')
    controllers_file = os.path.join(control_pkg, 'config', 'ros2_controllers.yaml')

    # controller_manager 节点
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        output='screen'
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen'
    )

    # Arm Controller spawner（在 joint_state_broadcaster 之后启动）
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['jaka_a5_arm_controller', '--controller-manager-timeout', '30'],
        output='screen'
    )

    # 等 joint_state_broadcaster 完全退出后再启动 arm_controller
    arm_controller_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]
        )
    )

    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_after_joint_state
    ])
```

### 6.3 ros2_control 架构说明

```
Gazebo Sim
    ↓ gz_ros2_control 系统插件
controller_manager
    ├── joint_state_broadcaster (发布 /joint_states)
    └── jaka_a5_arm_controller (接收 /jaka_a5_arm_controller/joint_trajectory)
```

---

## 7. MoveIt2 运动规划

### 7.1 创建 SRDF 文件

`jaka_a5_planning/config/jaka_a5.srdf`:

```xml
<?xml version="1.0"?>
<robot name="jaka_a5">

  <!-- 规划组定义 -->
  <group name="manipulator">
    <joint name="joint_1" />
    <joint name="joint_2" />
    <joint name="joint_3" />
    <joint name="joint_4" />
    <joint name="joint_5" />
    <joint name="joint_6" />
  </group>

  <!-- 预定义姿态 -->
  <group_state name="home" group="manipulator">
    <joint name="joint_1" value="0" />
    <joint name="joint_2" value="1.5707" />
    <joint name="joint_3" value="-1.5707" />
    <joint name="joint_4" value="1.5707" />
    <joint name="joint_5" value="1.5707" />
    <joint name="joint_6" value="0" />
  </group_state>

  <group_state name="zero" group="manipulator">
    <joint name="joint_1" value="0" />
    <joint name="joint_2" value="0" />
    <joint name="joint_3" value="0" />
    <joint name="joint_4" value="0" />
    <joint name="joint_5" value="0" />
    <joint name="joint_6" value="0" />
  </group_state>

  <!-- 碰撞检测禁用（相邻连杆） -->
  <disable_collisions link1="base_link" link2="J1" reason="Adjacent" />
  <disable_collisions link1="J1" link2="J2" reason="Adjacent" />
  <!-- ... 其他相邻连杆 ... -->

</robot>
```

### 7.2 创建 kinematics.yaml

`jaka_a5_planning/config/kinematics.yaml`:

```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

### 7.3 创建 ompl_planning.yaml

`jaka_a5_planning/config/ompl_planning.yaml`:

```yaml
ompl:
  planning_plugin: ompl_interface/OMPLPlanner
  request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/FixWorkspaceBounds
    default_planner_request_adapters/FixStartStateBounds
    default_planner_request_adapters/FixStartStateCollision
    default_planner_request_adapters/FixStartStatePathConstraints
  start_state_max_bounds_error: 0.1
  planner_configs:
    RRTConnect:
      type: geometric::RRTConnect
      range: 0.0
  manipulator:
    planner_configs:
      - RRTConnect
```

### 7.4 创建 move_group.launch.py

`jaka_a5_planning/launch/move_group.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    planning_pkg = get_package_share_directory('jaka_a5_planning')
    srdf_file = os.path.join(planning_pkg, 'config', 'jaka_a5.srdf')
    kinematics_file = os.path.join(planning_pkg, 'config', 'kinematics.yaml')
    ompl_file = os.path.join(planning_pkg, 'config', 'ompl_planning.yaml')

    # 读取配置文件
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()
    with open(kinematics_file, 'r') as f:
        kinematics = yaml.safe_load(f)
    with open(ompl_file, 'r') as f:
        ompl_config = yaml.safe_load(f)

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            {'robot_description_semantic': robot_description_semantic},
            {'robot_description_kinematics': kinematics},
            ompl_config,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([move_group])
```

---

## 8. Eye-in-Hand 相机配置

### 8.1 URDF 中的相机定义

在 `jaka_a5.robot.xacro` 中添加：

```xml
<!-- Eye-in-Hand Camera Mount（空 link 作为挂载点） -->
<link name="camera_link" />

<joint name="camera_joint" type="fixed">
  <origin xyz="0 0 0.02" rpy="0 0 0" />
  <parent link="J6" />
  <child link="camera_link" />
</joint>
```

### 8.2 Gazebo 中的相机传感器

在 `gazebo.urdf.xacro` 中添加：

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30.0</update_rate>
    <topic>/camera/image_raw</topic>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60度 -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.01</near>
        <far>100</far>
      </clip>
      <distortion>
        <k1>0.0</k1><k2>0.0</k2><p1>0.0</p1><p2>0.0</p2><kc>0.0</kc>
      </distortion>
    </camera>
  </sensor>
</gazebo>
```

### 8.3 相机参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| `update_rate` | 30.0 Hz | 每秒发布 30 帧 |
| `horizontal_fov` | 1.047 rad ≈ 60° | 水平视场角 |
| `image.width/height` | 640×480 | 分辨率 |
| `clip near/far` | 0.01/100 m | 可见距离范围 |

### 8.4 调整相机视角

如果想让相机俯视工作台，修改 `rpy`：

```xml
<!-- 相机向下倾斜 45 度 -->
<origin xyz="0 0 0.02" rpy="-0.785 0 0" />

<!-- 相机向下倾斜 45 度并往外偏移 5cm -->
<origin xyz="0 0.05 0.02" rpy="-0.785 0 0" />
```

---

## 9. AprilTag 视觉检测

### 9.1 安装 apriltag_ros

```bash
sudo apt install ros-humble-apriltag-ros
```

### 9.2 创建 apriltag.yaml 配置文件

`jaka_a5_vision/config/apriltag.yaml`:

```yaml
apriltag:
  ros__parameters:
    tag_family: 36h11        # AprilTag 家族
    tag_ids: [1]             # 目标 tag ID
    tag_size: 0.05           # tag 实际尺寸（米）
    publish_tf: true         # 发布 TF 变换
```

### 9.3 创建 vs.launch.py

`jaka_a5_vision/launch/vs.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        parameters=[{
            'tag_family': '36h11',
            'tag_ids': [1],
            'tag_size': 0.05,
            'publish_tf': True
        }],
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    apriltag_subscriber = Node(
        package='jaka_a5_vision',
        executable='apriltag_subscriber',
        name='apriltag_subscriber',
        output='screen'
    )

    vs_controller = Node(
        package='jaka_a5_vision',
        executable='vs_controller',
        name='visual_servo_controller',
        parameters=[{
            'position_threshold': 0.005,
            'max_step': 0.02,
            'desired_z': 0.1
        }],
        output='screen'
    )

    return LaunchDescription([
        apriltag_node,
        apriltag_subscriber,
        vs_controller
    ])
```

---

## 10. 视觉伺服控制器

### 10.1 apriltag_subscriber.py

`jaka_a5_vision/jaka_a5_vision/apriltag_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

class AprilTagSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_subscriber')
        self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.target_tag_id = 1

    def tag_callback(self, msg):
        for detection in msg.detections:
            # 处理 tag_id（兼容列表和单个值）
            detection_ids = detection.id if hasattr(detection.id, '__iter__') else [detection.id]

            if detection_ids and detection_ids[0] == self.target_tag_id:
                if hasattr(detection.pose, 'header'):
                    pose_msg = detection.pose
                    pose = PoseStamped()
                    pose.header = pose_msg.header
                    pose.pose = pose_msg.pose.pose
                else:
                    pose = PoseStamped()
                    pose.header = detection.header
                    pose.pose = detection.pose.pose

                self.publisher.publish(pose)
                self.get_logger().info(
                    f'Tag {self.target_tag_id} detected at '
                    f'x={pose.pose.position.x:.3f} '
                    f'y={pose.pose.position.y:.3f} '
                    f'z={pose.pose.position.z:.3f}'
                )

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 10.2 vs_controller.py

`jaka_a5_vision/jaka_a5_vision/vs_controller.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class VisualServoController(Node):
    def __init__(self):
        super().__init__('visual_servo_controller')

        # 订阅目标位姿和关节状态
        self.create_subscription(PoseStamped, '/target_pose', self.target_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # 发布关节轨迹
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/jaka_a5_arm_controller/joint_trajectory', 10
        )

        # 参数
        self.declare_parameter('position_threshold', 0.005)  # 5mm
        self.declare_parameter('max_step', 0.02)             # 20mm per step
        self.declare_parameter('desired_z', 0.1)             # 目标高度 100mm

        self.target_pose = None
        self.current_joint_positions = [0.0] * 6
        self.joint_name_to_idx = {
            'joint_1': 0, 'joint_2': 1, 'joint_3': 2,
            'joint_4': 3, 'joint_5': 4, 'joint_6': 5
        }

    def joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_name_to_idx:
                idx = self.joint_name_to_idx[name]
                self.current_joint_positions[idx] = msg.position[i]

    def target_callback(self, msg):
        self.target_pose = msg
        error = np.sqrt(
            msg.pose.position.x**2 +
            msg.pose.position.y**2 +
            (msg.pose.position.z - self.get_parameter('desired_z').value)**2
        )

        if error < self.get_parameter('position_threshold').value:
            self.get_logger().info('Target reached! Stopping.')
            self.send_stop()
        else:
            self.send_servo_command()

    def compute_delta(self):
        if self.target_pose is None:
            return [0.0] * 6

        x = self.target_pose.pose.position.x
        y = self.target_pose.pose.position.y
        z = self.target_pose.pose.position.z
        scale = 0.5

        delta = [scale * x, scale * y, scale * (z - self.get_parameter('desired_z').value), 0.0, 0.0, 0.0]

        # 限制最大步长
        max_step = self.get_parameter('max_step').value
        for i in range(3):
            if abs(delta[i]) > max_step:
                delta[i] = np.sign(delta[i]) * max_step

        return delta

    def send_servo_command(self):
        delta = self.compute_delta()
        new_joints = [curr + d for curr, d in zip(self.current_joint_positions, delta)]

        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        point = JointTrajectoryPoint()
        point.positions = new_joints
        point.time_from_start.sec = 1
        traj.points = [point]
        self.trajectory_pub.publish(traj)

    def send_stop(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        point = JointTrajectoryPoint()
        point.positions = self.current_joint_positions
        point.time_from_start.sec = 0
        traj.points = [point]
        self.trajectory_pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    controller = VisualServoController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

---

## 11. 系统集成与启动

### 11.1 创建 integration.launch.py

`jaka_a5_bringup/launch/integration.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    gazebo_pkg = FindPackageShare('jaka_a5_gazebo').find('jaka_a5_gazebo')
    control_pkg = FindPackageShare('jaka_a5_control').find('jaka_a5_control')
    planning_pkg = FindPackageShare('jaka_a5_planning').find('jaka_a5_planning')
    vision_pkg = FindPackageShare('jaka_a5_vision').find('jaka_a5_vision')

    # 0s: 启动 Gazebo 仿真
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'sim.launch.py'))
    )

    # 4s: 启动控制器
    control = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(control_pkg, 'launch', 'control.launch.py'))
            )
        ]
    )

    # 6s: 启动 MoveIt2
    moveit = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(planning_pkg, 'launch', 'move_group.launch.py'))
            )
        ]
    )

    # 8s: 启动视觉节点
    vision = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(vision_pkg, 'launch', 'vs.launch.py'))
            )
        ]
    )

    return LaunchDescription([gazebo, control, moveit, vision])
```

### 11.2 启动命令

```bash
# 1. 设置环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. 构建工作空间
cd ~/ros2_ws
colcon build

# 3. 启动完整系统
ros2 launch jaka_a5_bringup integration.launch.py
```

### 11.3 验证命令

```bash
# 查看活跃的 controller
ros2 control list_controllers

# 查看相机图像话题
ros2 topic echo /camera/image_raw --once

# 查看 AprilTag 检测结果
ros2 topic echo /tag_detections --once

# 查看 target_pose
ros2 topic echo /target_pose --once

# 查看关节状态
ros2 topic echo /joint_states --once

# 使用 rqt_image_view 查看相机图像
rqt_image_view /camera/image_raw
```

---

## 12. 常见问题排查

### 问题 1: robot_state_publisher aborts

**错误信息**:
```
[robot_state_publisher]: Could not find parameter 'robot_description'
[robot_state_publisher]: Aborting
```

**原因**: URDF 中引用了 `world` link 但未声明。

**解决**: 在 URDF 中添加：
```xml
<link name="world" />
```

### 问题 2: 控制器激活失败

**错误信息**:
```
[jaka_a5_arm_controller]: Not found
```

**解决**:
1. 检查 `ros2_controllers.yaml` 配置
2. 确认 `gz_ros2_control` 插件加载成功
3. 查看 `ros2 control list_controllers` 状态

### 问题 3: 相机图像话题无数据

**排查步骤**:
1. 确认 Gazebo 中相机 sensor 已激活
2. 检查 `ros_gz_bridge` 桥接配置
3. 使用 `ros2 topic hz /camera/image_raw` 查看话题频率

### 问题 4: AprilTag 检测不到

**排查步骤**:
1. 确认 AprilTag 标签尺寸与配置一致
2. 检查相机内参是否正确
3. 确认标签在相机视野内

### 问题 5: MoveIt 无法执行轨迹

**原因**: 缺少 `moveit_controllers.yaml` 配置。

**解决**: 创建 `moveit_controllers.yaml`:
```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - jaka_a5_arm_controller

  jaka_a5_arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
```

---

## 附录 A: 完整文件树

```
jaka_a5/
├── jaka_a5_description/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── urdf/
│   │   ├── jaka_a5.urdf.xacro
│   │   ├── jaka_a5.robot.xacro
│   │   └── gazebo.urdf.xacro
│   ├── launch/
│   │   └── view_robot.launch.py
│   └── meshes/
├── jaka_a5_gazebo/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── sim.launch.py
│   └── worlds/
│       └── battery_station.world
├── jaka_a5_control/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── control.launch.py
│   └── config/
│       └── ros2_controllers.yaml
├── jaka_a5_planning/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── move_group.launch.py
│   └── config/
│       ├── jaka_a5.srdf
│       ├── kinematics.yaml
│       └── ompl_planning.yaml
├── jaka_a5_vision/
│   ├── package.xml
│   ├── setup.py
│   ├── launch/
│   │   └── vs.launch.py
│   ├── config/
│   │   └── apriltag.yaml
│   └── jaka_a5_vision/
│       ├── __init__.py
│       ├── apriltag_subscriber.py
│       └── vs_controller.py
└── jaka_a5_bringup/
    ├── CMakeLists.txt
    ├── package.xml
    └── launch/
        └── integration.launch.py
```

## 附录 B: 参考资源

| 资源 | 链接 |
|------|------|
| ROS2 Humble 文档 | https://docs.ros.org/en/humble/ |
| MoveIt2 文档 | https://moveit.ros.org/ |
| Gazebo Fortress | https://gazebosim.org/docs/fortress |
| apriltag_ros | https://github.com/christianrauch/apriltag_ros |
| jaka-xiang (GitHub) | https://github.com/xiangscream/jaka-xiang |

---

*本文档由 Claude (蒠芷) 生成，基于毕业设计项目整理*
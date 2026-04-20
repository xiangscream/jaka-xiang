# ROS2 机械臂视觉闭环 — 一步步搭建教程

> 本文档详细记录从配置官方工作空间到完整运行项目的每一步
> 环境：Ubuntu 22.04 + ROS2 Humble + Gazebo Fortress
> **更新日期**: 2026-04-20

---

## 🚀 快速导航

| 阶段 | 步骤 | 主要内容 | 预计时间 |
|------|------|----------|----------|
| 第一阶段 | Step 1.1-1.3 | 环境配置 | 30分钟 |
| 第二阶段 | Step 2.1-2.3 | 创建包结构 | 15分钟 |
| 第三阶段 | Step 3.1-3.4 | URDF/xacro文件 | 30分钟 |
| 第四阶段 | Step 4.1-4.3 | Gazebo仿真 | 20分钟 |
| 第五阶段 | Step 5.1-5.3 | ros2_control | 20分钟 |
| 第六阶段 | Step 6.1-6.5 | MoveIt2 | 25分钟 |
| 第七阶段 | Step 7.1-7.6 | 视觉节点 | 30分钟 |
| 第八阶段 | Step 8.1-8.2 | 集成启动 | 15分钟 |
| 第九阶段 | Step 9.1-9.4 | 构建验证 | 20分钟 |

**总预计时间**: ~3.5小时（不含下载STL网格文件时间）

---

## ✅ 每步验证方法说明

在继续之前，了解每个步骤的**验证方法**非常重要：

```bash
# 验证模式：每个阶段完成后运行以下检查

# 1. xacro 解析验证
xacro path/to/file.urdf.xacro  # 无错误即为成功

# 2. 包发现验证
colcon list  # 应该能看到所有包

# 3. 构建验证
colcon build  # 应该成功，无 error

# 4. 运行时验证
ros2 launch pkg_name launch_name.launch.py  # 应该启动无崩溃
```

---

## 第一阶段：环境配置

### Step 1.1: 安装 Ubuntu 22.04

下载 Ubuntu 22.04 LTS 镜像并安装。

### Step 1.2: 安装 ROS2 Humble

> **预估时间**: 15-20分钟（取决于网络速度）
> **验证方法**: 步骤完成后运行 `ros2 doctor` 检查

```bash
# 1. 设置 locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加 ROS2 apt 源
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS2 Humble Desktop（包含 ros2_control, moveit2, gazebo）
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-moveit2 ros-humble-gazebo-ros-pkgs

# 4. 安装 Gazebo Fortress
sudo apt install gz-fortress

# 5. 安装 colcon 和 rosdep
sudo apt install python3-colcon-common-extensions python3-vcstool python3-colcon-ros python3-rosdep

# 6. 初始化 rosdep
sudo rosdep init
rosdep update

# 7. 验证安装 ✅
ros2 doctor

# 期望输出示例:
# [INFO] ros2 doctor version 3.1.0
# [INFO] checking ROS distribution [humble]
# [INFO] found ROS distribution file in /opt/ros/humble
# [INFO] ... (应该没有 ERROR)
```

> **常见问题**:
> - 如果 `ros2 doctor` 显示 ERROR，检查网络是否正常
> - 如果 Gazebo 安装失败，可能需要添加 osrfoundation 源

### Step 1.3: 创建工作空间

> **验证方法**: 运行 `colcon version` 应显示版本信息

```bash
# 创建工作空间目录
mkdir -p ~/ros2_ws/src

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 验证 colcon
colcon version

# （可选）将环境加载命令添加到 .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 第二阶段：创建包结构

### Step 2.1: 创建包目录结构

> **验证方法**: 创建完成后运行 `ls -R ~/ros2_ws/src/jaka_a5/` 应显示所有目录

```bash
cd ~/ros2_ws/src

# 创建所有包的目录
mkdir -p jaka_a5/jaka_a5_description/urdf
mkdir -p jaka_a5/jaka_a5_description/launch
mkdir -p jaka_a5/jaka_a5_description/meshes
mkdir -p jaka_a5/jaka_a5_description/rviz

mkdir -p jaka_a5/jaka_a5_gazebo/launch
mkdir -p jaka_a5/jaka_a5_gazebo/worlds
mkdir -p jaka_a5/jaka_a5_gazebo/models

mkdir -p jaka_a5/jaka_a5_control/launch
mkdir -p jaka_a5/jaka_a5_control/config

mkdir -p jaka_a5/jaka_a5_planning/launch
mkdir -p jaka_a5/jaka_a5_planning/config

mkdir -p jaka_a5/jaka_a5_vision/launch
mkdir -p jaka_a5/jaka_a5_vision/config
mkdir -p jaka_a5/jaka_a5_vision/jaka_a5_vision

mkdir -p jaka_a5/jaka_a5_bringup/launch

mkdir -p jaka_a5/docs/thesis

# 验证目录结构 ✅
tree ~/ros2_ws/src/jaka_a5  # 如果没有tree，用 ls -R
```

### Step 2.2: 创建 jaka_a5_description 包

> **关键概念**: `package.xml` 定义了包的元信息（名称、版本、依赖），CMakeLists.txt 定义了构建规则

```bash
cd ~/ros2_ws/src/jaka_a5/jaka_a5_description

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
  <build_depend>robot_state_publisher</build_depend>
  <build_depend>xacro</build_depend>
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

### Step 2.3: 创建其他包的 package.xml

> **概念**: 每个包都是独立的模块，有自己的 `package.xml` 和构建文件
> **验证方法**: 创建完所有包后，运行 `colcon list` 应能看到所有包

**jaka_a5_control/package.xml:**
```bash
cat > ~/ros2_ws/src/jaka_a5/jaka_a5_control/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>jaka_a5_control</name>
  <version>1.0.0</version>
  <description>JAKA A5 control packages</description>
  <maintainer email="xiang@example.com">Xiang</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>controller_manager</depend>
  <depend>ros2_controllers</depend>
  <depend>gazebo_ros2_control</depend>
  <depend>joint_state_broadcaster</depend>
  <depend>joint_trajectory_controller</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
```

**jaka_a5_gazebo/package.xml:**
```bash
cat > ~/ros2_ws/src/jaka_a5/jaka_a5_gazebo/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>jaka_a5_gazebo</name>
  <version>1.0.0</version>
  <description>JAKA A5 Gazebo simulation</description>
  <maintainer email="xiang@example.com">Xiang</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>ros_gz_sim</depend>
  <depend>ros_gz_bridge</depend>
  <depend>ros_gz_image</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
```

**jaka_a5_planning/package.xml:**
```bash
cat > ~/ros2_ws/src/jaka_a5/jaka_a5_planning/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>jaka_a5_planning</name>
  <version>1.0.0</version>
  <description>JAKA A5 MoveIt2 planning</description>
  <maintainer email="xiang@example.com">Xiang</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>moveit_ros_move_group</depend>
  <depend>robot_state_publisher</depend>
  <exec_depend>rviz2</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
```

**jaka_a5_vision/package.xml:**
```bash
cat > ~/ros2_ws/src/jaka_a5/jaka_a5_vision/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>jaka_a5_vision</name>
  <version>1.0.0</version>
  <description>JAKA A5 vision package</description>
  <maintainer email="xiang@example.com">Xiang</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>apriltag_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>sensor_msgs</depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF
```

**jaka_a5_bringup/package.xml:**
```bash
cat > ~/ros2_ws/src/jaka_a5/jaka_a5_bringup/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>jaka_a5_bringup</name>
  <version>1.0.0</version>
  <description>JAKA A5 bringup launch files</description>
  <maintainer email="xiang@example.com">Xiang</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>launch</depend>
  <depend>launch_ros</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
```

> **注意**: `jaka_a5_vision` 使用 `ament_python` 构建系统，其他包使用 `ament_cmake`

---

## 第三阶段：创建 URDF/xacro 文件

> **关键概念**: xacro 是 URDF 的宏扩展语言，允许使用变量、宏、条件语句，让 URDF 更易维护

### Step 3.1: 创建主 URDF 文件

`jaka_a5_description/urdf/jaka_a5.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="jaka_a5">

  <!-- 包含机器人模型和 Gazebo 配置 -->
  <xacro:include filename="jaka_a5.robot.xacro" />
  <xacro:include filename="gazebo.urdf.xacro" />

</robot>
```

### Step 3.2: 创建 jaka_a5.robot.xacro

`jaka_a5_description/urdf/jaka_a5.robot.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="jaka_a5">

  <!-- 常量 -->
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
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://jaka_a5_description/meshes/jaka_a5_meshes/J1.STL" />
      </geometry>
      <material name="">
        <color rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://jaka_a5_description/meshes/jaka_a5_meshes/J1.STL" />
      </geometry>
    </collision>
  </link>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 -0.00022535 0.12015" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="J1" />
    <axis xyz="0 0 1" />
    <limit lower="-6.28" upper="6.28" effort="2000" velocity="3.665" />
  </joint>

  <!-- Joint 2 ~ Joint 6 结构类似（省略，详细见源码） -->

  <!-- Eye-in-Hand Camera Mount（空 link 作为挂载点） -->
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

### Step 3.3: 创建 gazebo.urdf.xacro

`jaka_a5_description/urdf/gazebo.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Gazebo 材质 -->
  <gazebo reference="base_link"><material>Gazebo/DarkGrey</material></gazebo>
  <gazebo reference="J1"><material>Gazebo/Blue</material></gazebo>
  <gazebo reference="J2"><material>Gazebo/Blue</material></gazebo>
  <gazebo reference="J3"><material>Gazebo/Blue</material></gazebo>
  <gazebo reference="J4"><material>Gazebo/Grey</material></gazebo>
  <gazebo reference="J5"><material>Gazebo/Blue</material></gazebo>
  <gazebo reference="J6"><material>Gazebo/Grey</material></gazebo>

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

  <!-- Eye-in-Hand Camera Sensor -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30.0</update_rate>
      <topic>/camera/image_raw</topic>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
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

</robot>
```

### Step 3.4: 下载/获取 STL 网格文件

JAKA A5 的 STL 网格文件需要从官方或逆向工程获取。将它们放入：

```
jaka_a5_description/meshes/jaka_a5_meshes/
├── base_link.STL
├── J1.STL
├── J2.STL
├── J3.STL
├── J4.STL
├── J5.STL
└── J6.STL
```

> **注意**: 如果没有 STL 文件，可以先用简单的几何体（box/cylinder）替代进行测试。

---

## 第四阶段：创建 Gazebo 仿真环境

### Step 4.1: 创建 battery_station.world

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

    <!-- 电池 + AprilTag 模型 -->
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

### Step 4.2: 创建 sim.launch.py

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

    # Robot State Publisher
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

    # 时钟桥接
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 相机图像桥接
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

### Step 4.3: 创建 CMakeLists.txt

`jaka_a5_gazebo/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(jaka_a5_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## 第五阶段：配置 ros2_control 控制器

### Step 5.1: 创建 ros2_controllers.yaml

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

### Step 5.2: 创建 control.launch.py

`jaka_a5_control/launch/control.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # controller_manager 节点
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=['/path/to/ros2_controllers.yaml'],
        output='screen'
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen'
    )

    # Arm Controller spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['jaka_a5_arm_controller', '--controller-manager-timeout', '30'],
        output='screen'
    )

    # 等 joint_state_broadcaster 退出后再启动 arm_controller
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

### Step 5.3: 创建 CMakeLists.txt

`jaka_a5_control/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(jaka_a5_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_controllers REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## 第六阶段：配置 MoveIt2

### Step 6.1: 创建 SRDF 文件

`jaka_a5_planning/config/jaka_a5.srdf`:

```xml
<?xml version="1.0"?>
<robot name="jaka_a5">

  <group name="manipulator">
    <joint name="joint_1" />
    <joint name="joint_2" />
    <joint name="joint_3" />
    <joint name="joint_4" />
    <joint name="joint_5" />
    <joint name="joint_6" />
  </group>

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

  <disable_collisions link1="base_link" link2="J1" reason="Adjacent" />
  <disable_collisions link1="J1" link2="J2" reason="Adjacent" />
  <disable_collisions link1="J2" link2="J3" reason="Adjacent" />
  <disable_collisions link1="J3" link2="J4" reason="Adjacent" />
  <disable_collisions link1="J4" link2="J5" reason="Adjacent" />
  <disable_collisions link1="J5" link2="J6" reason="Adjacent" />

</robot>
```

### Step 6.2: 创建 kinematics.yaml

`jaka_a5_planning/config/kinematics.yaml`:

```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

### Step 6.3: 创建 ompl_planning.yaml

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

### Step 6.4: 创建 move_group.launch.py

`jaka_a5_planning/launch/move_group.launch.py`:

```python
from launch import LaunchDescription
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

### Step 6.5: 创建 CMakeLists.txt

`jaka_a5_planning/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(jaka_a5_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_move_group REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## 第七阶段：创建视觉节点

### Step 7.1: 创建 setup.py

`jaka_a5_vision/setup.py`:

```python
from setuptools import setup

package_name = 'jaka_a5_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vs.launch.py']),
        ('share/' + package_name + '/config', ['config/apriltag.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xiang',
    maintainer_email='xiang@example.com',
    description='JAKA A5 vision package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_subscriber = jaka_a5_vision.apriltag_subscriber:main',
            'vs_controller = jaka_a5_vision.vs_controller:main',
        ],
    },
)
```

### Step 7.2: 创建 apriltag_subscriber.py

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
            detection_ids = detection.id if hasattr(detection.id, '__iter__') else [detection.id]
            if detection_ids and detection_ids[0] == self.target_tag_id:
                pose = PoseStamped()
                pose.header = detection.header
                pose.pose = detection.pose.pose
                self.publisher.publish(pose)
                self.get_logger().info(f'Tag detected at x={pose.pose.position.x:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Step 7.3: 创建 vs_controller.py

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

        self.create_subscription(PoseStamped, '/target_pose', self.target_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/jaka_a5_arm_controller/joint_trajectory', 10
        )

        self.declare_parameter('position_threshold', 0.005)
        self.declare_parameter('max_step', 0.02)
        self.declare_parameter('desired_z', 0.1)

        self.target_pose = None
        self.current_joint_positions = [0.0] * 6
        self.joint_name_to_idx = {
            'joint_1': 0, 'joint_2': 1, 'joint_3': 2,
            'joint_4': 3, 'joint_5': 4, 'joint_6': 5
        }

    def joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_name_to_idx:
                self.current_joint_positions[self.joint_name_to_idx[name]] = msg.position[i]

    def target_callback(self, msg):
        self.target_pose = msg
        error = np.sqrt(
            msg.pose.position.x**2 + msg.pose.position.y**2 +
            (msg.pose.position.z - self.get_parameter('desired_z').value)**2
        )

        if error < self.get_parameter('position_threshold').value:
            self.send_stop()
        else:
            self.send_servo_command()

    def compute_delta(self):
        if self.target_pose is None:
            return [0.0] * 6

        scale = 0.5
        x = self.target_pose.pose.position.x
        y = self.target_pose.pose.position.y
        z = self.target_pose.pose.position.z

        delta = [scale * x, scale * y, scale * (z - self.get_parameter('desired_z').value), 0.0, 0.0, 0.0]

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

### Step 7.4: 创建 config/apriltag.yaml

`jaka_a5_vision/config/apriltag.yaml`:

```yaml
apriltag:
  ros__parameters:
    tag_family: 36h11
    tag_ids: [1]
    tag_size: 0.05
    publish_tf: true
```

### Step 7.5: 创建 launch/vs.launch.py

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

    return LaunchDescription([apriltag_node, apriltag_subscriber, vs_controller])
```

### Step 7.6: 创建 __init__.py

`jaka_a5_vision/jaka_a5_vision/__init__.py`:

```python
# Empty file to make the directory a Python package
```

---

## 第八阶段：创建集成启动文件

### Step 8.1: 创建 integration.launch.py

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'sim.launch.py'))
    )

    control = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(control_pkg, 'launch', 'control.launch.py'))
            )
        ]
    )

    moveit = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(planning_pkg, 'launch', 'move_group.launch.py'))
            )
        ]
    )

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

### Step 8.2: 创建 CMakeLists.txt

`jaka_a5_bringup/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(jaka_a5_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## 第九阶段：构建和验证

### Step 9.1: 构建工作空间

```bash
cd ~/ros2_ws

# 首次构建
colcon build

# 如果修改了代码，重新构建
colcon build --cmake-clean-cache

# 只构建特定包
colcon build --packages-select jaka_a5_description
```

### Step 9.2: 设置环境并验证

```bash
# 设置环境
source ~/ros2_ws/install/setup.bash

# 检查包
ros2 pkg list | grep jaka_a5

# 列出话题
ros2 topic list
```

### Step 9.3: 启动完整系统

```bash
# 方式1：使用集成启动文件
ros2 launch jaka_a5_bringup integration.launch.py

# 方式2：分步启动（调试用）
# 终端1: Gazebo
ros2 launch jaka_a5_gazebo sim.launch.py

# 终端2: 控制器
ros2 launch jaka_a5_control control.launch.py

# 终端3: MoveIt2
ros2 launch jaka_a5_planning move_group.launch.py

# 终端4: 视觉
ros2 launch jaka_a5_vision vs.launch.py
```

### Step 9.4: 验证各模块

```bash
# 检查控制器状态
ros2 control list_controllers

# 查看相机图像话题
ros2 topic hz /camera/image_raw

# 查看 AprilTag 检测
ros2 topic echo /tag_detections --once

# 查看关节状态
ros2 topic echo /joint_states --once

# 使用 rqt_image_view 查看图像
rqt_image_view /camera/image_raw
```

---

## 第十阶段：常见问题排查

### 问题排查清单

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| robot_state_publisher aborts | URDF 缺少 world link | 添加 `<link name="world" />` |
| 控制器激活失败 | gz_ros2_control 未正确加载 | 检查 gazebo.urdf.xacro 中的插件配置 |
| 相机无图像 | 桥接配置错误 | 检查 sim.launch.py 中的桥接话题 |
| AprilTag 检测不到 | 参数不匹配 | 确认 tag_size 与实际标签一致 |
| MoveIt 无法执行 | 缺少 moveit_controllers.yaml | 创建配置文件 |
| 机械臂不运动 | 控制器未激活 | 检查 ros2 control list_controllers |

---

## 第十阶段（补充）：验证命令详解

### 验证命令速查表

```bash
# ============ 环境验证 ============
# 1. ROS2 安装验证
ros2 doctor
# 期望：无 ERROR，有 warning 可接受

# 2. Gazebo 安装验证
gz sim --version
# 期望：显示版本号（如 fortress 或 harmonic）

# ============ 构建验证 ============
# 3. xacro 解析验证
xacro ~/ros2_ws/src/jaka_a5/jaka_a5_description/urdf/jaka_a5.urdf.xacro > /tmp/urdf_output.xml
echo $?  # 返回 0 表示成功

# 4. 包发现验证
colcon list
# 期望：显示 jaka_a5_description, jaka_a5_gazebo, 等

# 5. 构建验证
colcon build 2>&1 | tail -20
# 期望：显示 "Summary: X packages succeeded"

# ============ 运行时验证 ============
# 6. 控制器状态验证
ros2 control list_controllers
# 期望：joint_state_broadcaster 和 jaka_a5_arm_controller 显示 active

# 7. 话题列表验证
ros2 topic list | grep -E "camera|joint|tag"
# 期望：看到 /camera/image_raw, /joint_states, /tag_detections

# 8. 相机图像话题频率验证
ros2 topic hz /camera/image_raw
# 期望：显示 ~30 Hz（相机配置帧率）

# 9. TF 树验证
ros2 run rqt_tf_tree rqt_tf_tree &
# 期望：看到 world -> base_link -> J1 -> ... -> J6 -> camera_link

# 10. RViz 查看机械臂模型
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/jaka_a5/jaka_a5_description/rviz/jaka_a5.rviz
# 期望：看到完整的机械臂模型（需要先启动 robot_state_publisher）
```

---

## 第十一阶段（补充）：相机视角调整详解

### 相机视角参数说明

在 `jaka_a5.robot.xacro` 中，相机的挂载通过以下参数控制：

```xml
<joint name="camera_joint" type="fixed">
  <origin xyz="0 0 0.02" rpy="0 0 0" />
  <parent link="J6" />
  <child link="camera_link" />
</joint>
```

**xyz 参数**：
| 值 | 含义 |
|-----|------|
| `0 0 0.02` | 相机在 J6 法兰上方 2cm |
| `0 0.05 0.02` | 相机在 J6 前方 5cm，上方 2cm |
| `0 0 -0.02` | 相机在 J6 法兰下方 2cm（朝下） |

**rpy 参数**：
| 值 | 含义 |
|-----|------|
| `0 0 0` | 默认朝上（与法兰同向） |
| `-0.785 0 0` | 向下倾斜 45° |
| `-1.571 0 0` | 向下倾斜 90°（垂直朝下） |
| `0 0.785 0` | 向后倾斜 45° |

### 不同应用场景的配置

```xml
<!-- 场景1：俯视工作台（最常用） -->
<origin xyz="0 0 0.02" rpy="-0.785 0 0" />

<!-- 场景2：相机朝前下方看 -->
<origin xyz="0.05 0 0.02" rpy="-0.785 0 0" />

<!-- 场景3：相机垂直朝下 -->
<origin xyz="0 0 0" rpy="-1.571 0 0" />

<!-- 场景4：相机朝向斜后方向 -->
<origin xyz="0 0 0.02" rpy="0 0.785 -0.785" />
```

### 验证相机视角

启动 Gazebo 后，使用 `rqt_image_view` 查看相机图像：

```bash
# 查看相机图像
rqt_image_view /camera/image_raw

# 或者使用 image_view
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw
```

如果看不到目标，调整 `rpy` 值直到能看到 AprilTag 标记。

---

## 第十二阶段（补充）：完整启动脚本

### 启动脚本 (run_system.sh)

创建 `~/ros2_ws/run_system.sh`:

```bash
#!/bin/bash
# JAKA A5 视觉闭环系统启动脚本

# 设置工作空间环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "=========================================="
echo "JAKA A5 视觉闭环路径规划系统"
echo "=========================================="

# 构建工作空间（如需要）
if [ "$1" == "build" ]; then
    echo "[1/4] 构建工作空间..."
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
fi

# 启动完整系统
echo "[2/4] 启动 Gazebo 仿真..."
ros2 launch jaka_a5_bringup integration.launch.py

# 等待系统启动（约 10 秒）
sleep 10

echo "[3/4] 系统已启动！"
echo ""
echo "可用命令："
echo "  ros2 control list_controllers  - 查看控制器状态"
echo "  ros2 topic hz /camera/image_raw - 查看相机帧率"
echo "  rqt_image_view /camera/image_raw - 查看相机图像"
echo "  ros2 topic echo /tag_detections - 查看 AprilTag 检测"
echo ""
echo "按 Ctrl+C 停止系统"
```

赋予执行权限并使用：

```bash
chmod +x ~/ros2_ws/run_system.sh
./run_system.sh        # 直接启动
./run_system.sh build  # 构建并启动
```

---

## 附录：完整文件树

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

---

*本文档由 Claude (蒠芷) 生成，详细记录了项目搭建的每一步*
*更新日期: 2026-04-20*
# ROS2 机械臂视觉闭环 — 知识模块汇总

> 本文档整理毕设项目中各模块的相关知识

---

## 目录

1. [ROS2 核心概念](#1-ros2-核心概念)
2. [机械臂运动学基础](#2-机械臂运动学基础)
3. [Gazebo 仿真原理](#3-gazebo-仿真原理)
4. [ros2_control 控制器框架](#4-ros2_control-控制器框架)
5. [MoveIt2 运动规划](#5-moveit2-运动规划)
6. [视觉伺服控制](#6-视觉伺服控制)
7. [Eye-in-Hand 相机标定](#7-eye-in-hand-相机标定)
8. [AprilTag 视觉检测](#8-apriltag-视觉检测)

---

## 1. ROS2 核心概念

### 1.1 节点（Node）

ROS2 中的最小执行单元，每个节点负责一个特定功能。

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # 创建发布者/订阅者

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 1.2 话题（Topic）

节点间异步通信方式，发布-订阅模型。

```bash
# 查看活跃话题
ros2 topic list

# 手动发布消息
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello'"

# 监听话题
ros2 topic echo /chatter
```

### 1.3 服务（Service）

同步请求-响应模型。

```bash
# 调用服务
ros2 service call /spawn_entity ros_gz_sim/srv/SpawnEntity "{name: robot}"
```

### 1.4 Launch 文件

启动多个节点和配置的标准方式。

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='pkg', executable='node', name='node_name')
    ])
```

### 1.5 参数（Parameter）

节点配置数据。

```python
# 声明参数
self.declare_parameter('param_name', default_value)

# 读取参数
value = self.get_parameter('param_name').value
```

### 1.6 TF 坐标变换

机器人各部件之间的空间关系。

```bash
# 查看 TF 树
ros2 run rqt_tf_tree rqt_tf_tree

# 监听 TF
ros2 run tf2_ros tf2_echo world camera_link
```

---

## 2. 机械臂运动学基础

### 2.1 正运动学（Forward Kinematics）

已知关节角度，计算末端执行器位置和姿态。

```
关节角度 (θ1, θ2, ..., θn) → 末端位姿 (position, orientation)
```

JAKA A5 的 DH 参数（标准配置）：

| 连杆 | a (m) | α (rad) | d (m) | θ (rad) |
|------|-------|---------|-------|---------|
| 1 | 0 | -π/2 | 0.12015 | θ1 |
| 2 | 0.43 | 0 | 0 | θ2+π/2 |
| 3 | 0.3685 | 0 | -0.115 | θ3 |
| 4 | 0 | -π/2 | 0 | θ4-π/2 |
| 5 | 0 | π/2 | -0.1135 | θ5 |
| 6 | 0 | 0 | 0.107 | θ6 |

### 2.2 逆运动学（Inverse Kinematics）

已知末端位姿，计算所需的关节角度。

MoveIt2 使用 KDL 求解器：
```yaml
kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

### 2.3 雅可比矩阵（Jacobian）

描述关节速度与末端速度之间的关系：

```
v = J(θ) × θ̇
```

视觉伺服中用于将图像误差转换为关节速度。

### 2.4 坐标系

| 坐标系 | 说明 |
|--------|------|
| base_link | 机械臂基座中心 |
| J6 | 末端法兰中心 |
| camera_link | 相机光心 |
| world | 世界原点 |

---

## 3. Gazebo 仿真原理

### 3.1 Gazebo Sim vs Classic Gazebo

| 特性 | Gazebo Classic | Gazebo Sim (Ignition/Fortress) |
|------|---------------|--------------------------------|
| 命令 | `gazebo` | `gz sim` |
| ROS 桥接 | `gazebo_ros_pkgs` | `ros_gz_sim` |
| 控制器插件 | `libgazebo_ros2_control.so` | `gz_ros2_control` (系统插件) |

### 3.2 仿真流程

```
1. 加载 world 文件（环境、光照、模型）
2. 解析 URDF 生成 SDF
3. 创建物理引擎（ODE/Bullet/Simplex）
4. 启动传感器（相机、IMU 等）
5. 运行控制循环（1000Hz 物理，30Hz 传感器）
```

### 3.3 传感器桥接

Gazebo 传感器 → ROS2 话题需要通过桥接：

```python
# 图像桥接（Gazebo → ROS2）
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image']
)
```

### 3.4 SDF 模型文件

World 文件使用 SDF（Simulation Description Format）：

```xml
<sdf version="1.6">
  <world name="battery_station">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
    </physics>
    <model name="workbench">
      <pose>0 0 0.8 0 0 0</pose>
      <static>true</static>
      <link name="link">...</link>
    </model>
  </world>
</sdf>
```

---

## 4. ros2_control 控制器框架

### 4.1 架构

```
Hardware → Controller Manager → Controllers → actuator/motor
     ↓
  State Interface (位置、速度、力矩)
     ↓
  Command Interface (位置、速度、力矩)
```

### 4.2 控制器类型

| 控制器 | 用途 |
|--------|------|
| JointStateBroadcaster | 广播关节状态 |
| JointTrajectoryController | 轨迹跟踪控制 |
| Effort/JointPosition/JointVelocityController | 单环控制 |

### 4.3 ros2_control 与 Gazebo 集成

使用 `gz_ros2_control` 插件：

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>config/ros2_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### 4.4 控制器配置示例

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

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
```

### 4.5 spawner 启动顺序

必须先启动 `joint_state_broadcaster`，再启动其他控制器：

```python
from launch.event_handlers import OnProcessExit

arm_controller_after_joint_state = RegisterEventHandler(
    OnProcessExit(
        target_action=joint_state_broadcaster_spawner,
        on_exit=[arm_controller_spawner]
    )
)
```

---

## 5. MoveIt2 运动规划

### 5.1 MoveIt2 架构

```
User → MoveGroup Interface → Planning Pipeline
                              → OMPL/RRTConnect
                              → Collision Checking (FCL)
                              → Controller Interface
```

### 5.2 核心组件

| 组件 | 说明 |
|------|------|
| MoveGroup | 主规划接口 |
| Planning Scene | 规划场景管理 |
| CollisionEnvironment | 碰撞检测 |
| Kinematics Plugin | 运动学求解（KDL/Trac_IK） |
| OMPL | 规划算法库 |

### 5.3 SRDF 配置

定义规划组和预定义姿态：

```xml
<group name="manipulator">
  <joint name="joint_1" />
  <joint name="joint_2" />
  <!-- ... -->
</group>

<group_state name="home" group="manipulator">
  <joint name="joint_1" value="0" />
  <!-- ... -->
</group_state>
```

### 5.4 OMPL 规划算法

| 算法 | 适用场景 |
|------|---------|
| RRTConnect | 快速探索，快速收敛 |
| RRT* | 渐近最优，耗时更长 |
| PRM | 多查询，路径数据库 |
| SBL | 约束规划 |

### 5.5 末端姿态约束

有时需要限制末端姿态（如保持水平）：

```cpp
moveit_msgs::OrientationConstraint ocm;
ocm.link_name = "tool0";
ocm.orientation = {0, 0, 0, 1};  // 四元数
ocm.absolute_x_axis_tolerance = 0.05;
ocm.absolute_y_axis_tolerance = 0.05;
ocm.absolute_z_axis_tolerance = 3.14;  // Z 轴不约束
```

### 5.6 规划流程

```python
# Python MoveIt2 API
move_group = MoveGroupInterface("manipulator", "robot_description")

move_group.set_named_target("home")
plan = move_group.plan()

move_group.execute(plan)
```

---

## 6. 视觉伺服控制

### 6.1 视觉伺服的两种方式

| 类型 | 全称 | 原理 | 优缺点 |
|------|------|------|--------|
| **IBVS** | Image-Based Visual Servoing | 直接在图像空间控制 | 不需要相机标定，但容易陷入奇异性 |
| **PBVS** | Position-Based Visual Servoing | 在 3D 空间控制 | 全局稳定，但需要精确标定 |

### 6.2 IBVS 控制律

```
图像特征误差: e = s - s*
相机速度: v = -λ × J⁺ × e
关节速度: q̇ = J_v⁺ × v
```

其中 `J` 是图像雅可比矩阵，`J⁺` 是伪逆。

### 6.3 图像雅可比矩阵

将图像特征速度映射到相机速度：

```
ṡ = J_img × v_cam
```

简化形式（2D 点特征）：

```
J_img = [ -1/Z   0   x/Z   x*y  -(1+x²)  y ]
         [  0  -1/Z  y/Z  (1+y²)  -x*y   -x ]
           (u)   (v)  (depth) (image moments)
```

### 6.4 视觉伺服流程

```
1. 检测图像特征（AprilTag 角点）
2. 计算特征误差 e = s - s*
3. 估计特征深度 Z
4. 计算图像雅可比矩阵 J
5. 计算伪逆 J⁺
6. 计算控制 law: v = -λ × J⁺ × e
7. 转换为关节速度 q̇ = J_v⁺ × v
8. 发送关节速度命令
9. 重复
```

### 6.5 IBVS 的奇异性问题

当图像雅可比矩阵接近奇异时，控制量会变得非常大。

**缓解方法**：
- 添加最小奇异值约束
- 使用 Damped Least Squares (DLS)
- 切换到 PBVS

---

## 7. Eye-in-Hand 相机标定

### 7.1 手眼标定问题

已知：
- 相机相对于机械臂末端的变换 `T_cam2flange`
- 标定板相对于相机的变换 `T_board2cam`

求解：
- 机械臂基座相对于标定板的变换 `T_base2board`

### 7.2 标定方法

**Eye-in-Hand (相机在末端)**：

```
T_board2base = T_board2cam × T_cam2flange × T_flange2base
```

使用 OpenCV 的 `calibrateHandEye()`:

```python
import cv2
# 收集 N ≥ 15 个不同位姿的 (R_base2gripper, t_base2gripper) 和 (R_target2cam, t_target2cam)
T_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_DANIILIDIS
)
```

### 7.3 相机内参标定

使用 ROS 标定工具：

```bash
ros2 run camera_calibration cameracalibrator.py --approximate 0.0 --ros-args -p image:=/camera/image_raw -p camera:=/camera
```

标定后得到：
- 焦距 (fx, fy)
- 主点 (cx, cy)
- 畸变系数 (k1, k2, p1, p2)

### 7.4 标定精度验证

重投影误差 < 2 像素视为合格：

```
error = sqrt((u_measured - u_predicted)² + (v_measured - v_predicted)²)
```

---

## 8. AprilTag 视觉检测

### 8.1 AprilTag 简介

视觉基准标记，用于位姿估计。

| 家族 | 标记数量 | 用途 |
|------|----------|------|
| 36h11 | 587 | 通用，最常用 |
| 25h9 | 35 | 较大标记 |
| 16h5 | 30 | 小型标记 |

### 8.2 AprilTag 检测流程

```
1. 灰度化图像
2. 阈值分割
3. 轮廓检测
4. 拟合四边形
5. 解码（汉明距离）
6. 位姿估计（PnP）
```

### 8.3 位姿估计

已知：
- AprilTag 实际尺寸（tag_size）
- 相机内参 (fx, fy, cx, cy)
- AprilTag 角点在图像中的位置

求解：
- AprilTag 相对于相机的 3D 位姿

使用 EPnP 或 IPPE 算法。

### 8.4 ROS2 apriltag_ros 使用

```bash
ros2 run apriltag_ros apriltag_node
```

参数配置：
```yaml
apriltag:
  ros__parameters:
    tag_family: 36h11
    tag_ids: [1, 2, 3]
    tag_size: 0.05  # 5cm
    publish_tf: true
```

### 8.5 tf 树变换

AprilTag 检测结果通过 TF 发布：

```
camera_link → tag_id (相对位姿发布为 /tf 话题)
```

使用：
```bash
ros2 run tf2_ros tf2_echo camera_link tag_1
```

---

## 附录：关键命令汇总

```bash
# ROS2 基础
ros2 doctor                    # 检查 ROS2 安装
ros2 topic list                # 查看活跃话题
ros2 topic echo <topic>        # 监听话题
ros2 node list                 # 查看活跃节点
ros2 pkg list                   # 查看已安装包

# 控制器
ros2 control list_controllers   # 查看控制器状态
ros2 control controller_manager_spawner <controller>

# MoveIt2
ros2 run moveit2_scripts move_group

# Gazebo
gz sim -r world_file           # 运行 Gazebo
gz topic -l                    # 列出 Gazebo 话题

# TF
ros2 run rqt_tf_tree rqt_tf_tree
ros2 run tf2_tools view_frames

# 相机
rqt_image_view /camera/image_raw
ros2 topic hz /camera/image_raw
```

---

*本文档由 Claude (蒠芷) 整理*
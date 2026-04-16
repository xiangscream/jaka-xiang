# JAKA A5 ROS2 Gazebo 视觉闭环伺服 — 技术架构文档

> **版本**: v1.0
> **日期**: 2026-04-16
> **项目**: 基于 ROS2 Humble 的 JAKA A5 机械臂换电池工作站仿真

---

## 一、整体架构概览

### 1.1 系统拓扑

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           用户空间 (User Space)                          │
│                                                                          │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────────────┐    │
│  │  RViz GUI    │     │  Gazebo GUI  │     │  AprilTag 检测结果   │    │
│  │  (可视化)    │     │  (物理仿真)  │     │  (视觉反馈)          │    │
│  └──────────────┘     └──────────────┘     └──────────────────────┘    │
│         │                   │                        │                   │
└─────────┼───────────────────┼────────────────────────┼───────────────────┘
          │                   │                        │
┌─────────▼───────────────────▼────────────────────────▼───────────────────┐
│                          ROS2 Humble DDS 层                              │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                    ros2_control Controller Manager                  │    │
│  │  ┌─────────────────────┐    ┌─────────────────────────────┐      │    │
│  │  │ JointStateBroadcaster│    │ JointTrajectoryController   │      │    │
│  │  │ (/joint_states)      │    │ (position command)          │      │    │
│  │  └─────────────────────┘    └─────────────────────────────┘      │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    ▲                                     │
│  ┌──────────────┐     ┌───────────┴──────────┐     ┌──────────────┐    │
│  │ MoveIt2      │────▶│ /joint_trajectory    │◀────│ 视觉伺服     │    │
│  │ move_group   │     │ /controller/trajectory│     │ 控制器       │    │
│  └──────────────┘     └──────────────────────┘     └──────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
                                    ▲
┌───────────────────────────────────┼────────────────────────────────────┐
│                          Gazebo Fortress                                 │
│  ┌────────────────────┐    ┌──────────────────────────────────────┐      │
│  │ gazebo_ros2_control│◀───│       Robot Model (URDF)             │      │
│  │ Plugin             │    │  base_link → J1 → J2 → J3 → J4 →     │      │
│  └────────────────────┘    │  J5 → J6 → camera_link               │      │
│                            └──────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────────┘
                                    ▲
┌───────────────────────────────────┼────────────────────────────────────┐
│                           仿真世界层                                      │
│  ┌─────────────┐  ┌─────────────────┐  ┌──────────────────────────┐     │
│  │ Ground      │  │ Workbench       │  │ Battery + AprilTag       │     │
│  │ Plane       │  │ (工作台)        │  │ (视觉目标)               │     │
│  └─────────────┘  └─────────────────┘  └──────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.2 分层架构

| 层级 | 包 | 职责 | 关键文件 |
|------|-----|------|----------|
| **视觉层 (Vision)** | `jaka_a5_vision` | AprilTag检测 + 视觉伺服控制 | `vs_controller.py`, `apriltag_subscriber.py` |
| **规划层 (Planning)** | `jaka_a5_planning` | MoveIt2 运动规划 | `move_group.launch.py`, `*.srdf`, `*.yaml` |
| **控制层 (Control)** | `jaka_a5_control` | ros2_control 控制器管理 | `ros2_controllers.yaml`, `control.launch.py` |
| **仿真层 (Gazebo)** | `jaka_a5_gazebo` | 物理世界 + 机器人spawn | `battery_station.world`, `sim.launch.py` |
| **描述层 (Description)** | `jaka_a5_description` | URDF/xacro 机器人模型 | `jaka_a5.robot.xacro`, `gazebo.urdf.xacro` |

### 1.3 数据流

```
用户目标位置
      │
      ▼
┌─────────────────────────────────────────────────────────────┐
│                    MoveIt2 (运动规划)                        │
│  /joint_trajectory_controller/trajectory                     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              ros2_control JointTrajectoryController         │
│                    位置指令 (position)                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│           gazebo_ros2_control Plugin (Gazebo)               │
│                   物理仿真更新                               │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    JointStateBroadcaster                    │
│                     /joint_states                           │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│ MoveIt2       │    │ 视觉伺服      │    │ Robot State   │
│ (状态反馈)    │    │ 控制器       │    │ Publisher     │
└───────────────┘    └───────────────┘    └───────────────┘
                              │
                              ▼
                     ┌─────────────────┐
                     │ /target_pose    │
                     │  (目标位置)     │
                     └─────────────────┘
                              │
                              ▼
                     ┌─────────────────┐
                     │ 视觉伺服控制    │
                     │ 计算关节增量    │
                     └─────────────────┘
```

---

## 二、描述层 (jaka_a5_description)

### 2.1 职责

- 提供 JAKA A5 机械臂的完整 URDF/xacro 描述
- 定义 6 个关节的 kinematics 和 dynamics 参数
- 配置 Gazebo 所需的物理属性和材质
- 包含 Eye-in-Hand 相机 mount

### 2.2 文件结构

```
jaka_a5_description/
├── package.xml
├── CMakeLists.txt
├── urdf/
│   ├── jaka_a5.robot.xacro      # 机器人本体 (无Gazebo特定配置)
│   ├── gazebo.urdf.xacro        # Gazebo物理引擎配置
│   └── jaka_a5.urdf.xacro       # 主入口 (包含上述两者)
├── meshes/
│   └── jaka_a5_meshes/          # STL mesh 文件
│       ├── base_link.STL
│       ├── J1.STL ~ J6.STL
├── rviz/
│   └── jaka_a5.rviz
└── launch/
    └── view_robot.launch.py
```

### 2.3 机器人模型

**URDF 链式结构:**
```
world ──[fixed joint]──▶ base_link ──[joint_1]──▶ J1 ──[joint_2]──▶ J2
──[joint_3]──▶ J3 ──[joint_4]──▶ J4 ──[joint_5]──▶ J5 ──[joint_6]──▶ J6
──[fixed joint]──▶ camera_link
```

**关节参数:**

| 关节 | 类型 | 范围 (rad) | 力矩 (N·m) | 速度 (rad/s) |
|------|------|-----------|-----------|-------------|
| joint_1 | revolute | ±6.28 | 2000 | 3.665 |
| joint_2 | revolute | -1.48 ~ 4.63 | 5000 | 3.665 |
| joint_3 | revolute | ±3.05 | 5000 | 3.665 |
| joint_4 | revolute | -1.48 ~ 4.62 | 2000 | 3.665 |
| joint_5 | revolute | ±6.28 | 3000 | 3.665 |
| joint_6 | revolute | ±6.28 | 1000 | 3.665 |

### 2.4 xacro 模块化设计

**jaka_a5.robot.xacro** — 纯机器人几何/运动学:
- 定义所有 link 的 visual、collision、inertial
- 定义所有 joint 的类型、轴、限制
- 定义 transmissions（与 ros2_control 接口）

**gazebo.urdf.xacro** — Gazebo 特定配置:
- `<gazebo reference>` 为每个 link 指定 Gazebo 材质
- `gazebo_ros2_control` plugin 配置
- ROS2 control 插件注入

**jaka_a5.urdf.xacro** — 组合入口:
```xml
<xacro:include filename="jaka_a5.robot.xacro" />
<xacro:include filename="gazebo.urdf.xacro" />
```

---

## 三、仿真层 (jaka_a5_gazebo)

### 3.1 职责

- 提供 Gazebo 物理世界定义
- 管理机器人的 spawn（从 URDF 到 Gazebo）
- 配置仿真环境（光照、地面、工作台、目标物体）

### 3.2 文件结构

```
jaka_a5_gazebo/
├── package.xml
├── CMakeLists.txt
├── worlds/
│   └── battery_station.world      # SDF 世界文件
└── launch/
    └── sim.launch.py              # 仿真启动文件
```

### 3.3 World 文件配置

**battery_station.world** 采用 SDF 1.8 格式:

| 组件 | 类型 | 参数 |
|------|------|------|
| **physics** | ODE | max_step_size=0.001, real_time_factor=1.0, update_rate=1000 |
| **scene** | - | ambient=0.4, background=0.7, shadows=true |
| **light** | directional | sun, cast_shadows=true |
| **ground_plane** | static | 10×10m 平面 |
| **workbench** | static | 1.0×0.8×0.05m, z=0.8m |
| **battery_with_tag** | dynamic | 0.1×0.05×0.02m, 红色, 含AprilTag白块 |

### 3.4 Spawn 机制

**流程:**
```
sim.launch.py
    │
    ├── 1. gz sim -r battery_station.world
    │       └── 启动 Gazebo Fortress + 加载世界
    │
    ├── 2. subprocess: xacro + ros2 service call
    │       └── 处理 URDF → 调用 /spawn_entity 服务
    │
    └── 3. robot_state_publisher
            └── 发布 robot_description TF 树
```

**关键实现:**
- 使用 `TimerAction(period=3.0)` 等待 Gazebo 完全启动
- xacro 处理后的 XML 通过 `ros2 service call /spawn_entity` 传入 Gazebo
- `robot_state_publisher` 使用处理后的 URDF XML

---

## 四、控制层 (jaka_a5_control)

### 4.1 职责

- 配置 ros2_control 控制器管理器
- 定义 JointTrajectoryController 参数
- 管理控制器的生命周期（spawner）

### 4.2 文件结构

```
jaka_a5_control/
├── package.xml
├── CMakeLists.txt
├── config/
│   └── ros2_controllers.yaml        # 控制器配置
└── launch/
    └── control.launch.py            # 控制器启动文件
```

### 4.3 控制器配置

**controller_manager:**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # 100Hz 控制循环
```

**JointStateBroadcaster:**
```yaml
joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster
```

**JointTrajectoryController:**
```yaml
jaka_a5_arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
  command_interfaces: [position]
  state_interfaces: [position, velocity]
  gains:  # PID 位置控制增益
    joint_1: {p: 100.0, i: 0.0, d: 10.0}
    ...
```

### 4.4 控制器启动流程

```
control.launch.py
    │
    ├── controller_manager (ros2_control_node)
    │       └── 提供控制器管理服务
    │
    ├── RegisterEventHandler: OnProcessStart(controller_manager)
    │       └── joint_state_broadcaster spawner
    │       └── jaka_a5_arm_controller spawner
    │
    └── 各 spawner 通过 service 调用注册到 controller_manager
```

### 4.5 与 Gazebo 的关系

**重要:** `gazebo_ros2_control` plugin 嵌入在 URDF 中（gazebo.urdf.xacro），在 Gazebo 启动时自动加载。该 plugin 会在 Gazebo 进程中创建 controller_manager。

**启动顺序:**
1. Gazebo 启动 → 加载 robot model → ros2_control plugin 初始化
2. ros2_control_node 启动 → 连接 plugin 的 controller_manager
3. spawner 调用 → 注册 broadcasters 和 controllers

---

## 五、规划层 (jaka_a5_planning)

### 5.1 职责

- 配置 MoveIt2 运动规划
- 定义机器人规划组和关节限制
- 设置 OMPL 路径规划算法
- 提供 move_group 节点

### 5.2 文件结构

```
jaka_a5_planning/
├── package.xml
├── CMakeLists.txt
├── config/
│   ├── jaka_a5.srdf              # 语义机器人描述
│   ├── joint_limits.yaml         # 关节限制
│   ├── kinematics.yaml           # 运动学求解器配置
│   ├── ompl_planning.yaml        # OMPL 规划器配置
│   └── moveit.rviz               # RViz 配置
└── launch/
    └── move_group.launch.py       # move_group 启动文件
```

### 5.3 SRDF 配置

**规划组定义:**
```xml
<group name="jaka_a5">
  <joint name="joint_1"/>
  <joint name="joint_2"/>
  <joint name="joint_3"/>
  <joint name="joint_4"/>
  <joint name="joint_5"/>
  <joint name="joint_6"/>
</group>
```

**注意:** `fixed` 关节（world → base_link）不包含在规划组中，因为它不可运动。

**预设姿态:**
| 姿态名 | joint_1 | joint_2 | joint_3 | joint_4 | joint_5 | joint_6 |
|--------|---------|---------|---------|---------|---------|---------|
| zero | 0 | 0 | 0 | 0 | 0 | 0 |
| home | 0 | 1.57 | -1.57 | 1.57 | 1.57 | 0 |
| ready | 0 | 0.5 | -0.8 | 0.8 | 0.5 | 0 |

### 5.4 关节限制

```yaml
joint_limits:
  joint_1:
    has_velocity_limits: true
    max_velocity: 1.57  # rad/s
    has_acceleration_limits: false
  # ... joint_2 ~ joint_6 相同
```

### 5.5 运动学求解器

```yaml
jaka_a5:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.1  # 100ms
```

使用 KDL (Kinematics Dynamics Library) 的迭代求解器。

### 5.6 OMPL 规划配置

```yaml
planning_pipeline:
  planners:
    - name: ompl
      default_planner: RRTConnectkConfigDefault

OMPL:
  default_planner_config: RRTConnectkConfigDefault
  planner_configs:
    RRTConnectkConfigDefault:
      type: geometric::RRTConnectkConfigDefault
      range: 0.0
      goal_bias: 0.05
```

**RRTConnect 算法特点:**
- 双向快速探索随机树
- 高维空间效率高
- 适用于机械臂规划

### 5.7 move_group 启动

```python
move_group.launch.py
    │
    ├── xacro 处理 → robot_description
    ├── 读取 SRDF → robot_description_semantic
    ├── 读取 joint_limits.yaml
    ├── 读取 OMPL 配置
    │
    ├── move_group node
    │       └── 监听 /joint_trajectory_controller/trajectory
    │
    └── rviz node
            └── 提供可视化界面
```

---

## 六、视觉层 (jaka_a5_vision)

### 6.1 职责

- 配置 AprilTag 检测参数
- 接收 AprilTag 检测结果 → 发布目标位姿
- 实现视觉伺服控制器（PBVS）
- 发送关节轨迹命令到 ros2_control

### 6.2 文件结构

```
jaka_a5_vision/
├── package.xml
├── setup.py
├── jaka_a5_vision/
│   ├── __init__.py
│   ├── apriltag_subscriber.py    # AprilTag 检测结果处理
│   └── vs_controller.py           # 视觉伺服控制器
├── config/
│   └── apriltag.yaml              # AprilTag 检测配置
└── launch/
    └── vs.launch.py               # 视觉系统启动文件
```

### 6.3 AprilTag 配置

```yaml
apriltag_ros:
  ros__parameters:
    camera_name: camera
    image_topic: /camera/image_raw
    camera_info_topic: /camera/camera_info
    tag_family: 36h11        # AprilTag 36h11 家族
    tag_ids: [1]             # 检测 ID=1 的标签
    tag_size: 0.05           # 标签边长 5cm
    publish_tf: true         # 发布 TF 变换
    debug: false
```

### 6.4 视觉伺服架构

**PBVS (Position-Based Visual Servo):**

```
┌──────────────────────────────────────────────────────────────┐
│                      视觉伺服控制回路                         │
│                                                              │
│  相机图像 ──▶ AprilTag检测 ──▶ 目标位姿 (tag_id=1)            │
│                                          │                   │
│                                          ▼                   │
│                                   ┌─────────────┐           │
│                                   │ 误差计算    │           │
│                                   │ e = p_des - p_cur      │
│                                   └─────────────┘           │
│                                          │                   │
│                                          ▼                   │
│                                   ┌─────────────┐           │
│                                   │ 控制律      │           │
│                                   │ Δq = λ·L⁺·e│           │
│                                   └─────────────┘           │
│                                          │                   │
│                                          ▼                   │
│                                   ┌─────────────┐           │
│                                   │ 关节增量    │           │
│                                   │ Δq = [Δj1~6]│           │
│                                   └─────────────┘           │
│                                          │                   │
│                                          ▼                   │
│                               /joint_trajectory_controller   │
│                                 /trajectory                  │
└──────────────────────────────────────────────────────────────┘
```

### 6.5 节点详解

**apriltag_subscriber.py:**
```python
AprilTagSubscriber(Node)
    │
    ├── 订阅: /tag_detections (apriltag_msgs/AprilTagDetectionArray)
    │       └── 检测到 tag_id=1 时提取 pose
    │
    └── 发布: /target_pose (geometry_msgs/PoseStamped)
            └── 相机坐标系下的目标位置
```

**vs_controller.py:**
```python
VisualServoController(Node)
    │
    ├── 参数:
    │   ├── position_threshold: 0.005  # 5mm 停止阈值
    │   ├── max_step: 0.02             # 20mm 最大步长
    │   └── desired_z: 0.1             # 目标高度 100mm
    │
    ├── 订阅:
    │   ├── /target_pose (PoseStamped)      # 目标位置
    │   └── /joint_states (JointState)      # 当前关节状态
    │
    ├── 发布: /joint_trajectory_controller/trajectory
    │
    └── 控制律:
            Δq = scale × [x, y, z-desired_z, 0, 0, 0]
            其中 scale=0.5, max_step 限制
```

### 6.6 控制律详解

```python
def compute_delta(self):
    x, y, z = target_pose.position
    desired_z = self.get_parameter('desired_z').value

    scale = 0.5
    delta = [
        scale * x,           # X方向映射到 joint_1
        scale * y,           # Y方向映射到 joint_2
        scale * (z - desired_z),  # Z方向映射到 joint_3
        0.0, 0.0, 0.0        # 姿态暂未控制
    ]

    # 限幅
    for i in range(3):
        delta[i] = clamp(delta[i], -max_step, max_step)

    return delta
```

**局限性:** 当前控制律为简化版本，假设:
- X方向主要影响 joint_1
- Y方向主要影响 joint_2
- Z方向主要影响 joint_3

实际机械臂运动学更复杂，需要 Jacobian 逆矩阵进行精确控制。

### 6.7 视觉系统启动

```python
vs.launch.py
    │
    ├── apriltag_ros/apriltag_node
    │       └── 监听相机话题，发布检测结果
    │
    ├── jaka_a5_vision/apriltag_subscriber
    │       └── 转换检测结果为目标位姿
    │
    └── jaka_a5_vision/vs_controller
            └── 视觉伺服控制循环
```

---

## 七、集成层 (jaka_a5)

### 7.1 职责

- 整合所有子系统
- 管理启动顺序和时序
- 提供单一入口启动整个系统

### 7.2 文件结构

```
jaka_a5/                    # Meta-package
├── package.xml
├── setup.py
└── launch/
    └── integration.launch.py
```

### 7.3 启动时序

```
integration.launch.py
    │
    ├── t=0s: Gazebo simulation
    │       └── 启动 Gazebo + 加载 world + spawn robot
    │
    ├── t=2s: ros2_control
    │       └── 启动 controller_manager + 加载控制器
    │
    ├── t=4s: MoveIt2
    │       └── 启动 move_group + RViz
    │
    └── t=6s: Vision
            └── 启动 AprilTag + 视觉伺服控制器
```

### 7.4 完整系统启动命令

```bash
cd /home/xiang/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jaka_a5 launch/integration.launch.py
```

---

## 八、通信接口

### 8.1 话题 (Topics)

| 话题 | 类型 | 方向 | 频率 | 说明 |
|------|------|------|------|------|
| `/joint_states` | sensor_msgs/JointState | Gazebo → 全局 | 100Hz | 机器人关节状态 |
| `/tag_detections` | apriltag_msgs/AprilTagDetectionArray | AprilTag → | 30Hz | 标签检测结果 |
| `/target_pose` | geometry_msgs/PoseStamped | apriltag_sub → vs_controller | 30Hz | 目标位姿 |
| `/joint_trajectory_controller/trajectory` | trajectory_msgs/JointTrajectory | vs_controller/MoveIt → controller | 10Hz | 轨迹命令 |
| `/camera/image_raw` | sensor_msgs/Image | 相机驱动 → AprilTag | 30Hz | 原始图像 |

### 8.2 服务 (Services)

| 服务 | 类型 | 提供者 | 调用者 | 说明 |
|------|------|--------|--------|------|
| `/spawn_entity` | ros_gz_sim/srv/SpawnEntity | Gazebo | sim.launch.py | 实体spawn |
| `/controller_manager/load_controller` | controller_manager/srv/LoadController | controller_manager | spawner | 加载控制器 |
| `/controller_manager/switch_controller` | controller_manager/srv/SwitchController | controller_manager | spawner | 切换控制器 |

### 8.3 动作 (Actions)

| 动作 | 类型 | 客户端 | 服务器 | 说明 |
|------|------|--------|--------|------|
| `/joint_trajectory_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | move_group / vs_controller | JointTrajectoryController | 执行轨迹 |

---

## 九、关键配置参数

### 9.1 控制器参数

```yaml
# ros2_controllers.yaml
update_rate: 100                    # 控制频率 Hz
gains:                              # PID 位置增益
  joint_1: {p: 100.0, i: 0.0, d: 10.0}
  joint_2: {p: 100.0, i: 0.0, d: 10.0}
  joint_3: {p: 100.0, i: 0.0, d: 10.0}
  joint_4: {p: 100.0, i: 0.0, d: 10.0}
  joint_5: {p: 100.0, i: 0.0, d: 10.0}
  joint_6: {p: 100.0, i: 0.0, d: 10.0}
constraints:
  goal_time: 0.6                   # 目标到达超时
  stopped_velocity_tolerance: 0.05  # 停止容差
```

### 9.2 视觉伺服参数

```yaml
# vs_controller 运行时参数
position_threshold: 0.005           # 5mm - 停止阈值
max_step: 0.02                      # 20mm - 单步最大位移
desired_z: 0.1                      # 100mm - 目标高度
scale: 0.5                          # 控制增益

# apriltag.yaml
tag_family: 36h11                   # 标签家族
tag_ids: [1]                        # 目标标签ID
tag_size: 0.05                      # 5cm - 标签边长
```

### 9.3 仿真参数

```yaml
# battery_station.world
physics:
  max_step_size: 0.001              # 1ms 物理步长
  real_time_factor: 1.0              # 实时因子
  real_time_update_rate: 1000       # 1000Hz 更新率

# 场景
scene:
  ambient: 0.4 0.4 0.4              # 环境光
  background: 0.7 0.7 0.7          # 背景色
  shadows: true                     # 阴影
```

---

## 十、数据流总结

```
┌────────────────────────────────────────────────────────────────────────┐
│                         完整数据流图                                    │
│                                                                        │
│  用户 ──▶ RViz ──▶ MoveIt2 ──▶ Controller ──▶ Gazebo ──▶ 物理 ──▶ 关节 │
│                      │              │                    │            │
│                      │              │                    ▼            │
│                      │              │              JointState         │
│                      │              │                    │            │
│                      │              │              ┌──────┴──────┐     │
│                      │              │              │  状态反馈   │     │
│                      │              │              └─────────────┘     │
│                      │              │                    │            │
│                      │              │              vs_controller       │
│                      │              │                    │            │
│                      │              │              ┌─────┴─────┐     │
│                      │              │              │ /target_pose│    │
│                      │              │              └────────────┘     │
│                      │              │                    │            │
│                      │              │              apriltag_sub       │
│                      │              │                    │            │
│                      │              │              ┌─────┴─────┐     │
│                      │              │              │/tag_detections│   │
│                      │              │              └────────────┘     │
│                      │              │                    │            │
│                      │              │              AprilTag节点      │
│                      │              │                    │            │
│                      │              │              ┌─────┴─────┐     │
│                      │              │              │/camera/image│    │
│                      │              │              └────────────┘     │
│                      │              │                    │            │
│                      └──────────────┴────────────────────┘            │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 十一、扩展点与后续工作

### 11.1 近期扩展

1. **相机仿真集成**
   - 集成 `ros_gz_sim` 相机模型
   - 配置 `/camera/image_raw` → AprilTag 检测

2. **更精确的视觉伺服**
   - 实现基于 Jacobian 的 PBVS 控制
   - 添加姿态控制（当前仅位置）

3. **碰撞检测**
   - 启用 MoveIt2 碰撞检测
   - 添加工作台和电池的碰撞模型

### 11.2 长期优化

1. **力控扩展**
   - 添加力矩传感器仿真
   - 实现混合力/位置控制

2. **多目标规划**
   - 电池更换序列规划
   - 抓取-放置策略

3. **实际机器人接口**
   - JAKA SDK 驱动集成
   - 真实硬件迁移

---

## 附录 A: 文件索引

| 文件路径 | 说明 |
|----------|------|
| `jaka_a5_description/urdf/jaka_a5.robot.xacro` | 机器人本体 URDF |
| `jaka_a5_description/urdf/gazebo.urdf.xacro` | Gazebo 配置 |
| `jaka_a5_gazebo/worlds/battery_station.world` | 仿真世界 |
| `jaka_a5_gazebo/launch/sim.launch.py` | 仿真启动 |
| `jaka_a5_control/config/ros2_controllers.yaml` | 控制器配置 |
| `jaka_a5_control/launch/control.launch.py` | 控制器启动 |
| `jaka_a5_planning/config/jaka_a5.srdf` | MoveIt SRDF |
| `jaka_a5_planning/config/ompl_planning.yaml` | OMPL 配置 |
| `jaka_a5_planning/launch/move_group.launch.py` | MoveIt 启动 |
| `jaka_a5_vision/config/apriltag.yaml` | AprilTag 配置 |
| `jaka_a5_vision/jaka_a5_vision/vs_controller.py` | 视觉伺服控制器 |
| `jaka_a5/launch/integration.launch.py` | 集成启动 |

---

## 附录 B: 依赖包列表

| 包名 | 来源 | 用途 |
|------|------|------|
| ros2 humble | ROS2 | 框架 |
| gazebo_ros2_control | ros-gazebo-plugins | Gazebo-ROS2 桥接 |
| ros_gz_sim | ros-gazebo | Gazebo 启动 |
| controller_manager | ros2_control | 控制器管理 |
| joint_trajectory_controller | ros2_controllers | 轨迹控制 |
| moveit_ros_planning_interface | MoveIt2 | 运动规划 |
| apriltag_ros | apriltag_ros | 标签检测 |
| KDL | orocos_kinematics_dynamics | 运动学求解 |

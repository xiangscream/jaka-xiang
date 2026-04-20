# ROS2 机械臂视觉闭环 — 系统架构设计文档

> 本文档描述 JAKA A5 视觉闭环路径规划系统的软件架构
> **更新日期**: 2026-04-20

---

## 目录

1. [系统概述](#1-系统概述)
2. [架构图](#2-架构图)
3. [模块详细设计](#3-模块详细设计)
4. [数据流图](#4-数据流图)
5. [时序图](#5-时序图)
6. [接口定义](#6-接口定义)
7. [错误处理机制](#7-错误处理机制)

---

## 1. 系统概述

### 1.1 系统目标

实现一个**换电池工作站**中的机械臂视觉闭环路径规划系统，具备以下能力：
- Gazebo 仿真环境中的视觉闭环控制
- AprilTag 视觉检测与位姿估计
- 视觉伺服控制（IBVS/PBVS）
- MoveIt2 运动规划与执行
- ros2_control 控制器管理

### 1.2 技术栈

| 组件 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 22.04 | 操作系统 |
| ROS2 | Humble | 机器人操作系统 |
| Gazebo | Fortress | 仿真引擎 |
| MoveIt2 | Humble | 运动规划 |
| apriltag_ros | - | AprilTag 检测 |
| Python | 3.10+ | 开发语言 |

### 1.3 系统约束

- **实时性**: 视觉伺服控制周期 ≤ 100ms
- **可靠性**: 相机话题可用率 > 95%
- **安全性**: 末端执行速度限制 ≤ 0.3m/s

---

## 2. 架构图

### 2.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              用户层 (User Layer)                              │
│   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                     │
│   │ RViz 可视化  │    │ 规划指令    │    │ 监控面板    │                     │
│   └──────┬──────┘    └──────┬──────┘    └──────┬──────┘                     │
└──────────┼──────────────────┼──────────────────┼────────────────────────────┘
           │                  │                  │
┌──────────▼──────────────────▼──────────────────▼────────────────────────────┐
│                            应用层 (Application Layer)                         │
│   ┌─────────────────────────────────────────────────────────────────────────┐│
│   │                         MoveIt2 Move Group                              ││
│   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                   ││
│   │  │ PlanningScene│  │  Kinematics  │  │   OMPL      │                   ││
│   │  │   Manager    │  │   Solver     │  │  (RRTConnect)│                   ││
│   │  └──────────────┘  └──────────────┘  └──────────────┘                   ││
│   └─────────────────────────────────────────────────────────────────────────┘│
│   ┌────────────────┐  ┌────────────────┐  ┌────────────────┐               │
│   │ apriltag_ros   │  │apriltag_subscriber│ │vs_controller  │               │
│   │  (Tag检测)      │→ │ (位姿提取)       │→ │ (视觉伺服)     │               │
│   └────────────────┘  └────────────────┘  └───────┬────────┘               │
└────────────────────────────────────────────────────┼────────────────────────┘
                                                     │
┌────────────────────────────────────────────────────▼────────────────────────┐
│                            控制层 (Control Layer)                            │
│   ┌─────────────────────────────────────────────────────────────────────────┐│
│   │                     controller_manager                                  ││
│   │  ┌─────────────────────────┐  ┌─────────────────────────────────────┐ ││
│   │  │   joint_state_broadcaster │  │    jaka_a5_arm_controller           │ ││
│   │  │   (/joint_states 广播)    │  │  (JointTrajectoryController)        │ ││
│   │  └─────────────────────────┘  └─────────────────────────────────────┘ ││
│   └─────────────────────────────────────────────────────────────────────────┘│
└────────────────────────────────────────────────────┬────────────────────────┘
                                                     │
┌────────────────────────────────────────────────────▼────────────────────────┐
│                            硬件抽象层 (Hardware Abstraction Layer)           │
│   ┌─────────────────────────┐  ┌─────────────────────────────────────────┐ │
│   │    gz_ros2_control      │  │           Gazebo Sim                    │ │
│   │   (GazeboSimSystem)     │←→│   (Ignition Fortress)                   │ │
│   │                         │  │  ┌─────────┐  ┌─────────┐  ┌───────────┐  │ │
│   │                         │  │  │ 机械臂  │  │ 相机    │  │ 环境模型  │  │ │
│   │                         │  │  │ jaka_a5 │  │ sensor  │  │ 工作台+标签│  │ │
│   └─────────────────────────┘  └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 包结构架构

```
jaka_a5_ws/src/jaka_a5/
│
├── jaka_a5_description/          # 机器人描述层
│   ├── urdf/
│   │   ├── jaka_a5.urdf.xacro     # 主入口（含gazebo配置）
│   │   ├── jaka_a5.robot.xacro    # 机器人运动学模型
│   │   └── gazebo.urdf.xacro      # Gazebo插件配置
│   └── meshes/                    # STL网格文件
│
├── jaka_a5_gazebo/                 # 仿真环境层
│   ├── launch/
│   │   └── sim.launch.py          # Gazebo启动文件
│   ├── worlds/
│   │   └── battery_station.world  # 仿真世界
│   └── models/                    # Gazebo模型
│
├── jaka_a5_control/               # 控制器层
│   ├── launch/
│   │   └── control.launch.py      # 控制器启动
│   └── config/
│       └── ros2_controllers.yaml  # 控制器配置
│
├── jaka_a5_planning/               # 规划层
│   ├── launch/
│   │   └── move_group.launch.py   # MoveIt2启动
│   └── config/
│       ├── jaka_a5.srdf           # 语义描述
│       ├── kinematics.yaml        # 运动学配置
│       └── ompl_planning.yaml     # OMPL配置
│
├── jaka_a5_vision/                 # 视觉层
│   ├── launch/
│   │   └── vs.launch.py           # 视觉节点启动
│   ├── config/
│   │   └── apriltag.yaml          # AprilTag配置
│   └── jaka_a5_vision/
│       ├── apriltag_subscriber.py  # Tag检测订阅
│       └── vs_controller.py        # 视觉伺服控制器
│
└── jaka_a5_bringup/               # 集成层
    └── launch/
        └── integration.launch.py  # 统一启动入口
```

### 2.3 TF 树架构

```
world (0, 0, 0)
    │
    └── base_link (基座中心)
            │
            ├── J1 (关节1)
            │       │
            │       └── J2 (关节2)
            │               │
            │               └── J3 (关节3)
            │                       │
            │                       └── J4 (关节4)
            │                               │
            │                               └── J5 (关节5)
            │                                       │
            │                                       └── J6 (末端法兰)
            │                                               │
            │                                               └── camera_link (相机)
            │                                                       │
            │                                                       └── (相机光心)
            │
            (相机视角) → 观察 → AprilTag (tag_id=1)
```

---

## 3. 模块详细设计

### 3.1 jaka_a5_description 模块

**职责**: 定义机器人的物理结构和 Gazebo 配置

**关键文件**:
- `jaka_a5.robot.xacro`: 定义 6-DOF 机械臂运动学模型
- `gazebo.urdf.xacro`: 定义 Gazebo 插件和传感器

**核心元素**:

| 元素 | 说明 |
|------|------|
| `link` (J1~J6 + base_link) | 机械臂连杆，包含惯性、视觉、碰撞体 |
| `joint` (joint_1~joint_6) | 旋转关节，定义运动轴和限位 |
| `camera_link` | 空 link，作为相机挂载点 |
| `ros2_control` | 声明硬件接口，连接 ros2_control 框架 |

### 3.2 jaka_a5_gazebo 模块

**职责**: 管理 Gazebo 仿真环境和话题桥接

**关键文件**:
- `sim.launch.py`: 启动 Gazebo + robot_state_publisher + 桥接节点
- `battery_station.world`: 定义仿真世界（工作台、电池、AprilTag）

**桥接配置**:

| Gazebo 话题 | ROS2 话题 | 消息类型 |
|------------|----------|----------|
| `/clock` | `/clock` | `rosgraph_msgs/msg/Clock` |
| `/camera/image_raw` | `/camera/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/camera_info` | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` |

### 3.3 jaka_a5_control 模块

**职责**: 管理 ros2_control 控制器生命周期

**关键文件**:
- `control.launch.py`: 启动 controller_manager + spawner
- `ros2_controllers.yaml`: 定义控制器类型和参数

**控制器配置**:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster

jaka_a5_arm_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  ros__parameters:
    joints: [joint_1~joint_6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

### 3.4 jaka_a5_vision 模块

**职责**: 处理视觉检测和控制

**关键文件**:
- `apriltag_subscriber.py`: 订阅 `/tag_detections`，发布 `/target_pose`
- `vs_controller.py`: 视觉伺服控制器，计算并发送关节轨迹

**话题接口**:

| 输入话题 | 输出话题 | 说明 |
|---------|---------|------|
| `/tag_detections` | - | AprilTag 检测结果 |
| `/joint_states` | - | 当前关节状态 |
| - | `/target_pose` | 目标位姿 |
| - | `/jaka_a5_arm_controller/joint_trajectory` | 关节轨迹命令 |

### 3.5 jaka_a5_planning 模块

**职责**: 提供运动规划和执行能力

**关键文件**:
- `move_group.launch.py`: 启动 MoveIt2 move_group
- `jaka_a5.srdf`: 定义规划组和预定义姿态
- `kinematics.yaml`: 配置运动学求解器
- `ompl_planning.yaml`: 配置 OMPL 规划算法

---

## 4. 数据流图

### 4.1 视觉闭环数据流

```
                    ┌─────────────────────────────────────────────────────────────────┐
                    │                     Gazebo Sim (30 Hz)                             │
                    │                                                                         │
                    │    camera_link (TF)                                                  │
                    │         │                                                              │
                    │         ▼                                                              │
                    │    [Camera Sensor]                                                    │
                    │         │                                                              │
                    │         │ /camera/image_raw (+ /camera/camera_info)                   │
                    └─────────┼─────────────────────────────────────────────────────────────┘
                              │
                              │ ros_gz_bridge / ros_gz_image
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│                                    ROS2 话题空间                                             │
│                                                                                             │
│    /camera/image_raw ──────────────────────────┐                                           │
│                                                │                                           │
│                                                ▼                                           │
│                                       [apriltag_ros 节点]                                   │
│                                                │                                           │
│                                                │ /tag_detections                          │
│                                                ▼                                           │
│                                    [apriltag_subscriber 节点]                               │
│                                                │                                           │
│                                                │ /target_pose (PoseStamped)               │
│                           ┌────────────────────┼────────────────────┐                    │
│                           │                    │                    │                     │
│                           ▼                    ▼                    ▼                     │
│                   [vs_controller]       [/joint_states]      [MoveIt2]                    │
│                           │                    │                    │                     │
│                           │                    │                    │                     │
│                           │    ┌───────────────┘                    │                     │
│                           │    │                                    │                     │
│                           ▼    ▼                                    ▼                     │
│                   [JointTrajectory]                          [Trajectory Execution]       │
│                           │                                                         │
│                           ▼                                                         │
│                   [controller_manager]                                               │
│                           │                                                         │
│                           ▼                                                         │
│                   [GazeboSimSystem]                                                  │
│                           │                                                         │
│                           ▼                                                         │
│                   [Gazebo Sim 物理引擎]                                               │
│                           │                                                         │
│                           │ (机械臂关节运动)                                            │
│                           ▼                                                         │
│                    [jaka_a5 机械臂模型]                                                │
│                                                                                             │
└─────────────────────────────────────────────────────────────────────────────────────────────┘
                              │
                              │ (循环)
                              │
                              └─── 回到起点
```

### 4.2 状态机转换

```
                    ┌─────────────┐
                    │   STANDBY   │  初始状态，等待启动
                    └──────┬──────┘
                           │
                           ▼
              ┌────────────────────────┐
              │      INITIALIZING      │
              │  Gazebo启动，控制器激活 │
              └───────────┬────────────┘
                          │ 控制器就绪
                          ▼
              ┌────────────────────────┐
              │        HOMING           │  机械臂归位
              │   MoveIt "home" 姿态   │
              └───────────┬────────────┘
                          │ 归位完成
                          ▼
              ┌────────────────────────┐
              │      SEARCHING          │  搜索 AprilTag
              │   camera扫描工作区域    │
              └───────────┬────────────┘
                          │ 检测到 tag_id=1
                          ▼
              ┌────────────────────────┐
              │      TRACKING          │  视觉跟踪
              │ vs_controller 计算误差  │
              └───────────┬────────────┘
                          │ 误差 < 阈值
                          ▼
              ┌────────────────────────┐
              │      COMPLETED          │  目标到达
              │    停止机械臂运动       │
              └───────────┬────────────┘
                          │ 新目标
                          ▼
              ┌────────────────────────┐
              │      ERROR              │  异常处理
              │ 相机丢失/控制器错误     │
              └────────────────────────┘
```

---

## 5. 时序图

### 5.1 系统启动时序

```
用户                    integration.launch.py          sim.launch.py        control.launch.py      move_group.launch.py      vs.launch.py
 │                            │                          │                     │                        │                       │
 │  ros2 launch ...           │                          │                     │                        │                       │
 │───────────────────────────>│                          │                     │                        │                       │
 │                            │                          │                     │                        │                       │
 │                            │ IncludeLaunchDescription  │                     │                        │                       │
 │                            │─────────────────────────>│                     │                        │                       │
 │                            │                          │                     │                        │                       │
 │                            │                          │ 启动 Gazebo Sim     │                        │                       │
 │                            │                          │<────────────────────│                        │                       │
 │                            │                          │                     │                        │                       │
 │                            │                          │  启动 robot_state_publisher                     │                       │
 │                            │                          │─────────────────────>│                        │                       │
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │  延迟 3s spawn 机器人                        │                       │
 │                            │                          │                     │────────────────────────>│                       │
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │  TimerAction(4s)      │                       │
 │                            │                          │                     │────────────────────────>│                       │
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │                        │  启动 controller_manager
 │                            │                          │                     │                        │<──────────────────────│
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │                        │  启动 joint_state_broadcaster
 │                            │                          │                     │                        │────────────────────────>│
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │                        │  启动 arm_controller
 │                            │                          │                     │                        │────────────────────────>│
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │  TimerAction(6s)      │                       │
 │                            │                          │                     │────────────────────────│                       │
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │                        │        启动 move_group
 │                            │                          │                     │                        │<──────────────────────│
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │                        │  TimerAction(8s)      │
 │                            │                          │                     │                        │────────────────────────│
 │                            │                          │                     │                        │                       │
 │                            │                          │                     │                        │          启动 apriltag + vs_controller
 │                            │                          │                     │                        │<──────────────────────│
 │                            │                          │                     │                        │                       │
 │                            │ [系统启动完成]           │                     │                        │                       │
 │<──────────────────────────│                          │                     │                        │                       │
```

### 5.2 视觉伺服控制周期

```
相机            apriltag_ros        apriltag_subscriber      vs_controller     controller_manager      机械臂
  │                   │                      │                    │                     │                  │
  │ (30Hz) /camera/   │                      │                    │                     │                  │
  │<───────────────────│                      │                    │                     │                  │
  │                   │                      │                    │                     │                  │
  │                   │ /tag_detections      │                    │                     │                  │
  │                   │────────────────────>│                    │                     │                  │
  │                   │                      │                    │                     │                  │
  │                   │                      │ /target_pose       │                     │                  │
  │                   │                      │───────────────────>│                     │                  │
  │                   │                      │                    │                     │                  │
  │                   │                      │                    │ /joint_states      │                  │
  │                   │                      │                    │<───────────────────│                  │
  │                   │                      │                    │                     │                  │
  │                   │                      │                    │ 计算误差和 delta   │                  │
  │                   │                      │                    │                     │                  │
  │                   │                      │                    │ /joint_trajectory  │                  │
  │                   │                      │                    │───────────────────>│                  │
  │                   │                      │                    │                     │                  │
  │                   │                      │                    │                     │  发送位置指令    │
  │                   │                      │                    │                     │──────────────────>│
  │                   │                      │                    │                     │                  │
  │                   │                      │                    │                     │        机械臂运动 │
  │                   │                      │                    │                     │                  │
  │ (回到相机)         │                      │                    │                     │                  │
```

---

## 6. 接口定义

### 6.1 话题接口

| 话题名 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|----------|--------|--------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo 相机 | apriltag_ros | 原始图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo 相机 | apriltag_ros | 相机内参 |
| `/tag_detections` | `apriltag_msgs/AprilTagDetectionArray` | apriltag_ros | apriltag_subscriber | 检测结果 |
| `/target_pose` | `geometry_msgs/PoseStamped` | apriltag_subscriber | vs_controller | 目标位姿 |
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | vs_controller, MoveIt | 关节状态 |
| `/jaka_a5_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | vs_controller | joint_trajectory_controller | 轨迹命令 |

### 6.2 服务接口

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/spawn_entity` | `ros_gz_sim/srv/SpawnEntity` | 在 Gazebo 中生成模型 |
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | 列出所有控制器 |
| `/controller_manager/load_controller` | `controller_manager_msgs/LoadController` | 加载控制器 |
| `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | 切换控制器 |

### 6.3 参数接口

**vs_controller 参数**:
```yaml
visual_servo_controller:
  ros__parameters:
    position_threshold: 0.005    # 位置误差阈值 (5mm)
    max_step: 0.02               # 最大步长 (20mm)
    desired_z: 0.1               # 目标高度 (100mm)
    scale: 0.5                    # 控制增益
```

**apriltag 参数**:
```yaml
apriltag:
  ros__parameters:
    tag_family: 36h11
    tag_ids: [1]
    tag_size: 0.05               # 5cm
    publish_tf: true
```

---

## 7. 错误处理机制

### 7.1 错误类型与处理

| 错误类型 | 错误描述 | 处理方式 |
|----------|----------|----------|
| `CAMERA_NO_DATA` | 相机话题无数据 | 切换到固定位置模式，报警 |
| `TAG_NOT_DETECTED` | AprilTag 检测不到 | 进入 SEARCHING 状态，旋转机械臂 |
| `CONTROLLER_INACTIVE` | 控制器未激活 | 重新 spawn controller |
| `IK_FAILED` | 逆运动学求解失败 | 触发重规划，使用 RRTConnect |
| `TRAJECTORY_ABORTED` | 轨迹执行中止 | 进入 ERROR 状态，等待人工干预 |

### 7.2 恢复策略

```python
class ErrorRecovery:
    def __init__(self):
        self.error_count = {}
        self.max_retries = 3

    def handle_error(self, error_type):
        if error_type == 'TAG_NOT_DETECTED':
            # 旋转机械臂扫描
            self.rotate_search_pattern()
        elif error_type == 'CONTROLLER_INACTIVE':
            # 重新加载控制器
            self.reload_controller()
        elif error_type == 'IK_FAILED':
            # 使用 RRTConnect 重规划
            self.replan_with_rrt()
```

### 7.3 看门狗机制

```python
class Watchdog:
    def __init__(self, timeout=1.0):
        self.timeout = timeout
        self.last_update = time.time()

    def check(self):
        if time.time() - self.last_update > self.timeout:
            raise WatchdogTimeout("Topic update timeout")
        self.last_update = time.time()
```

---

## 附录：类图

### A.1 视觉节点类图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                    rclpy.Node                                      │
│  ┌─────────────────────────────────────────────────────────────────────────────┐ │
│  │ VisualServoController                                                       │ │
│  ├─────────────────────────────────────────────────────────────────────────────┤ │
│  │ - target_pose: PoseStamped                                                   │ │
│  │ - current_joint_positions: List[float]                                      │ │
│  │ - joint_name_to_idx: Dict                                                   │ │
│  ├─────────────────────────────────────────────────────────────────────────────┤ │
│  │ + __init__()                                                                │ │
│  │ + target_callback(msg: PoseStamped)                                          │ │
│  │ + joint_states_callback(msg: JointState)                                     │ │
│  │ + compute_delta() -> List[float]                                            │ │
│  │ + send_servo_command()                                                      │ │
│  │ + send_stop()                                                               │ │
│  └─────────────────────────────────────────────────────────────────────────────┘ │
│                                         ▲                                         │
│                                         │ 继承                                     │
┌─────────────────────────────────────────┼─────────────────────────────────────────┐
│  ┌──────────────────────────────────────┴──────────────────────────────────────┐  │
│  │ AprilTagSubscriber                                                                │  │
│  ├──────────────────────────────────────────────────────────────────────────────┤  │
│  │ - target_tag_id: int                                                           │  │
│  ├──────────────────────────────────────────────────────────────────────────────┤  │
│  │ + tag_callback(msg: AprilTagDetectionArray)                                  │  │
│  └──────────────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### A.2 控制器类图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           ros2_control 架构                                      │
│                                                                                  │
│  ┌────────────────────────┐         ┌────────────────────────┐              │
│  │   ControllerManager      │         │   HardwareInterface     │              │
│  │                         │─────────>│   (GazeboSimSystem)     │              │
│  └───────────┬──────────────┘         └───────────┬──────────────┘              │
│              │                                  │                              │
│              │  管理                              │  实现                         │
│              ▼                                  ▼                              │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                           Controllers                                      │  │
│  │  ┌─────────────────────────┐    ┌──────────────────────────────┐           │  │
│  │  │  JointStateBroadcaster  │    │   JointTrajectoryController  │           │  │
│  │  │  (广播关节状态)         │    │   (轨迹跟踪控制)            │           │  │
│  │  └─────────────────────────┘    └──────────────────────────────┘           │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 附录：版本历史

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| 1.0 | 2026-04-20 | 初始版本 |

---

*本文档由 Claude (蒠芷) 生成*
*GitHub: https://github.com/xiangscream/jaka-xiang*
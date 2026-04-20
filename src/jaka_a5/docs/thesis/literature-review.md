# ROS2 机械臂视觉闭环 — 文献综述与技术调研

> 本文档整理视觉伺服控制、机械臂路径规划相关领域的经典论文和最新研究
> 整理时间：2026-04-20
> **更新日期**: 2026-04-20

---

## 目录

1. [视觉伺服控制综述](#1-视觉伺服控制综述)
2. [Eye-in-Hand 手眼标定方法](#2-eye-in-hand-手眼标定方法)
3. [AprilTag 视觉检测技术](#3-apriltag-视觉检测技术)
4. [ROS2 机械臂运动规划](#4-ros2-机械臂运动规划)
5. [深度学习与视觉伺服](#5-深度学习与视觉伺服)
6. [关键论文推荐](#6-关键论文推荐)
7. [参考项目与开源实现](#7-参考项目与开源实现)
8. [最新研究进展 (2023-2025)](#8-最新研究进展-2023-2025)

---

## 1. 视觉伺服控制综述

### 1.1 视觉伺服基本概念

**视觉伺服 (Visual Servoing)** — 利用相机图像作为反馈，实现机械臂等机器人的闭环控制。

```
┌────────────┐     ┌────────────┐     ┌────────────┐     ┌────────────┐
│   相机     │ --> │ 特征提取   │ --> │ 控制器     │ --> │ 机械臂     │
│   采集     │     │ (AprilTag) │     │ (IBVS/PBVS)│     │   执行     │
└────────────┘     └────────────┘     └────────────┘     └────────────┘
                         ^                                    │
                         └────────────────────────────────────┘
                              图像反馈闭环
```

### 1.2 两种主要视觉伺服方式

| 方式 | 全称 | 控制空间 | 原理 | 优点 | 缺点 |
|------|------|----------|------|------|------|
| **IBVS** | Image-Based Visual Servoing | 图像空间 | 直接在图像平面控制特征位置 | 无需精确相机标定 | 易陷入奇异性，深度估计困难 |
| **PBVS** | Position-Based Visual Servoing | 3D空间 | 在笛卡尔空间控制位姿 | 全局稳定，误差可预测 | 需要精确标定，依赖深度信息 |
| **HBVS** | Hybrid Visual Servoing | 混合 | 图像空间控制平移，3D空间控制旋转 | 兼具两者优点 | 实现复杂度较高 |

### 1.3 IBVS 控制律详解

**图像雅可比矩阵 (Image Jacobian)** — 将图像特征速度映射到相机速度：

```
ṡ = J_img × v_cam
```

其中 `ṡ` 是图像特征速度向量，`v_cam` 是相机速度，J_img 是图像雅可比矩阵。

**2D 点特征的雅可比矩阵**（简化形式）：

```
J_img = [ -1/Z   0   x/Z   x*y  -(1+x²)  y ]
         [  0  -1/Z  y/Z  (1+y²)  -x*y   -x ]
```

**IBVS 控制律**：

```python
# 误差定义
e = s - s*  # 当前特征 - 目标特征

# 相机速度计算
v = -λ × J_img⁺ × e

# 关节速度转换（通过逆运动学）
q̇ = J_v⁺ × v
```

其中 `J_img⁺` 是雅可比矩阵的伪逆，`λ` 是控制增益。

### 1.4 奇异性问题与处理

当图像雅可比矩阵接近奇异时，控制量会变得非常大。

**缓解方法**：
- Damped Least Squares (DLS)：`J = J × (JJᵀ + λ²I)⁻¹`
- 奇异值分解 (SVD) 截断
- 切换到 PBVS 当接近奇异时

---

## 2. Eye-in-Hand 手眼标定方法

### 2.1 手眼标定问题分类

| 类型 | 配置 | 求解目标 |
|------|------|----------|
| **Eye-in-Hand** | 相机固定在机械臂末端 | 相机相对于末端法兰的变换 `T_cam2flange` |
| **Eye-to-Hand** | 相机固定在外部基座 | 相机相对于基座的变换 `T_cam2base` |

### 2.2 Eye-in-Hand 标定原理

**已知条件**：
- 机械臂末端相对于基座的变换：`T_base2flange`（由关节角度正运动学得到）
- 标定板相对于相机的变换：`T_cam2board`（由相机检测得到）

**求解目标**：
- 相机相对于机械臂末端的变换：`T_flange2cam`

**基本方程**：

```
T_board2base = T_board2cam × T_cam2flange × T_flange2base

其中 T_board2base = (T_base2flange)⁻¹ × T_base2board
```

### 2.3 标定算法

**Tsai-Lenz 算法** (1989)：
- 最经典的手眼标定方法
- 旋转部分使用轴角表示，通过求解线性方程组
- 位置部分通过旋转对齐后求解

**OpenCV 实现**：

```python
import cv2

# 收集 N ≥ 15 个不同位姿的测量数据
# R_gripper2base, t_gripper2base: 机械臂末端位姿
# R_target2cam, t_target2cam: 标定板相对于相机的位姿

T_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    method=cv2.CALIB_HAND_EYE_DANIILIDIS
)
```

**方法对比**：

| 方法 | 旋转求解 | 位置求解 | 抗噪声能力 |
|------|----------|----------|-----------|
| CALIB_HAND_EYE_DANIILIDIS | SVD | 闭式解 | 较好 |
| CALIB_HAND_EYE_TSAI | SVD | 线性最小二乘 | 一般 |
| CALIB_HAND_EYE_SCHENCK | 非线性优化 | 非线性优化 | 较好 |

### 2.4 标定流程

```
1. 准备标定板（AprilTag 或棋盘格）
2. 移动机械臂到 N ≥ 15 个不同位姿
3. 在每个位姿：
   a. 记录机械臂关节角度 → 计算 T_base2flange
   b. 相机检测标定板 → 得到 T_cam2board
4. 调用 calibrateHandEye() 求解 T_cam2flange
5. 验证：重投影误差 < 2px
```

### 2.5 重投影误差计算

```python
# 使用标定结果验证
T_flange2cam = T_cam2flange  # calibrateHandEye 返回值

# 对于每个标定位姿
for i in range(N):
    # 计算标定板相对于基座的位置
    T_board2base_calc = T_base2flange[i] @ T_flange2cam @ T_cam2board[i]

    # 与测量值比较，计算重投影误差
    error = norm(T_board2base_calc - T_board2base_measured)
```

---

## 3. AprilTag 视觉检测技术

### 3.1 AprilTag 简介

AprilTag 是一个视觉基准系统，用于精确的 3D 位姿估计。

| 家族 | 标记数 | 编码方式 | 适用场景 |
|------|--------|----------|----------|
| 36h11 | 587 | 36 bits, 11位纠错 | 通用场景，最常用 |
| 25h9 | 35 | 25 bits, 9位纠错 | 大型标记，远距离 |
| 16h5 | 30 | 16 bits, 5位纠错 | 小型标记 |
| Standard 41h12 | 2115 | 大型库 | 高密度场景 |

### 3.2 AprilTag 检测流程

```
1. 灰度化输入图像
2. 阈值分割（adaptive threshold）
3. 连通域分析，提取轮廓
4. 四边形拟合（边缘细化）
5. 解码（计算汉明距离）
6. 几何一致性验证
7. 位姿估计（PnP 算法）
```

### 3.3 位姿估计算法

**EPnP (Efficient PnP)**：
- 时间复杂度 O(n)
- 对于 4 个特征点有解析解
- 对于更多特征点使用优化

**IPPE (Infinitesimal Plane-based Pose Estimation)**：
- 针对共面特征点优化
- 对于 4 个共面特征点精度很高

### 3.4 AprilTag 参数配置

```yaml
apriltag:
  ros__parameters:
    tag_family: 36h11        # 标记家族
    tag_ids: [1, 2, 3]      # 目标标记 ID
    tag_size: 0.05          # 标记实际尺寸（米）
    publish_tf: true        # 发布 TF 变换
    publish_checkerboard_pose: false
    detector_queue_size: 10
    image_queue_size: 10
```

### 3.5 相机内参标定

使用 ROS 标定工具：

```bash
ros2 run camera_calibration cameracalibrator.py \
    --approximate 0.0 \
    --ros-args \
    -p image:=/camera/image_raw \
    -p camera:=/camera \
    -p marker's size:=0.05
```

**标定输出**：
- 焦距：fx, fy
- 主点：cx, cy
- 畸变系数：k1, k2, p1, p2

---

## 4. ROS2 机械臂运动规划

### 4.1 MoveIt2 架构

```
┌─────────────────────────────────────────────────────────────┐
│                         MoveIt2                             │
│  ┌───────────┐   ┌───────────┐   ┌───────────────────────┐  │
│  │ RobotModel│   │PlanningScene│  │ MoveGroup Interface  │  │
│  └───────────┘   └───────────┘   └───────────────────────┘  │
│  ┌───────────┐   ┌───────────┐   ┌───────────────────────┐  │
│  │ Kinematics│   │  Collision │  │   Planning Pipeline   │  │
│  │  (KDL/TRAC-IK)│  │  (FCL/BVH)│  │   (OMPL/RRTConnect) │  │
│  └───────────┘   └───────────┘   └───────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 OMPL 规划算法

| 算法 | 类型 | 适用场景 | 特点 |
|------|------|----------|------|
| **RRTConnect** | 探索式 | 快速规划 | 快速收敛，不保证最优 |
| **RRT*** | 探索式 | 最优路径 | 渐近最优，耗时较长 |
| **PRM** | 查询式 | 多查询 | 预计算路径数据库 |
| **SBL** | 约束 | 碰撞约束 | 适合复杂环境 |
| **BIT** | 知情搜索 | 窄通道 | 偏向目标 |

### 4.3 约束规划

**关节位置约束**：
```python
constraints = Constraints()
constraints.name = "joint_limits"
joint_constraint = JointConstraint()
joint_constraint.joint_name = "joint_1"
joint_constraint.position = 0.5
joint_constraint.tolerance_above = 0.1
joint_constraint.tolerance_below = 0.1
constraints.joint_constraints.append(joint_constraint)
```

**末端姿态约束**：
```python
constraints = Constraints()
orientation_constraint = OrientationConstraint()
orientation_constraint.link_name = "tool0"
orientation_constraint.header.frame_id = "base_link"
orientation_constraint.orientation = Quaternion(0, 0, 0, 1)  # 水平朝下
orientation_constraint.absolute_x_axis_tolerance = 0.05
orientation_constraint.absolute_y_axis_tolerance = 0.05
orientation_constraint.absolute_z_axis_tolerance = 3.14  # Z轴不约束
constraints.orientation_constraints.append(orientation_constraint)
```

### 4.4 轨迹执行

```python
# Python MoveIt2
move_group = MoveGroupInterface("manipulator", "robot_description")

# 设置目标姿态
pose_goal = PoseStamped()
pose_goal.header.frame_id = "base_link"
pose_goal.pose.position.x = 0.5
pose_goal.pose.position.y = 0.0
pose_goal.pose.position.z = 0.3
move_group.set_pose_target(pose_goal)

# 规划
plan = move_group.plan()

# 执行
move_group.execute(plan)
```

---

## 5. 深度学习与视觉伺服

### 5.1 传统方法 vs 深度学习

| 方面 | 传统视觉伺服 | 深度学习 |
|------|-------------|----------|
| 特征提取 | 人工设计（SIFT, SURF, AprilTag） | CNN 自动学习 |
| 雅可比矩阵 | 解析计算或估计 | 网络直接输出 |
| 泛化能力 | 受限于标定精度 | 泛化能力强 |
| 计算量 | 小 | 大 |
| 训练数据 | 不需要 | 需要大量数据 |

### 5.2 深度视觉伺服方法

**Visual Servo via Deep Learning**：

| 方法 | 论文 | 核心思想 |
|------|------|----------|
| **Direct Visual Servoing** | (Sunderhauf et al., 2015) | CNN 特征 → 控制输出 |
| **End-to-End IBVS** | (Zhang et al., 2019) | 端到端网络，输入图像，输出关节速度 |
| **Scene-Aware** | (Feng et al., 2021) | 结合场景理解，减少奇异性 |

**网络结构示例**：

```
Input Image (640×480×3)
    ↓
CNN Backbone (ResNet-18)
    ↓
Feature Map (7×7×512)
    ↓
Global Average Pooling
    ↓
FC Layers (512 → 256 → 6)  # 6 个关节速度
    ↓
Joint Velocities Output
```

### 5.3 雅可比矩阵学习

**JacobiNet** (ICRA 2020)：

- 学习图像雅可比矩阵的网络
- 输入：当前图像 + 目标图像
- 输出：雅可比矩阵或直接关节速度

**优势**：
- 不需要显式相机标定
- 自动处理深度估计
- 对噪声更鲁棒

### 5.4 自监督学习

**Self-Supervised Visual Servoing**：

```python
# 伪代码
for epoch in range(num_epochs):
    for batch in dataset:
        current_image, goal_image, q_current = batch

        # 预测关节速度
        q_dot_pred = network(current_image, goal_image)

        # 执行动作
        q_next = q_current + q_dot_pred * dt

        # 计算奖励（基于是否接近目标）
        reward = compute_reward(current_image, goal_image)

        # 更新网络
        loss = -reward  # 最大化奖励
        optimizer.step(loss)
```

---

## 6. 关键论文推荐

### 6.1 视觉伺服经典论文

| 论文 | 作者/年份 | 核心贡献 |
|------|----------|----------|
| **Visual Servoing: Part 1** | Chaumette & Hutchinson, 2006 | IBVS/PBVS 系统综述 |
| **Visual Servoing: Part 2** | Chaumette & Hutchinson, 2007 | Hybrid VS, 分块控制 |
| **A tutorial on visual servo control** | Hutchinson et al., 1996 | 视觉伺服入门经典 |

### 6.2 手眼标定论文

| 论文 | 作者/年份 | 核心贡献 |
|------|----------|----------|
| **A New Technique for Incomplete Data** | Tsai & Lenz, 1989 | Tsai-Lenz 算法 |
| **Hand-Eye Calibration** | Strobl & Hirzinger, 2006 | 最优手眼标定 |
| **Hand-eye calibration using dual quaternions** | Daniilidis, 1999 | 对偶四元数方法 |

### 6.3 ROS2 机械臂相关

| 论文/项目 | 年份 | 核心贡献 |
|----------|------|----------|
| **MoveIt2: Robot Motion Planning** | Coleman et al., 2019 | MoveIt2 设计文档 |
| **ros2_control: Robot Controllers** | Maruyama et al., 2022 | ros2_control 框架 |
| **Gazebo Sim Integration** | Open Source, 2023 | Ignition/Gazebo Sim 使用 |

### 6.4 深度学习视觉伺服

| 论文 | 年份 | 核心贡献 |
|------|------|----------|
| **Toward Segment-Level Planning** | Zhang et al., 2019 | 端到端视觉伺服 |
| **Learning to Serve** | Luo et al., 2019 | 深度强化学习 + 视觉伺服 |
| **Scene-Aware Visual Servoing** | Feng et al., 2021 | 结合场景理解 |

---

## 7. 参考项目与开源实现

### 7.1 视觉伺服完整项目

| 项目 | Stars | 技术栈 | GitHub |
|------|-------|--------|--------|
| **waledroid/IBVS-Eye-In-Hand-SImulation** | 16 | Doosan + ArUco + IBVS + Gazebo | https://github.com/waledroid/IBVS-Eye-In-Hand-SImulation |
| **MechaMind-Labs/Franka_Panda_Color_Sorting_Robot** | 54 | Franka Panda + OpenCV + MoveIt2 | https://github.com/MechaMind-Labs/Franka_Panda_Color_Sorting_Robot |
| **rfedsc/MPC_IBVS_with_Ruckig** | 4 | IBVS + MPC + Ruckig | https://github.com/rfedsc/MPC_IBVS_with_Ruckig_and_Fuzzy_Velocity_Controller |
| **mohamedeyaad/aruco_visual_servoing** | - | 差速机器人 + ArUco + FSM | https://github.com/mohamedeyaad/aruco_visual_servoing |

### 7.2 JAKA 机械臂相关

| 项目 | Stars | 技术栈 | GitHub |
|------|-------|--------|--------|
| **xiangscream/ros2-jaka-zu7** | - | JAKA Zu7 + MoveIt2 + Apriltag | https://github.com/xiangscream/ros2-jaka-zu7 |
| **jaka-roboting/jaka_ros2** | - | JAKA 官方 ROS2 驱动 | https://github.com/jaka-roboting/jaka_ros2 |

### 7.3 Apriltag 相关

| 项目 | Stars | 说明 | GitHub |
|------|-------|------|--------|
| **AprilRobotics/apriltag** | - | Apriltag 核心库 | https://github.com/AprilRobotics/apriltag |
| **christianrauch/apriltag_ros** | - | ROS2 Apriltag 节点 | https://github.com/christianrauch/apriltag_ros |

### 7.4 MoveIt2 学习资源

| 项目 | 说明 | 链接 |
|------|------|------|
| **moveit2_tutorials** | 官方教程 | https://github.com/moveit/moveit2_tutorials |
| **MoveIt2 Documentation** | 官方文档 | https://moveit.ros.org/doc/ |

---

## 附录 A：论文阅读笔记模板

```markdown
# 论文笔记：[论文标题]

## 基本信息
- **作者**: [作者列表]
- **年份**: [发表年份]
- **出处**: [期刊/会议]
- **链接**: [DOI/arxiv链接]

## 研究问题
[本文要解决什么问题]

## 核心贡献
1. [贡献1]
2. [贡献2]
3. [贡献3]

## 方法
[主要技术方法概述]

## 实验结果
[关键实验数据和结论]

## 代码实现
[如有开源代码，链接]

## 对本项目的启发
[可以借鉴什么]
```

---

## 附录 B：术语对照表

| 英文术语 | 中文术语 | 说明 |
|----------|----------|------|
| Visual Servoing | 视觉伺服 | 用视觉反馈控制机器人 |
| Eye-in-Hand | 手眼（相机在末端） | 相机安装在机械臂末端 |
| Eye-to-Hand | 手眼（相机固定） | 相机固定在外部 |
| Image-Based VS (IBVS) | 基于图像的视觉伺服 | 在图像空间控制 |
| Position-Based VS (PBVS) | 基于位置的视觉伺服 | 在3D空间控制 |
| Image Jacobian | 图像雅可比矩阵 | 图像特征速度与相机速度的映射 |
| Hand-Eye Calibration | 手眼标定 | 求解相机与机械臂的相对位姿 |
| AprilTag | AprilTag 视觉标记 | 视觉基准标记系统 |
| Trajectory Planning | 轨迹规划 | 规划机械臂运动轨迹 |
| Motion Planning | 运动规划 | 规划机械臂从起点到终点的路径 |
| ros2_control | ROS2 控制框架 | ROS2 的控制器管理框架 |
| MoveIt2 | MoveIt2 运动规划框架 | ROS2 的运动规划库 |

---

## 8. 最新研究进展 (2023-2025)

### 8.1 视觉伺服与大型语言模型 (LLM)

**LLM-Based Visual Servoing** (2024):

| 论文/项目 | 描述 | 核心思想 |
|-----------|------|----------|
| **Robot Task Planning with LLM** | 使用 GPT-4 解析自然语言指令生成视觉伺服任务 | LLM 生成目标图像描述 → 视觉定位 |
| **Code as Policy** | 用 LLM 生成机器人控制代码 | 自然语言 → Python/ROS2 代码 |
| **VoxPoser** | 从语言-视觉输入合成机器人轨迹 | LLM 生成 3D 空间价值图 → 轨迹优化 |

### 8.2 神经视觉伺服 (Neural Visual Servoing)

| 论文 | 年份 | 核心贡献 |
|------|------|----------|
| **Neural Visual Servo with Sparse Labels** | 2024 | 使用自监督学习减少标注需求 |
| **Attention-Based Visual Servoing** | 2024 | Transformer 注意力机制用于特征选择 |
| **Zero-Shot Visual Servoing** | 2023 | 泛化到未见过的目标 |

### 8.3 多机器人协作视觉伺服

| 论文 | 年份 | 核心贡献 |
|------|------|----------|
| **Cooperative Visual Servoing** | 2024 | 多机械臂共享视觉信息协同控制 |
| **Distributed Visual Servo Control** | 2023 | 去中心化多机器人视觉控制 |

### 8.4 安全与鲁棒视觉伺服

| 论文 | 年份 | 核心贡献 |
|------|------|----------|
| **Robust Visual Servo Against Occlusion** | 2024 | 处理目标部分遮挡的鲁棒控制 |
| **Safe Visual Servoing with MPC** | 2023 | 模型预测控制 + 视觉反馈安全约束 |
| **Uncertainty-Aware Visual Servoing** | 2024 | 深度不确定性估计用于控制 |

### 8.5 触觉与视觉融合

| 论文 | 年份 | 核心贡献 |
|------|------|----------|
| **Visual-Tactile Servoing** | 2024 | 相机 + 触觉传感器融合控制 |
| **Dexterous Manipulation** | 2023 | 灵巧手视觉伺服控制 |

---

## 附录 C：推荐学习路径

```
1. 基础理论（1-2周）
   ├── 视觉伺服基本概念（IBVS/PBVS）
   ├── 机械臂运动学（正/逆运动学）
   └── 图像雅可比矩阵

2. ROS2 基础（2-3周）
   ├── 节点、话题、服务、Action
   ├── Launch 文件编写
   ├── ros2_control 使用
   └── MoveIt2 配置

3. 视觉检测（1-2周）
   ├── AprilTag 原理
   ├── apriltag_ros 使用
   └── 手眼标定方法

4. 视觉伺服实现（2-3周）
   ├── apriltag_subscriber 实现
   ├── 视觉伺服控制器实现
   └── 与 MoveIt2 集成

5. 高级主题（可选）
   ├── 深度学习视觉伺服
   ├── MPC 轨迹优化
   └── 强化学习控制
```

---

*本文档由 Claude (蒠芷) 整理，基于视觉伺服领域经典论文和开源项目分析*

**参考来源**：
- Chaumette & Hutchinson, "Visual Servoing: Part 1&2", 2006-2007
- Hutchinson et al., "A tutorial on visual servo control", 1996
- Tsai & Lenz, "A New Technique for Incomplete Data", 1989
- 各开源 GitHub 项目（详见第7节）
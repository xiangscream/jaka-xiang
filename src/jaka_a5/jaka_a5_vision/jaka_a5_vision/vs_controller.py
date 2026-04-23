import math
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from jaka_a5_vision.experiment_logger import CsvExperimentLogger


TransformData = Tuple[List[List[float]], List[float]]


class ServoState(str, Enum):
    S0_STANDBY = 's0_standby'
    S1_SIDE_APPROACH = 's1_side_approach'
    S2_ALIGN = 's2_align'
    S3_CLAMP = 's3_clamp'
    S4_EXTRACT = 's4_extract'
    S5_SAFE_EXIT = 's5_safe_exit'
    S6_PLACE_OLD_BATTERY = 's6_place_old_battery'
    S7_TOOL_RESET = 's7_tool_reset'
    S8_GRASP_NEW_BATTERY = 's8_grasp_new_battery'
    S9_MOVE_TO_DOCK = 's9_move_to_dock'
    S10_INSERT_NEW_BATTERY = 's10_insert_new_battery'
    S11_RESET = 's11_reset'


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def quaternion_to_matrix(quaternion: Quaternion) -> List[List[float]]:
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def matrix_to_quaternion(matrix: List[List[float]]) -> Quaternion:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2][1] - matrix[1][2]) / scale
        qy = (matrix[0][2] - matrix[2][0]) / scale
        qz = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        qw = (matrix[2][1] - matrix[1][2]) / scale
        qx = 0.25 * scale
        qy = (matrix[0][1] + matrix[1][0]) / scale
        qz = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        qw = (matrix[0][2] - matrix[2][0]) / scale
        qx = (matrix[0][1] + matrix[1][0]) / scale
        qy = 0.25 * scale
        qz = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        qw = (matrix[1][0] - matrix[0][1]) / scale
        qx = (matrix[0][2] + matrix[2][0]) / scale
        qy = (matrix[1][2] + matrix[2][1]) / scale
        qz = 0.25 * scale

    quat = Quaternion()
    quat.x = qx
    quat.y = qy
    quat.z = qz
    quat.w = qw
    return quat


def make_transform(rotation: List[List[float]], translation: List[float]) -> TransformData:
    return rotation, translation


def transform_from_pose(pose: Pose) -> TransformData:
    return make_transform(
        quaternion_to_matrix(pose.orientation),
        [pose.position.x, pose.position.y, pose.position.z],
    )


def pose_from_transform(transform: TransformData) -> Pose:
    rotation, translation = transform
    pose = Pose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation = matrix_to_quaternion(rotation)
    return pose


def matmul3(lhs: List[List[float]], rhs: List[List[float]]) -> List[List[float]]:
    return [
        [
            lhs[row][0] * rhs[0][col]
            + lhs[row][1] * rhs[1][col]
            + lhs[row][2] * rhs[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def apply_rotation(rotation: List[List[float]], vector: List[float]) -> List[float]:
    return [
        rotation[0][0] * vector[0] + rotation[0][1] * vector[1] + rotation[0][2] * vector[2],
        rotation[1][0] * vector[0] + rotation[1][1] * vector[1] + rotation[1][2] * vector[2],
        rotation[2][0] * vector[0] + rotation[2][1] * vector[1] + rotation[2][2] * vector[2],
    ]


def add_vectors(lhs: List[float], rhs: List[float]) -> List[float]:
    return [lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]]


def subtract_vectors(lhs: List[float], rhs: List[float]) -> List[float]:
    return [lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]]


def transpose3(matrix: List[List[float]]) -> List[List[float]]:
    return [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]


def compose_transforms(lhs: TransformData, rhs: TransformData) -> TransformData:
    lhs_rotation, lhs_translation = lhs
    rhs_rotation, rhs_translation = rhs
    rotation = matmul3(lhs_rotation, rhs_rotation)
    translation = add_vectors(apply_rotation(lhs_rotation, rhs_translation), lhs_translation)
    return make_transform(rotation, translation)


def invert_transform(transform: TransformData) -> TransformData:
    rotation, translation = transform
    inverse_rotation = transpose3(rotation)
    inverse_translation = apply_rotation(
        inverse_rotation,
        [-translation[0], -translation[1], -translation[2]],
    )
    return make_transform(inverse_rotation, inverse_translation)


def offset_transform(transform: TransformData, offset_world: List[float]) -> TransformData:
    rotation, translation = transform
    return make_transform(rotation, add_vectors(translation, offset_world))


def offset_along_local_axes(transform: TransformData, offset_local: List[float]) -> TransformData:
    rotation, translation = transform
    return make_transform(rotation, add_vectors(translation, apply_rotation(rotation, offset_local)))


def vector_norm(vector: List[float]) -> float:
    return math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])


class VisualServoController(Node):
    def __init__(self) -> None:
        super().__init__('visual_servo_controller')

        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('target_topic', '/tag_pose_camera')
        self.declare_parameter('tag_pose_world_topic', '/tag_pose_world')
        self.declare_parameter('battery_center_topic', '/battery_center')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('command_topic', '/jaka_a5_arm_controller/joint_trajectory')
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('set_entity_pose_service', '/world/battery_station/set_pose')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('tool_frame', 'J6')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('group_name', 'manipulator')
        self.declare_parameter('battery_model_name', 'battery_with_tag')
        self.declare_parameter('hover_distance', 0.16)
        self.declare_parameter('grasp_distance', 0.10)
        self.declare_parameter('xy_tolerance', 0.012)
        self.declare_parameter('z_tolerance', 0.015)
        self.declare_parameter('grasp_pose_hold_duration', 5.0)
        self.declare_parameter('align_loss_grace_cycles', 5)
        self.declare_parameter('side_approach_loss_grace_cycles', 8)
        self.declare_parameter('side_approach_settle_duration', 2.0)
        self.declare_parameter('side_approach_reacquire_timeout', 6.0)
        self.declare_parameter('standby_search_start_delay', 2.0)
        self.declare_parameter('initial_search_freeze_duration', 4.0)
        self.declare_parameter('initial_tag_confirm_duration', 0.2)
        self.declare_parameter('trackable_target_timeout', 2.5)
        self.declare_parameter(
            'search_observation_joints',
            [0.20943951023931953, 1.9198621771937625, -2.181661564992912, 0.3141592653589793, 0.22689280275926285, 0.0],
        )
        self.declare_parameter('search_pan_joint', 'joint_4')
        self.declare_parameter('search_tilt_joint', 'joint_5')
        self.declare_parameter('search_pan_amplitude', 0.35)
        self.declare_parameter('search_tilt_amplitude', 0.18)
        self.declare_parameter('search_pan_steps', 5)
        self.declare_parameter('target_confirm_cycles', 2)
        self.declare_parameter('min_search_sweep_steps_before_lock', 6)
        self.declare_parameter('search_lock_visible_duration', 0.6)
        self.declare_parameter('search_spiral_radial_step', 0.012)
        self.declare_parameter('search_spiral_max_radius', 0.05)
        self.declare_parameter('search_spiral_angle_step', 0.6)
        self.declare_parameter('search_spiral_start_angle', 0.0)
        self.declare_parameter('search_ik_failure_reset_threshold', 3)
        self.declare_parameter('search_joint_window', [0.10, 0.12, 0.12, 0.22, 0.22, 0.35])
        self.declare_parameter('search_joint_step_limits', [0.10, 0.12, 0.12, 0.16, 0.16, 0.22])
        self.declare_parameter('search_max_joint_step', 0.18)
        self.declare_parameter('servo_joint_step_limits', [0.09, 0.11, 0.11, 0.16, 0.18, 0.20])
        self.declare_parameter('servo_joint_cost_weights', [2.2, 2.8, 2.8, 1.0, 0.8, 0.6])
        self.declare_parameter('servo_joint_cost_threshold', 0.95)
        self.declare_parameter('accepted_joint_seed_max_delta', 0.18)
        self.declare_parameter('angle_wrap_joints', ['joint_1', 'joint_5', 'joint_6'])
        self.declare_parameter('servo_max_joint_step', 0.25)
        self.declare_parameter('xy_gain', 0.7)
        self.declare_parameter('z_gain', 0.6)
        self.declare_parameter('max_servo_step', 0.015)
        self.declare_parameter('search_trajectory_time', 0.8)
        self.declare_parameter('servo_trajectory_time', 1.0)
        self.declare_parameter('trajectory_time', 0.8)
        self.declare_parameter('joint_tolerance', 0.04)
        self.declare_parameter('command_timeout', 2.5)
        self.declare_parameter('target_timeout', 1.5)
        self.declare_parameter('extract_offset', [0.0, 0.0, 0.10])
        self.declare_parameter('safe_exit_offset', [0.0, -0.18, 0.10])
        self.declare_parameter('old_battery_place_offset', [0.0, -0.22, 0.02])
        self.declare_parameter('tool_reset_offset', [0.0, 0.0, 0.12])
        self.declare_parameter('new_battery_pick_offset', [0.0, 0.22, -0.02])
        self.declare_parameter('dock_offset', [0.0, -0.22, 0.0])
        self.declare_parameter('insert_offset', [0.0, 0.0, -0.08])
        self.declare_parameter('final_reset_offset', [0.0, 0.0, 0.12])
        self.declare_parameter('path_step', 0.03)
        self.declare_parameter('path_tolerance', 0.015)
        self.declare_parameter('tag_to_battery_center', 0.011)
        self.declare_parameter('pre_grasp_topic', '/task_frames/pre_grasp')
        self.declare_parameter('grasp_target_topic', '/task_frames/grasp_target')
        self.declare_parameter('place_target_topic', '/task_frames/place_target')
        self.declare_parameter('use_moveit_coarse_positioning', False)
        self.declare_parameter('servo_enable_topic', '')
        self.declare_parameter('servo_cycle_topic', '/visual_servo/cycle_id')
        self.declare_parameter('debug_pause_on_state_transition', False)
        self.declare_parameter('debug_step_topic', '/visual_servo/debug_step')
        self.declare_parameter('debug_hold_topic', '/visual_servo/debug_hold')
        self.declare_parameter('event_log_path', '/tmp/jaka_a5_experiment_log.csv')

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.target_topic = self.get_parameter('target_topic').value
        self.tag_pose_world_topic = self.get_parameter('tag_pose_world_topic').value
        self.battery_center_topic = self.get_parameter('battery_center_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.ik_service_name = self.get_parameter('ik_service').value
        self.set_entity_pose_service_name = self.get_parameter('set_entity_pose_service').value
        self.world_frame = self.get_parameter('world_frame').value
        self.tool_frame = self.get_parameter('tool_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.group_name = self.get_parameter('group_name').value
        self.battery_model_name = self.get_parameter('battery_model_name').value
        self.hover_distance = float(self.get_parameter('hover_distance').value)
        self.grasp_distance = float(self.get_parameter('grasp_distance').value)
        self.xy_tolerance = float(self.get_parameter('xy_tolerance').value)
        self.z_tolerance = float(self.get_parameter('z_tolerance').value)
        self.grasp_pose_hold_duration = max(
            0.0, float(self.get_parameter('grasp_pose_hold_duration').value)
        )
        self.align_loss_grace_cycles = max(1, int(self.get_parameter('align_loss_grace_cycles').value))
        self.side_approach_loss_grace_cycles = max(
            1, int(self.get_parameter('side_approach_loss_grace_cycles').value)
        )
        self.side_approach_settle_duration = max(
            0.0, float(self.get_parameter('side_approach_settle_duration').value)
        )
        self.side_approach_reacquire_timeout = max(
            0.2, float(self.get_parameter('side_approach_reacquire_timeout').value)
        )
        self.standby_search_start_delay = max(
            0.0, float(self.get_parameter('standby_search_start_delay').value)
        )
        self.initial_search_freeze_duration = max(
            self.standby_search_start_delay,
            float(self.get_parameter('initial_search_freeze_duration').value),
        )
        self.initial_tag_confirm_duration = max(
            0.0, float(self.get_parameter('initial_tag_confirm_duration').value)
        )
        self.search_observation_joints = [float(value) for value in self.get_parameter('search_observation_joints').value]
        self.search_pan_joint = str(self.get_parameter('search_pan_joint').value)
        self.search_tilt_joint = str(self.get_parameter('search_tilt_joint').value)
        self.search_pan_amplitude = max(0.0, float(self.get_parameter('search_pan_amplitude').value))
        self.search_tilt_amplitude = max(0.0, float(self.get_parameter('search_tilt_amplitude').value))
        self.search_pan_steps = max(3, int(self.get_parameter('search_pan_steps').value))
        self.target_confirm_cycles = max(1, int(self.get_parameter('target_confirm_cycles').value))
        self.min_search_sweep_steps_before_lock = max(
            0, int(self.get_parameter('min_search_sweep_steps_before_lock').value)
        )
        self.search_lock_visible_duration = max(
            0.0, float(self.get_parameter('search_lock_visible_duration').value)
        )
        self.search_spiral_radial_step = float(self.get_parameter('search_spiral_radial_step').value)
        self.search_spiral_max_radius = float(self.get_parameter('search_spiral_max_radius').value)
        self.search_spiral_angle_step = float(self.get_parameter('search_spiral_angle_step').value)
        self.search_spiral_start_angle = float(self.get_parameter('search_spiral_start_angle').value)
        self.search_ik_failure_reset_threshold = max(
            1, int(self.get_parameter('search_ik_failure_reset_threshold').value)
        )
        self.search_joint_window = [
            float(value) for value in self.get_parameter('search_joint_window').value
        ]
        self.search_joint_step_limits = self._resolve_joint_vector_parameter('search_joint_step_limits')
        self.search_max_joint_step = max(0.01, float(self.get_parameter('search_max_joint_step').value))
        self.servo_joint_step_limits = self._resolve_joint_vector_parameter('servo_joint_step_limits')
        self.servo_joint_cost_weights = self._resolve_weight_vector_parameter('servo_joint_cost_weights')
        self.servo_joint_cost_threshold = max(
            0.0, float(self.get_parameter('servo_joint_cost_threshold').value)
        )
        self.accepted_joint_seed_max_delta = max(
            0.0, float(self.get_parameter('accepted_joint_seed_max_delta').value)
        )
        self.angle_wrap_joints = {
            str(value) for value in self.get_parameter('angle_wrap_joints').value
        }
        self.servo_max_joint_step = max(0.01, float(self.get_parameter('servo_max_joint_step').value))
        self.xy_gain = float(self.get_parameter('xy_gain').value)
        self.z_gain = float(self.get_parameter('z_gain').value)
        self.max_servo_step = float(self.get_parameter('max_servo_step').value)
        self.search_trajectory_time = max(0.05, float(self.get_parameter('search_trajectory_time').value))
        self.servo_trajectory_time = max(0.05, float(self.get_parameter('servo_trajectory_time').value))
        self.trajectory_time = float(self.get_parameter('trajectory_time').value)
        self.joint_tolerance = float(self.get_parameter('joint_tolerance').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.trackable_target_timeout = max(
            self.target_timeout, float(self.get_parameter('trackable_target_timeout').value)
        )
        self.extract_offset = [float(value) for value in self.get_parameter('extract_offset').value]
        self.safe_exit_offset = [float(value) for value in self.get_parameter('safe_exit_offset').value]
        self.old_battery_place_offset = [float(value) for value in self.get_parameter('old_battery_place_offset').value]
        self.tool_reset_offset = [float(value) for value in self.get_parameter('tool_reset_offset').value]
        self.new_battery_pick_offset = [float(value) for value in self.get_parameter('new_battery_pick_offset').value]
        self.dock_offset = [float(value) for value in self.get_parameter('dock_offset').value]
        self.insert_offset = [float(value) for value in self.get_parameter('insert_offset').value]
        self.final_reset_offset = [float(value) for value in self.get_parameter('final_reset_offset').value]
        self.path_step = float(self.get_parameter('path_step').value)
        self.path_tolerance = float(self.get_parameter('path_tolerance').value)
        self.tag_to_battery_center = float(self.get_parameter('tag_to_battery_center').value)
        self.pre_grasp_topic = self.get_parameter('pre_grasp_topic').value
        self.grasp_target_topic = self.get_parameter('grasp_target_topic').value
        self.place_target_topic = self.get_parameter('place_target_topic').value
        self.use_moveit_coarse_positioning = bool(
            self.get_parameter('use_moveit_coarse_positioning').value
        )
        self.servo_enable_topic = str(self.get_parameter('servo_enable_topic').value)
        self.servo_cycle_topic = str(self.get_parameter('servo_cycle_topic').value)
        self.debug_pause_on_state_transition = bool(
            self.get_parameter('debug_pause_on_state_transition').value
        )
        self.debug_step_topic = str(self.get_parameter('debug_step_topic').value)
        self.debug_hold_topic = str(self.get_parameter('debug_hold_topic').value)
        self.event_log_path = str(self.get_parameter('event_log_path').value)

        self.state = ServoState.S0_STANDBY
        self.current_joint_state: Dict[str, float] = {}
        self.last_target_pose: Optional[PoseStamped] = None
        self.last_target_received_time = None
        self.last_target_stamp = None
        self.last_tag_pose_world: Optional[PoseStamped] = None
        self.last_battery_center_pose: Optional[PoseStamped] = None
        self.commanded_joint_positions: Optional[List[float]] = None
        self.last_accepted_joint_positions: Optional[List[float]] = None
        self.last_command_time = None
        self.last_target_feedback_time = None
        self.awaiting_motion = False
        self.pending_ik_future = None
        self.pending_set_pose_future = None
        self.path_anchor: Optional[TransformData] = None
        self.battery_attached = False
        self.tool_to_battery_transform: Optional[TransformData] = None
        self.last_requested_state: Optional[ServoState] = None
        self.service_warning_emitted = False
        self.attach_service_warning_emitted = False
        self.servo_enabled = not self.servo_enable_topic
        self.last_logged_servo_enabled: Optional[bool] = None
        self.last_servo_enable_update_time = None
        self.servo_gate_logged = False
        self.task_cycle_complete = False
        self.search_pose_commanded = False
        self.search_sweep_started = False
        self.search_sweep_index = 0
        self.search_sweep_steps_completed = 0
        self.search_anchor_joints: Optional[List[float]] = None
        self.search_use_current_anchor = False
        self.search_anchor_tool: Optional[TransformData] = None
        self.search_anchor_camera: Optional[TransformData] = None
        self.frozen_pre_grasp_transform: Optional[TransformData] = None
        self.frozen_grasp_transform: Optional[TransformData] = None
        self.search_ik_failure_count = 0
        self.align_target_missing_cycles = 0
        self.align_goal_hold_start_time = None
        self.align_goal_hold_logged = False
        self.side_approach_target_missing_cycles = 0
        self.side_approach_enable_time = None
        self.side_approach_settle_logged = False
        self.target_confirmed = False
        self.fresh_target_cycles = 0
        self.fresh_target_hold_start_time = None
        self.search_pause_logged = False
        self.search_delay_logged = False
        self.search_halt_commanded = False
        self.last_confirmation_gate_reason: Optional[str] = None
        self.cycle_id = 0
        self.active_cycle_id = 0
        self.debug_waiting_for_step = False
        self.debug_wait_state: Optional[ServoState] = None
        self.debug_wait_reason: Optional[str] = None
        self.standby_enter_wall_time = time.monotonic()
        self.state_enter_time = self.get_clock().now()
        self.event_logger = CsvExperimentLogger(self.event_log_path)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trajectory_pub = self.create_publisher(JointTrajectory, self.command_topic, 10)
        self.pre_grasp_pub = self.create_publisher(PoseStamped, self.pre_grasp_topic, 10)
        self.grasp_target_pub = self.create_publisher(PoseStamped, self.grasp_target_topic, 10)
        self.place_target_pub = self.create_publisher(PoseStamped, self.place_target_topic, 10)
        self.servo_cycle_pub = self.create_publisher(Int32, self.servo_cycle_topic, 10)
        debug_hold_qos = QoSProfile(depth=1)
        debug_hold_qos.reliability = ReliabilityPolicy.RELIABLE
        debug_hold_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.debug_hold_pub = self.create_publisher(Bool, self.debug_hold_topic, debug_hold_qos)
        self.target_sub = self.create_subscription(PoseStamped, self.target_topic, self.target_callback, 10)
        self.tag_pose_world_sub = self.create_subscription(PoseStamped, self.tag_pose_world_topic, self.tag_pose_world_callback, 10)
        self.battery_center_sub = self.create_subscription(PoseStamped, self.battery_center_topic, self.battery_center_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 20)
        self.debug_step_sub = self.create_subscription(
            String,
            self.debug_step_topic,
            self.debug_step_callback,
            10,
        )
        self.servo_enable_sub = None
        if self.servo_enable_topic:
            servo_enable_qos = QoSProfile(depth=1)
            servo_enable_qos.reliability = ReliabilityPolicy.RELIABLE
            servo_enable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.servo_enable_sub = self.create_subscription(
                Bool,
                self.servo_enable_topic,
                self.servo_enable_callback,
                servo_enable_qos,
            )

        self.ik_client = self.create_client(GetPositionIK, self.ik_service_name)
        self.set_entity_pose_client = self.create_client(SetEntityPose, self.set_entity_pose_service_name)

        self.control_timer = self.create_timer(0.2, self.control_loop)
        self._publish_debug_hold(False)
        self.get_logger().info('IK visual servo controller started')
        self._log_event('controller_started', f'CSV logging enabled at {self.event_log_path}')

    def target_callback(self, msg: PoseStamped) -> None:
        self.last_target_pose = msg
        self.last_target_received_time = self.get_clock().now()
        self.last_target_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self._log_target_feedback(msg)

    def tag_pose_world_callback(self, msg: PoseStamped) -> None:
        self.last_tag_pose_world = msg

    def battery_center_callback(self, msg: PoseStamped) -> None:
        self.last_battery_center_pose = msg

    def servo_enable_callback(self, msg: Bool) -> None:
        previous_value = self.servo_enabled
        self.servo_enabled = msg.data
        self.last_servo_enable_update_time = self.get_clock().now()
        self.servo_gate_logged = False
        if self.last_logged_servo_enabled is None or previous_value != self.servo_enabled:
            state = 'enabled' if self.servo_enabled else 'disabled'
            self._log_event('servo_gate_changed', f'visual servo {state}')
            self.last_logged_servo_enabled = self.servo_enabled
        if not self.servo_enabled:
            self.task_cycle_complete = False

    def debug_step_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command not in {'', 'next', 'continue', 'resume', 'step'}:
            self.get_logger().info(
                f'Ignoring debug step command "{msg.data}". Use one of: next, continue, resume, step'
            )
            return

        if not self.debug_waiting_for_step:
            self.get_logger().info('Debug step command received, but the state machine is not paused')
            return

        waiting_state = self.debug_wait_state.value if self.debug_wait_state is not None else self.state.value
        self.debug_waiting_for_step = False
        self.debug_wait_state = None
        self.debug_wait_reason = None
        self._publish_debug_hold(False)
        self.get_logger().info(f'Debug step received, resuming state machine from {waiting_state}')
        self._log_event('debug_step_resume', f'resumed state machine from {waiting_state}')

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.current_joint_state[name] = position

    def control_loop(self) -> None:
        self._update_attached_battery_pose()
        self._update_target_confirmation()

        if self.debug_waiting_for_step:
            return

        if self.state == ServoState.S0_STANDBY and not self.task_cycle_complete and self.target_confirmed:
            self._freeze_task_targets()
            self._start_cycle_if_needed('fresh AprilTag target acquired')
            if self.use_moveit_coarse_positioning:
                self._publish_cycle_id()
                transition_reason = (
                    'Target confirmed during fan search, waiting for coarse side-approach completion'
                )
            else:
                transition_reason = (
                    'Target confirmed during fan search, start direct visual coarse alignment to hover pose'
                )
            self._set_state(ServoState.S1_SIDE_APPROACH, transition_reason)

        self._publish_task_frames()

        if self.state == ServoState.S0_STANDBY and not self.target_confirmed:
            self._hold_search_observation_pose()

        if not self._joint_state_ready():
            return

        if self.pending_ik_future is not None:
            return

        if self.awaiting_motion:
            if self._trajectory_reached():
                self.awaiting_motion = False
                self.commanded_joint_positions = None
            elif self._command_timed_out():
                self.awaiting_motion = False
                self.commanded_joint_positions = None
                self.get_logger().warn('Trajectory convergence timed out, continuing with next control step')
                self._log_event('trajectory_timeout', 'joint trajectory did not converge before timeout')
            else:
                return

        if self.state == ServoState.S0_STANDBY:
            return

        if self.state == ServoState.S1_SIDE_APPROACH:
            if self.use_moveit_coarse_positioning:
                if not self.servo_enabled:
                    if not self.servo_gate_logged:
                        self.get_logger().info('Visual servo is waiting for external enable signal')
                        self.servo_gate_logged = True
                    return

                if (
                    self.last_servo_enable_update_time is None
                    or self.last_servo_enable_update_time < self.state_enter_time
                ):
                    if not self.servo_gate_logged:
                        self.get_logger().info('Waiting for coarse-position enable update of current cycle')
                        self.servo_gate_logged = True
                    return

                if self.side_approach_enable_time is None:
                    self.side_approach_enable_time = self.get_clock().now()
                    self.side_approach_settle_logged = False

                if self._has_fresh_target():
                    self.side_approach_target_missing_cycles = 0
                    self._set_state(
                        ServoState.S2_ALIGN,
                        'Coarse side approach complete, start fine alignment to centered 3 cm grasp pose',
                    )
                    return

                since_enable_sec = (
                    self.get_clock().now() - self.side_approach_enable_time
                ).nanoseconds / 1e9
                if since_enable_sec < self.side_approach_settle_duration:
                    if not self.side_approach_settle_logged:
                        self._log_event(
                            'target_reacquire_settling',
                            f'waiting {self.side_approach_settle_duration:.1f}s after coarse positioning '
                            'before evaluating tag-reacquire timeout',
                        )
                        self.side_approach_settle_logged = True
                    return

                reacquire_wait_sec = since_enable_sec - self.side_approach_settle_duration
                if reacquire_wait_sec >= self.side_approach_reacquire_timeout:
                    self._log_event(
                        'target_reacquire_timeout',
                        f'AprilTag not reacquired within {self.side_approach_reacquire_timeout:.1f}s '
                        'after coarse-position settling; returning to standby',
                    )
                    self._set_state(
                        ServoState.S0_STANDBY,
                        'AprilTag not reacquired after coarse positioning; restart standby confirmation',
                    )
                return

            if not self._has_fresh_target():
                self.side_approach_target_missing_cycles += 1
                if self.side_approach_target_missing_cycles < self.side_approach_loss_grace_cycles:
                    return
                self._log_event(
                    'target_lost',
                    'AprilTag lost during direct coarse alignment; returning to standby for reacquisition',
                )
                self._set_state(
                    ServoState.S0_STANDBY,
                    'AprilTag lost during direct coarse alignment; restart standby confirmation and search gating',
                )
                return

            self.side_approach_target_missing_cycles = 0
            if self._servo_to_tag(self.hover_distance):
                self._set_state(
                    ServoState.S2_ALIGN,
                    'Direct coarse alignment complete, start fine alignment to grasp pose',
                )
            return

        if self.use_moveit_coarse_positioning and not self.servo_enabled:
            if not self.servo_gate_logged:
                self.get_logger().info('Visual servo is waiting for external enable signal')
                self.servo_gate_logged = True
            return

        if self.state == ServoState.S2_ALIGN:
            if self.align_goal_hold_start_time is not None:
                held_duration = (
                    self.get_clock().now() - self.align_goal_hold_start_time
                ).nanoseconds / 1e9
                if held_duration >= self.grasp_pose_hold_duration:
                    self._set_state(
                        ServoState.S3_CLAMP,
                        'AprilTag centered and grasp-ready pose held for required duration, clamp old battery',
                    )
                return

            if self._tag_within_servo_tolerance(self.grasp_distance, allow_trackable_target=True):
                self.align_goal_hold_start_time = self.get_clock().now()
                self.align_goal_hold_logged = False

                if not self.align_goal_hold_logged:
                    self._log_event(
                        'grasp_pose_hold_started',
                        f'grasp-ready pose reached at camera-tag distance {self.grasp_distance:.3f} m; '
                        f'holding for {self.grasp_pose_hold_duration:.1f}s before clamp',
                    )
                    self.align_goal_hold_logged = True
                return

            self.align_goal_hold_start_time = None
            self.align_goal_hold_logged = False
            self._servo_to_tag(self.grasp_distance, allow_trackable_target=True)
            return

        if self.state == ServoState.S3_CLAMP:
            if self._attach_battery_to_tool():
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.S4_EXTRACT, 'Old battery clamped, start extraction')
            return

        if self.state == ServoState.S4_EXTRACT:
            if self._drive_tool_from_anchor(self.extract_offset):
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.S5_SAFE_EXIT, 'Old battery extracted, move to safe exit pose')
            return

        if self.state == ServoState.S5_SAFE_EXIT:
            if self._drive_tool_from_anchor(self.safe_exit_offset):
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.S6_PLACE_OLD_BATTERY, 'Safe exit reached, place old battery')
            return

        if self.state == ServoState.S6_PLACE_OLD_BATTERY:
            if self._drive_tool_from_anchor(self.old_battery_place_offset):
                self._release_battery()
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.S7_TOOL_RESET, 'Old battery placed, reset tool')
            return

        if self.state == ServoState.S7_TOOL_RESET:
            if self._drive_tool_from_anchor(self.tool_reset_offset):
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.S8_GRASP_NEW_BATTERY, 'Tool reset complete, move to grasp new battery')
            return

        if self.state == ServoState.S8_GRASP_NEW_BATTERY:
            if self._drive_tool_from_anchor(self.new_battery_pick_offset):
                if self._attach_battery_from_current_tool():
                    self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                    self._set_state(ServoState.S9_MOVE_TO_DOCK, 'New battery grasped in task-level simulation, move to dock')
            return

        if self.state == ServoState.S9_MOVE_TO_DOCK:
            if self._drive_tool_from_anchor(self.dock_offset):
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.S10_INSERT_NEW_BATTERY, 'Dock pose reached, insert new battery')
            return

        if self.state == ServoState.S10_INSERT_NEW_BATTERY:
            if self._drive_tool_from_anchor(self.insert_offset):
                self._release_battery()
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.S11_RESET, 'Insertion complete, reset manipulator')
            return

        if self.state == ServoState.S11_RESET:
            if self._drive_tool_from_anchor(self.final_reset_offset):
                self.task_cycle_complete = True
                self._log_event('cycle_complete', 'battery-swap cycle finished successfully')
                self._set_state(ServoState.S0_STANDBY, 'Battery-swap cycle complete, return to standby')
                self.active_cycle_id = 0
            return

    def _joint_state_ready(self) -> bool:
        return all(name in self.current_joint_state for name in self.joint_names)

    def _has_fresh_target(self) -> bool:
        if self.last_target_pose is None or self.last_target_received_time is None or self.last_target_stamp is None:
            return False

        receive_age = (self.get_clock().now() - self.last_target_received_time).nanoseconds / 1e9
        return receive_age <= self.target_timeout

    def _has_trackable_target(self) -> bool:
        if self.last_target_pose is None or self.last_target_received_time is None:
            return False

        receive_age = (self.get_clock().now() - self.last_target_received_time).nanoseconds / 1e9
        return receive_age <= self.trackable_target_timeout

    def _command_timed_out(self) -> bool:
        if self.last_command_time is None:
            return False

        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        return age > self.command_timeout

    def _trajectory_reached(self) -> bool:
        if self.commanded_joint_positions is None:
            return True

        current = self._joint_position_vector()
        if current is None:
            return False

        max_error = max(abs(current[index] - self.commanded_joint_positions[index]) for index in range(len(self.joint_names)))
        return max_error < self.joint_tolerance

    def _joint_position_vector(self) -> Optional[List[float]]:
        if not self._joint_state_ready():
            return None
        return [self.current_joint_state[name] for name in self.joint_names]

    def _resolve_joint_vector_parameter(self, parameter_name: str) -> List[float]:
        values = [float(value) for value in self.get_parameter(parameter_name).value]
        if len(values) != len(self.joint_names):
            self.get_logger().warn(
                f'{parameter_name} length {len(values)} does not match manipulator joint count {len(self.joint_names)}; '
                'falling back to repeated final value'
            )
            fallback_value = values[-1] if values else 0.1
            values = [fallback_value for _ in self.joint_names]
        return values

    def _resolve_weight_vector_parameter(self, parameter_name: str) -> List[float]:
        values = [float(value) for value in self.get_parameter(parameter_name).value]
        if len(values) != len(self.joint_names):
            self.get_logger().warn(
                f'{parameter_name} length {len(values)} does not match manipulator joint count {len(self.joint_names)}; '
                'falling back to uniform weights'
            )
            values = [1.0 for _ in self.joint_names]
        return values

    def _preferred_ik_seed_positions(self) -> Optional[List[float]]:
        current_positions = self._joint_position_vector()
        if current_positions is None:
            return None
        if self.last_accepted_joint_positions is None:
            return current_positions

        max_delta = max(
            abs(self.last_accepted_joint_positions[index] - current_positions[index])
            for index in range(len(self.joint_names))
        )
        if max_delta > self.accepted_joint_seed_max_delta:
            return current_positions
        return list(self.last_accepted_joint_positions)

    def _joint_step_limits_for_state(self, state: Optional[ServoState]) -> Optional[List[float]]:
        if state == ServoState.S0_STANDBY:
            return self.search_joint_step_limits
        if state in {ServoState.S1_SIDE_APPROACH, ServoState.S2_ALIGN}:
            return self.servo_joint_step_limits
        return None

    def _trajectory_time_for_state(self, state: Optional[ServoState]) -> float:
        if state == ServoState.S0_STANDBY:
            return self.search_trajectory_time
        if state in {ServoState.S1_SIDE_APPROACH, ServoState.S2_ALIGN}:
            return self.servo_trajectory_time
        return self.trajectory_time

    def _apply_joint_step_limits(
        self,
        candidate_positions: List[float],
        state: Optional[ServoState],
    ) -> List[float]:
        current_positions = self._joint_position_vector()
        joint_step_limits = self._joint_step_limits_for_state(state)
        if current_positions is None or joint_step_limits is None:
            return list(candidate_positions)

        limited_positions = list(candidate_positions)
        for index in range(len(limited_positions)):
            delta = limited_positions[index] - current_positions[index]
            limit = abs(joint_step_limits[index])
            limited_positions[index] = current_positions[index] + clamp(delta, limit)
        return limited_positions

    def _per_joint_limit_violations(
        self,
        candidate_positions: List[float],
        current_positions: List[float],
        joint_step_limits: List[float],
    ) -> List[str]:
        violations: List[str] = []
        for index, joint_name in enumerate(self.joint_names):
            delta = abs(candidate_positions[index] - current_positions[index])
            if delta > joint_step_limits[index]:
                violations.append(f'{joint_name}:{delta:.3f}>{joint_step_limits[index]:.3f}')
        return violations

    def _weighted_joint_cost(self, candidate_positions: List[float], current_positions: List[float]) -> float:
        return sum(
            self.servo_joint_cost_weights[index] * abs(candidate_positions[index] - current_positions[index])
            for index in range(len(self.joint_names))
        )

    def _normalize_angle_wrap_joints(
        self,
        candidate_positions: List[float],
        reference_positions: List[float],
    ) -> List[float]:
        normalized_positions = list(candidate_positions)
        for index, joint_name in enumerate(self.joint_names):
            if joint_name not in self.angle_wrap_joints:
                continue

            delta = normalized_positions[index] - reference_positions[index]
            while delta > math.pi:
                normalized_positions[index] -= 2.0 * math.pi
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                normalized_positions[index] += 2.0 * math.pi
                delta += 2.0 * math.pi
        return normalized_positions

    def _hold_search_observation_pose(self) -> None:
        if self._should_pause_standby_search():
            if not self.search_pause_logged:
                if self._has_trackable_target() or self.fresh_target_hold_start_time is not None:
                    self.get_logger().info('Tag 已进入可确认窗口，暂停继续搜索并保持当前视角')
                    self._log_event(
                        'search_paused',
                        'target remains trackable in standby; pausing observation-pose return and fan search',
                    )
                else:
                    self.get_logger().info('启动宽限期内保持机械臂静止，等待初始 Tag 进入稳定观测')
                    self._log_event(
                        'search_startup_freeze',
                        'holding manipulator still during startup grace window before enabling standby search motion',
                    )
                self.search_pause_logged = True
            if (
                not self.search_halt_commanded
                and self._joint_state_ready()
                and self.pending_ik_future is None
                and (self.awaiting_motion or self.commanded_joint_positions is not None)
            ):
                current = self._joint_position_vector()
                if current is not None:
                    self._publish_trajectory(current)
                    self.search_halt_commanded = True
                    self._log_event(
                        'search_motion_halted',
                        'standby search motion halted while waiting for stable initial target acquisition',
                    )
            return

        self.search_pause_logged = False
        self.search_halt_commanded = False

        standby_age = time.monotonic() - self.standby_enter_wall_time
        if (
            not self.search_pose_commanded
            and not self.search_sweep_started
            and standby_age < self.standby_search_start_delay
        ):
            if not self.search_delay_logged:
                self._log_event(
                    'search_start_delayed',
                    f'delaying standby search start for {self.standby_search_start_delay:.2f}s to allow initial tag acquisition',
                )
                self.search_delay_logged = True
            return

        self.search_delay_logged = False

        if not self._joint_state_ready() or self.awaiting_motion or self.pending_ik_future is not None:
            return

        current = self._joint_position_vector()
        if current is None:
            return

        search_anchor_joints = self._get_search_anchor_joints(current)
        if search_anchor_joints is None:
            return

        if len(search_anchor_joints) != len(self.joint_names):
            if not self.service_warning_emitted:
                self.get_logger().warn('search_observation_joints length does not match manipulator joint count')
                self.service_warning_emitted = True
            return

        max_error = max(
            abs(current[index] - search_anchor_joints[index])
            for index in range(len(self.joint_names))
        )
        if max_error < self.joint_tolerance:
            self.search_pose_commanded = True
            if not self.search_sweep_started:
                if self.search_pan_joint not in self.joint_names or self.search_tilt_joint not in self.joint_names:
                    self.get_logger().warn('search pan/tilt joints are not part of the manipulator joint list')
                    self._log_event(
                        'search_config_error',
                        'search pan/tilt joints are not part of the manipulator joint list',
                    )
                    return
                self.search_sweep_started = True
                self.search_sweep_index = 0
                self.search_sweep_steps_completed = 0
                self.search_ik_failure_count = 0

        if self.search_pose_commanded:
            self._run_search_sweep(current)
            return

        self.get_logger().info('Moving to fixed search observation pose for AprilTag acquisition')
        self._log_event('search_pose_commanded', 'moving to fixed observation pose for AprilTag acquisition')
        self._publish_trajectory(search_anchor_joints)
        self.search_pose_commanded = True

    def _should_pause_standby_search(self) -> bool:
        if self._has_trackable_target() or self.fresh_target_hold_start_time is not None:
            return True

        standby_age = time.monotonic() - self.standby_enter_wall_time
        return (
            self.active_cycle_id == 0
            and not self.search_pose_commanded
            and not self.search_sweep_started
            and standby_age < self.initial_search_freeze_duration
        )

    def _run_search_sweep(self, current: List[float]) -> None:
        fan_waypoints = self._search_fan_waypoints()
        if not fan_waypoints:
            return

        target_joints = fan_waypoints[self.search_sweep_index]
        max_error = max(
            abs(current[index] - target_joints[index])
            for index in range(len(self.joint_names))
        )
        if max_error >= self.joint_tolerance:
            self._publish_trajectory(target_joints)
            return

        self.search_sweep_index = (self.search_sweep_index + 1) % len(fan_waypoints)
        self.search_sweep_steps_completed += 1
        self.search_ik_failure_count = 0
        next_target = fan_waypoints[self.search_sweep_index]
        pan_index = self.joint_names.index(self.search_pan_joint)
        tilt_index = self.joint_names.index(self.search_tilt_joint)
        pan_offset = next_target[pan_index] - self.search_observation_joints[pan_index]
        tilt_offset = next_target[tilt_index] - self.search_observation_joints[tilt_index]
        self.get_logger().info(
            'Fan-search waypoint reached, sweeping camera orientation while keeping the camera position nearly fixed'
        )
        self._log_event(
            'search_sweep_step',
            'fan search '
            f'joint offsets {self.search_pan_joint}/{self.search_tilt_joint} -> ({pan_offset:.3f}, {tilt_offset:.3f})',
        )
        self._publish_trajectory(next_target)

    def _search_fan_waypoints(self) -> List[List[float]]:
        if self.search_pan_joint not in self.joint_names or self.search_tilt_joint not in self.joint_names:
            return []
        if self.search_anchor_joints is None:
            return []

        pan_index = self.joint_names.index(self.search_pan_joint)
        tilt_index = self.joint_names.index(self.search_tilt_joint)
        pan_steps = max(3, self.search_pan_steps)
        if pan_steps % 2 == 0:
            pan_steps += 1

        if pan_steps == 1:
            pan_values = [0.0]
        else:
            pan_values = [
                -self.search_pan_amplitude + (2.0 * self.search_pan_amplitude * index / (pan_steps - 1))
                for index in range(pan_steps)
            ]

        tilt_values = [0.0]
        if self.search_tilt_amplitude > 1e-6:
            tilt_values.extend([
                self.search_tilt_amplitude * 0.5,
                -self.search_tilt_amplitude * 0.5,
                self.search_tilt_amplitude,
                -self.search_tilt_amplitude,
            ])

        waypoints: List[List[float]] = []
        for row_index, tilt_offset in enumerate(tilt_values):
            row_pan_values = pan_values if row_index % 2 == 0 else list(reversed(pan_values))
            for pan_offset in row_pan_values:
                waypoint = list(self.search_anchor_joints)
                waypoint[pan_index] += pan_offset
                waypoint[tilt_index] += tilt_offset
                waypoints.append(waypoint)

        return waypoints

    def _get_search_anchor_joints(self, current: List[float]) -> Optional[List[float]]:
        if self.search_anchor_joints is not None:
            return self.search_anchor_joints

        if len(self.search_observation_joints) != len(self.joint_names):
            return None

        anchor = list(self.search_observation_joints)
        if self.search_use_current_anchor:
            # After losing the tag during coarse/fine positioning, search around the current posture
            # instead of pulling the arm back through a large reset move.
            for index in range(len(self.joint_names)):
                anchor[index] = current[index]

        self.search_anchor_joints = anchor
        return self.search_anchor_joints

    def _search_spiral_targets(self) -> List[TransformData]:
        if self.search_anchor_tool is None or self.search_anchor_camera is None:
            return []

        targets: List[TransformData] = []
        step_count = max(
            1,
            int(math.ceil(self.search_spiral_max_radius / max(self.search_spiral_radial_step, 1e-6))) * 8,
        )
        for index in range(step_count):
            target = self._spiral_target_tool_transform(index)
            if target is not None:
                targets.append(target)
        return targets

    def _spiral_target_tool_transform(self, index: int) -> Optional[TransformData]:
        target_camera = self._spiral_target_camera_transform(index)
        if target_camera is None or self.search_anchor_tool is None or self.search_anchor_camera is None:
            return None

        tool_to_camera = compose_transforms(invert_transform(self.search_anchor_tool), self.search_anchor_camera)
        return compose_transforms(target_camera, invert_transform(tool_to_camera))

    def _spiral_target_camera_transform(self, index: int) -> Optional[TransformData]:
        if self.search_anchor_camera is None:
            return None

        if index == 0:
            return self.search_anchor_camera

        angle = self.search_spiral_start_angle + index * self.search_spiral_angle_step
        radius = min(self.search_spiral_max_radius, self.search_spiral_radial_step * math.sqrt(index))
        local_offset = [radius * math.cos(angle), radius * math.sin(angle), 0.0]
        return offset_along_local_axes(self.search_anchor_camera, local_offset)

    def _lookup_frame_transform(self, frame_name: str) -> Optional[TransformData]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                frame_name,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Failed to lookup transform {self.world_frame} -> {frame_name}: {exc}')
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = Pose()
        pose.position.x = translation.x
        pose.position.y = translation.y
        pose.position.z = translation.z
        pose.orientation = rotation
        return transform_from_pose(pose)

    def _tag_within_servo_tolerance(
        self,
        desired_distance: float,
        *,
        allow_trackable_target: bool = False,
    ) -> bool:
        target_available = self._has_trackable_target() if allow_trackable_target else self._has_fresh_target()
        if not target_available or self.last_target_pose is None:
            return False

        tag = self.last_target_pose.pose.position
        lateral_error = math.hypot(tag.x, tag.y)
        distance_error = tag.z - desired_distance
        return lateral_error < self.xy_tolerance and abs(distance_error) < self.z_tolerance

    def _servo_to_tag(self, desired_distance: float, *, allow_trackable_target: bool = False) -> bool:
        target_available = self._has_trackable_target() if allow_trackable_target else self._has_fresh_target()
        if not target_available:
            return False
        desired_tool = self._compute_servo_goal_transform(desired_distance)
        if desired_tool is None:
            return False

        if self._tag_within_servo_tolerance(
            desired_distance,
            allow_trackable_target=allow_trackable_target,
        ):
            return True
        return self._request_ik_for_transform(desired_tool)

    def _compute_servo_goal_transform(self, desired_distance: float) -> Optional[TransformData]:
        if self.last_target_pose is None:
            return None

        world_tool = self._lookup_frame_transform(self.tool_frame)
        world_camera = self._lookup_frame_transform(self.camera_frame)
        if world_tool is None or world_camera is None:
            return None

        tag = self.last_target_pose.pose.position
        distance_error = tag.z - desired_distance
        camera_delta = [
            clamp(tag.x * self.xy_gain, self.max_servo_step),
            clamp(tag.y * self.xy_gain, self.max_servo_step),
            clamp(distance_error * self.z_gain, self.max_servo_step),
        ]

        world_camera_rotation, _ = world_camera
        world_delta = apply_rotation(world_camera_rotation, camera_delta)
        desired_camera = offset_transform(world_camera, world_delta)
        tool_to_camera = compose_transforms(invert_transform(world_tool), world_camera)
        return compose_transforms(desired_camera, invert_transform(tool_to_camera))

    def _compute_task_goal_transform(self, desired_distance: float) -> Optional[TransformData]:
        if self.last_target_pose is None:
            return None

        world_tool = self._lookup_frame_transform(self.tool_frame)
        world_camera = self._lookup_frame_transform(self.camera_frame)
        if world_tool is None or world_camera is None:
            return None

        current_camera_to_tag = transform_from_pose(self.last_target_pose.pose)
        desired_camera_to_tag_pose = Pose()
        desired_camera_to_tag_pose.position.x = 0.0
        desired_camera_to_tag_pose.position.y = 0.0
        desired_camera_to_tag_pose.position.z = desired_distance
        desired_camera_to_tag_pose.orientation = self.last_target_pose.pose.orientation
        desired_camera_to_tag = transform_from_pose(desired_camera_to_tag_pose)

        world_tag = compose_transforms(world_camera, current_camera_to_tag)
        desired_camera = compose_transforms(world_tag, invert_transform(desired_camera_to_tag))
        tool_to_camera = compose_transforms(invert_transform(world_tool), world_camera)
        return compose_transforms(desired_camera, invert_transform(tool_to_camera))

    def _drive_tool_from_anchor(self, offset_world: List[float]) -> bool:
        if self.path_anchor is None:
            self.path_anchor = self._lookup_frame_transform(self.tool_frame)
            if self.path_anchor is None:
                return False

        current_tool = self._lookup_frame_transform(self.tool_frame)
        if current_tool is None:
            return False

        target_tool = offset_transform(self.path_anchor, offset_world)
        _, current_translation = current_tool
        _, target_translation = target_tool
        error = subtract_vectors(target_translation, current_translation)
        distance = vector_norm(error)

        if distance < self.path_tolerance:
            return True

        step_scale = min(1.0, self.path_step / max(distance, 1e-6))
        limited_step = [component * step_scale for component in error]
        desired_tool = offset_transform(current_tool, limited_step)
        return self._request_ik_for_transform(desired_tool)

    def _request_ik_for_transform(self, transform: TransformData) -> bool:
        if not self.ik_client.service_is_ready():
            if not self.service_warning_emitted:
                self.get_logger().warn(f'IK service not ready: {self.ik_service_name}')
                self.service_warning_emitted = True
            return False

        self.service_warning_emitted = False
        joint_positions = self._preferred_ik_seed_positions()
        if joint_positions is None:
            return False

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.tool_frame
        request.ik_request.avoid_collisions = False
        request.ik_request.pose_stamped.header.frame_id = self.world_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = pose_from_transform(transform)
        request.ik_request.robot_state.joint_state.name = list(self.joint_names)
        request.ik_request.robot_state.joint_state.position = list(joint_positions)
        request.ik_request.timeout.sec = 1
        request.ik_request.timeout.nanosec = 0

        self.last_requested_state = self.state
        self.pending_ik_future = self.ik_client.call_async(request)
        self.pending_ik_future.add_done_callback(self._handle_ik_response)
        return False

    def _handle_ik_response(self, future) -> None:
        self.pending_ik_future = None

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'IK request failed: {exc}')
            self._log_event('ik_failure', f'IK request exception: {exc}')
            return

        if response.error_code.val != 1:
            self.get_logger().warn(
                f'IK solver returned error {response.error_code.val} in state {self.last_requested_state}'
            )
            if self.last_requested_state == ServoState.S0_STANDBY:
                self.search_ik_failure_count += 1
                if self.search_ik_failure_count >= self.search_ik_failure_reset_threshold:
                    self._reset_search_pattern('search IK failures exceeded threshold')
            self._log_event(
                'ik_failure',
                f'IK solver error {response.error_code.val} in state {self.last_requested_state}',
            )
            return

        solution_map = {
            name: position
            for name, position in zip(response.solution.joint_state.name, response.solution.joint_state.position)
        }
        if not all(name in solution_map for name in self.joint_names):
            self.get_logger().warn('IK solution missing required manipulator joints')
            self._log_event('ik_failure', 'IK solution missing required manipulator joints')
            return

        ordered_positions = [solution_map[name] for name in self.joint_names]
        reference_positions = self._joint_position_vector()
        if reference_positions is not None:
            ordered_positions = self._normalize_angle_wrap_joints(ordered_positions, reference_positions)
        limited_positions = self._apply_joint_step_limits(ordered_positions, self.last_requested_state)
        if not self._ik_solution_allowed(limited_positions):
            return
        self._publish_trajectory(limited_positions)

    def _ik_solution_allowed(self, candidate_positions: List[float]) -> bool:
        current_positions = self._joint_position_vector()
        if current_positions is None:
            return False

        max_step = max(
            abs(candidate_positions[index] - current_positions[index])
            for index in range(len(self.joint_names))
        )

        if self.last_requested_state == ServoState.S0_STANDBY:
            if len(self.search_joint_window) != len(self.joint_names):
                self.get_logger().warn('search_joint_window length does not match manipulator joint count')
                self._log_event('ik_failure', 'search_joint_window length does not match manipulator joint count')
                return False

            exceeds_window = any(
                abs(candidate_positions[index] - self.search_observation_joints[index]) > self.search_joint_window[index]
                for index in range(len(self.joint_names))
            )
            limit_violations = self._per_joint_limit_violations(
                candidate_positions,
                current_positions,
                self.search_joint_step_limits,
            )
            if exceeds_window or max_step > self.search_max_joint_step or limit_violations:
                self.search_ik_failure_count += 1
                self._log_event(
                    'ik_solution_rejected',
                    'search IK solution exceeded standby joint window or joint-step limit',
                )
                if self.search_ik_failure_count >= self.search_ik_failure_reset_threshold:
                    self._reset_search_pattern('search IK solution exceeded joint limits repeatedly')
                return False
            self.search_ik_failure_count = 0
            return True

        if self.last_requested_state in {ServoState.S1_SIDE_APPROACH, ServoState.S2_ALIGN}:
            limit_violations = self._per_joint_limit_violations(
                candidate_positions,
                current_positions,
                self.servo_joint_step_limits,
            )
            if limit_violations:
                self._log_event(
                    'ik_solution_rejected',
                    f'servo IK solution exceeded per-joint limits ({", ".join(limit_violations)})',
                )
                return False
            if max_step > self.servo_max_joint_step:
                self._log_event(
                    'ik_solution_rejected',
                    f'servo IK solution exceeded single-step joint limit ({max_step:.3f} rad)',
                )
                return False
            continuity_cost = self._weighted_joint_cost(candidate_positions, current_positions)
            if continuity_cost > self.servo_joint_cost_threshold:
                self._log_event(
                    'ik_solution_rejected',
                    f'servo IK solution continuity cost {continuity_cost:.3f} exceeded threshold '
                    f'{self.servo_joint_cost_threshold:.3f}',
                )
                return False

        return True

    def _publish_trajectory(self, joint_positions: List[float]) -> None:
        target_positions = self._apply_joint_step_limits(joint_positions, self.state)
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        trajectory_time = self._trajectory_time_for_state(self.state)
        point.time_from_start.sec = int(trajectory_time)
        point.time_from_start.nanosec = int((trajectory_time - int(trajectory_time)) * 1e9)

        trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)

        self.commanded_joint_positions = list(target_positions)
        self.last_accepted_joint_positions = list(target_positions)
        self.last_command_time = self.get_clock().now()
        self.awaiting_motion = True
        self._log_event('trajectory_commanded', 'published joint trajectory command')

    def _attach_battery_to_tool(self) -> bool:
        world_tool = self._lookup_frame_transform(self.tool_frame)
        world_camera = self._lookup_frame_transform(self.camera_frame)
        if world_tool is None or world_camera is None:
            return False

        if self.last_target_pose is not None and self._has_fresh_target():
            camera_to_tag = transform_from_pose(self.last_target_pose.pose)
            world_tag = compose_transforms(world_camera, camera_to_tag)
            world_battery = offset_along_local_axes(world_tag, [0.0, 0.0, -self.tag_to_battery_center])
        else:
            world_battery = offset_along_local_axes(world_tool, [0.0, 0.0, -0.06])

        self.tool_to_battery_transform = compose_transforms(invert_transform(world_tool), world_battery)
        self.battery_attached = True
        self._request_battery_pose_update()
        return True

    def _release_battery(self) -> None:
        self.battery_attached = False
        self.tool_to_battery_transform = None

    def _attach_battery_from_current_tool(self) -> bool:
        world_tool = self._lookup_frame_transform(self.tool_frame)
        if world_tool is None:
            return False

        world_battery = offset_along_local_axes(world_tool, [0.0, 0.0, -0.06])
        self.tool_to_battery_transform = compose_transforms(invert_transform(world_tool), world_battery)
        self.battery_attached = True
        self._request_battery_pose_update()
        return True

    def _update_attached_battery_pose(self) -> None:
        if not self.battery_attached or self.tool_to_battery_transform is None:
            return

        if self.pending_set_pose_future is not None:
            return

        self._request_battery_pose_update()

    def _request_battery_pose_update(self) -> None:
        if not self.set_entity_pose_client.service_is_ready():
            if not self.attach_service_warning_emitted:
                self.get_logger().warn(f'Gazebo pose service not ready: {self.set_entity_pose_service_name}')
                self.attach_service_warning_emitted = True
            return

        self.attach_service_warning_emitted = False
        world_tool = self._lookup_frame_transform(self.tool_frame)
        if world_tool is None or self.tool_to_battery_transform is None:
            return

        world_battery = compose_transforms(world_tool, self.tool_to_battery_transform)
        request = SetEntityPose.Request()
        request.entity.name = self.battery_model_name
        request.entity.type = Entity.MODEL
        request.pose = pose_from_transform(world_battery)

        self.pending_set_pose_future = self.set_entity_pose_client.call_async(request)
        self.pending_set_pose_future.add_done_callback(self._handle_set_pose_response)

    def _handle_set_pose_response(self, future) -> None:
        self.pending_set_pose_future = None
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Gazebo set_pose request failed: {exc}')
            self._log_event('set_pose_failure', f'Gazebo set_pose exception: {exc}')
            return

        if not response.success:
            self.get_logger().warn('Gazebo rejected battery pose update')
            self._log_event('set_pose_failure', 'Gazebo rejected attached battery pose update')

    def _set_state(self, new_state: ServoState, reason: str) -> None:
        if self.state == new_state:
            return

        previous_state = self.state
        previous_duration = self._state_duration_sec()
        self._log_event(
            'state_exit',
            f'{previous_state.value} -> {new_state.value}: {reason}',
            state=previous_state.value,
            stage_duration_sec=previous_duration,
        )

        self.state = new_state
        self.state_enter_time = self.get_clock().now()
        self.align_target_missing_cycles = 0
        self.align_goal_hold_start_time = None
        self.align_goal_hold_logged = False
        self.side_approach_target_missing_cycles = 0
        self.side_approach_enable_time = None
        self.side_approach_settle_logged = False
        self.awaiting_motion = False
        self.commanded_joint_positions = None
        self.last_command_time = None
        self.pending_ik_future = None
        if new_state in {ServoState.S1_SIDE_APPROACH, ServoState.S2_ALIGN, ServoState.S0_STANDBY}:
            self.path_anchor = None
        if new_state == ServoState.S0_STANDBY:
            if previous_state in {ServoState.S1_SIDE_APPROACH, ServoState.S2_ALIGN} and self.active_cycle_id != 0:
                self.active_cycle_id = 0
                self._publish_cycle_id_value(0)
                self._log_event(
                    'cycle_reset',
                    f'resetting coarse-position/servo gate after recovery from {previous_state.value}',
                )
            self.search_pose_commanded = False
            self.search_sweep_started = False
            self.search_sweep_index = 0
            self.search_sweep_steps_completed = 0
            self.search_anchor_joints = None
            self.search_use_current_anchor = previous_state in {ServoState.S1_SIDE_APPROACH, ServoState.S2_ALIGN}
            self.search_anchor_tool = None
            self.search_anchor_camera = None
            self.frozen_pre_grasp_transform = None
            self.frozen_grasp_transform = None
            self.search_ik_failure_count = 0
            self.target_confirmed = False
            self.fresh_target_cycles = 0
            self.fresh_target_hold_start_time = None
            self.standby_enter_wall_time = time.monotonic()
        self.get_logger().info(
            f'状态切换: {previous_state.value} -> {new_state.value} '
            f'(上一阶段耗时={previous_duration:.2f}s, 原因={reason})'
        )
        self._log_event('state_enter', reason)
        self._arm_debug_pause_if_needed(previous_state, new_state, reason)

    def _arm_debug_pause_if_needed(
        self,
        previous_state: ServoState,
        new_state: ServoState,
        reason: str,
    ) -> None:
        if not self.debug_pause_on_state_transition:
            return

        self.debug_waiting_for_step = True
        self.debug_wait_state = new_state
        self.debug_wait_reason = reason
        self._publish_debug_hold(True)
        self.get_logger().info(
            'Debug pause armed after state transition. '
            f'Current state={new_state.value}, previous={previous_state.value}. '
            f'Reason={reason}. Resume with: '
            f'ros2 topic pub --once {self.debug_step_topic} std_msgs/msg/String "{{data: next}}"'
        )
        self._log_event(
            'debug_step_wait',
            f'paused after transition {previous_state.value} -> {new_state.value}; '
            f'resume with topic {self.debug_step_topic}',
        )

    def _publish_debug_hold(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.debug_hold_pub.publish(msg)

    def _publish_task_frames(self) -> None:
        if not self._should_publish_target_frames():
            return

        pre_grasp_transform = self.frozen_pre_grasp_transform
        if pre_grasp_transform is None:
            pre_grasp_transform = self._compute_task_goal_transform(self.hover_distance)
        pre_grasp = self._make_pose_stamped(pre_grasp_transform)
        if pre_grasp is not None:
            self.pre_grasp_pub.publish(pre_grasp)

        grasp_transform = self.frozen_grasp_transform
        if grasp_transform is None:
            grasp_transform = self._compute_task_goal_transform(self.grasp_distance)
        grasp_target = self._make_pose_stamped(grasp_transform)
        if grasp_target is not None:
            self.grasp_target_pub.publish(grasp_target)

        place_target = self._compute_place_target_pose()
        if place_target is not None:
            self.place_target_pub.publish(place_target)

    def _compute_place_target_pose(self) -> Optional[PoseStamped]:
        task_state_offsets = {
            ServoState.S4_EXTRACT: self.extract_offset,
            ServoState.S5_SAFE_EXIT: self.safe_exit_offset,
            ServoState.S6_PLACE_OLD_BATTERY: self.old_battery_place_offset,
            ServoState.S7_TOOL_RESET: self.tool_reset_offset,
            ServoState.S8_GRASP_NEW_BATTERY: self.new_battery_pick_offset,
            ServoState.S9_MOVE_TO_DOCK: self.dock_offset,
            ServoState.S10_INSERT_NEW_BATTERY: self.insert_offset,
            ServoState.S11_RESET: self.final_reset_offset,
        }
        if self.path_anchor is not None and self.state in task_state_offsets:
            target_transform = offset_transform(self.path_anchor, task_state_offsets[self.state])
            return self._make_pose_stamped(target_transform)

        if self.last_battery_center_pose is None:
            return None

        pose = PoseStamped()
        pose.header = self.last_battery_center_pose.header
        pose.pose = self.last_battery_center_pose.pose
        pose.pose.position.y += self.old_battery_place_offset[1]
        return pose

    def _make_pose_stamped(self, transform: Optional[TransformData]) -> Optional[PoseStamped]:
        if transform is None:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.world_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = pose_from_transform(transform)
        return pose

    def _update_target_confirmation(self) -> None:
        fresh_target = self._has_fresh_target()
        bypass_search_gate = False
        required_visible_duration = self.search_lock_visible_duration
        if self.state == ServoState.S0_STANDBY:
            required_visible_duration = min(
                self.search_lock_visible_duration,
                self.initial_tag_confirm_duration,
            )

        if fresh_target:
            if self.fresh_target_hold_start_time is None:
                self.fresh_target_hold_start_time = self.get_clock().now()
            visible_duration = (
                self.get_clock().now() - self.fresh_target_hold_start_time
            ).nanoseconds / 1e9
            bypass_search_gate = visible_duration >= required_visible_duration
        else:
            if (
                self.state == ServoState.S0_STANDBY
                and not self.target_confirmed
                and self.fresh_target_hold_start_time is not None
            ):
                self._set_confirmation_gate_reason(
                    'fresh_target_lost',
                    'fresh target visibility was interrupted before confirmation completed',
                )
            self.fresh_target_hold_start_time = None
            self.search_pause_logged = False

        if (
            self.state == ServoState.S0_STANDBY
            and self.search_sweep_steps_completed < self.min_search_sweep_steps_before_lock
            and not bypass_search_gate
        ):
            if fresh_target:
                visible_duration = (
                    self.get_clock().now() - self.fresh_target_hold_start_time
                ).nanoseconds / 1e9
                remaining_duration = max(0.0, required_visible_duration - visible_duration)
                self._set_confirmation_gate_reason(
                    'waiting_bypass_visible_duration',
                    f'stable visible duration {visible_duration:.2f}s below bypass threshold; '
                    f'{remaining_duration:.2f}s remaining before search-step gate is bypassed',
                )
            else:
                self._set_confirmation_gate_reason(
                    'blocked_by_min_sweep',
                    f'search sweep steps {self.search_sweep_steps_completed}/'
                    f'{self.min_search_sweep_steps_before_lock} and target not yet fresh',
                )
            if fresh_target:
                self.fresh_target_cycles = min(self.fresh_target_cycles + 1, self.target_confirm_cycles)
            else:
                self.fresh_target_cycles = max(0, self.fresh_target_cycles - 1)
            self.target_confirmed = False
            return

        if fresh_target:
            self._clear_confirmation_gate_reason()
            self.fresh_target_cycles = min(self.fresh_target_cycles + 1, self.target_confirm_cycles)
        else:
            self._set_confirmation_gate_reason(
                'target_not_fresh',
                'target pose did not satisfy fresh receipt/stamp timeout checks',
            )
            self.fresh_target_cycles = max(0, self.fresh_target_cycles - 1)
            self.target_confirmed = False
            return

        if not self.target_confirmed and self.fresh_target_cycles >= self.target_confirm_cycles:
            self.target_confirmed = True
            self._clear_confirmation_gate_reason()
            if (
                self.state == ServoState.S0_STANDBY
                and self.search_sweep_steps_completed < self.min_search_sweep_steps_before_lock
                and bypass_search_gate
            ):
                self.get_logger().info(
                    'Tag 持续稳定可见，跳过最低扫描步数限制，直接进入粗定位'
                )
                self._log_event(
                    'target_confirmation_bypass',
                    'stable AprilTag visibility bypassed minimum search sweep steps',
                )
            self._log_event(
                'target_confirmed',
                f'AprilTag observed for {self.fresh_target_cycles} consecutive control cycles',
            )

    def _should_publish_target_frames(self) -> bool:
        return self.target_confirmed or self.state != ServoState.S0_STANDBY

    def _set_confirmation_gate_reason(self, reason: str, detail: str) -> None:
        if self.last_confirmation_gate_reason == reason:
            return
        self.last_confirmation_gate_reason = reason
        self._log_event(reason, detail)

    def _clear_confirmation_gate_reason(self) -> None:
        self.last_confirmation_gate_reason = None

    def _freeze_task_targets(self) -> None:
        if self.frozen_pre_grasp_transform is None:
            self.frozen_pre_grasp_transform = self._compute_task_goal_transform(self.hover_distance)
        if self.frozen_grasp_transform is None:
            self.frozen_grasp_transform = self._compute_task_goal_transform(self.grasp_distance)

    def _reset_search_pattern(self, reason: str) -> None:
        self.search_sweep_started = False
        self.search_pose_commanded = False
        self.search_sweep_index = 0
        self.search_sweep_steps_completed = 0
        self.search_anchor_tool = None
        self.search_anchor_camera = None
        self.search_ik_failure_count = 0
        self._log_event('search_reset', reason)

    def _log_target_feedback(self, msg: PoseStamped) -> None:
        now = self.get_clock().now()
        if self.last_target_feedback_time is not None:
            age = (now - self.last_target_feedback_time).nanoseconds / 1e9
            if age < 0.8:
                return

        world_tool = self._lookup_frame_transform(self.tool_frame)
        world_camera = self._lookup_frame_transform(self.camera_frame)
        if world_tool is None or world_camera is None:
            return

        camera_to_tag = transform_from_pose(msg.pose)
        tool_to_camera = compose_transforms(invert_transform(world_tool), world_camera)
        tool_to_tag = compose_transforms(tool_to_camera, camera_to_tag)
        _, tool_translation = tool_to_tag
        camera_translation = msg.pose.position

        self.get_logger().info(
            '检测到Tag: '
            f'相机坐标系 xyz=({camera_translation.x:.3f}, {camera_translation.y:.3f}, {camera_translation.z:.3f}), '
            f'末端坐标系 xyz=({tool_translation[0]:.3f}, {tool_translation[1]:.3f}, {tool_translation[2]:.3f})'
        )
        self.last_target_feedback_time = now

    def _start_cycle_if_needed(self, reason: str) -> None:
        if self.active_cycle_id != 0:
            return
        self.cycle_id += 1
        self.active_cycle_id = self.cycle_id
        self._log_event('cycle_started', reason, state=self.state.value)

    def _publish_cycle_id(self) -> None:
        self._publish_cycle_id_value(self.active_cycle_id)

    def _publish_cycle_id_value(self, cycle_id: int) -> None:
        msg = Int32()
        msg.data = cycle_id
        self.servo_cycle_pub.publish(msg)

    def _state_duration_sec(self) -> float:
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def _sim_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _log_event(
        self,
        event: str,
        detail: str,
        *,
        state: Optional[str] = None,
        stage_duration_sec: Optional[float] = None,
    ) -> None:
        self.event_logger.log_event(
            node='visual_servo_controller',
            sim_time_sec=self._sim_time_sec(),
            cycle_id=self.active_cycle_id,
            state=state or self.state.value,
            event=event,
            detail=detail,
            stage_duration_sec=stage_duration_sec,
            target_fresh=self._has_fresh_target(),
            servo_enabled=self.servo_enabled,
            battery_attached=self.battery_attached,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

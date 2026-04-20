import math
from enum import Enum
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


TransformData = Tuple[List[List[float]], List[float]]


class ServoState(str, Enum):
    SEARCH = 'search'
    ALIGN = 'align'
    APPROACH = 'approach'
    GRASP = 'grasp'
    LIFT = 'lift'
    TRANSFER = 'transfer'
    PLACE = 'place'
    RETREAT = 'retreat'
    DONE = 'done'


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
        self.declare_parameter('target_topic', '/target_pose')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('command_topic', '/jaka_a5_arm_controller/joint_trajectory')
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('set_entity_pose_service', '/world/battery_station/set_pose')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('tool_frame', 'J6')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('group_name', 'manipulator')
        self.declare_parameter('battery_model_name', 'battery_with_tag')
        self.declare_parameter('hover_distance', 0.20)
        self.declare_parameter('grasp_distance', 0.12)
        self.declare_parameter('xy_tolerance', 0.012)
        self.declare_parameter('z_tolerance', 0.015)
        self.declare_parameter('xy_gain', 0.7)
        self.declare_parameter('z_gain', 0.6)
        self.declare_parameter('max_servo_step', 0.025)
        self.declare_parameter('trajectory_time', 0.8)
        self.declare_parameter('joint_tolerance', 0.04)
        self.declare_parameter('command_timeout', 2.5)
        self.declare_parameter('target_timeout', 1.0)
        self.declare_parameter('lift_height', 0.10)
        self.declare_parameter('transfer_y_offset', -0.22)
        self.declare_parameter('place_drop', 0.08)
        self.declare_parameter('retreat_height', 0.10)
        self.declare_parameter('path_step', 0.03)
        self.declare_parameter('path_tolerance', 0.015)
        self.declare_parameter('tag_to_battery_center', 0.011)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.target_topic = self.get_parameter('target_topic').value
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
        self.xy_gain = float(self.get_parameter('xy_gain').value)
        self.z_gain = float(self.get_parameter('z_gain').value)
        self.max_servo_step = float(self.get_parameter('max_servo_step').value)
        self.trajectory_time = float(self.get_parameter('trajectory_time').value)
        self.joint_tolerance = float(self.get_parameter('joint_tolerance').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.lift_height = float(self.get_parameter('lift_height').value)
        self.transfer_y_offset = float(self.get_parameter('transfer_y_offset').value)
        self.place_drop = float(self.get_parameter('place_drop').value)
        self.retreat_height = float(self.get_parameter('retreat_height').value)
        self.path_step = float(self.get_parameter('path_step').value)
        self.path_tolerance = float(self.get_parameter('path_tolerance').value)
        self.tag_to_battery_center = float(self.get_parameter('tag_to_battery_center').value)

        self.state = ServoState.SEARCH
        self.current_joint_state: Dict[str, float] = {}
        self.last_target_pose: Optional[PoseStamped] = None
        self.last_target_received_time = None
        self.commanded_joint_positions: Optional[List[float]] = None
        self.last_command_time = None
        self.awaiting_motion = False
        self.pending_ik_future = None
        self.pending_set_pose_future = None
        self.path_anchor: Optional[TransformData] = None
        self.battery_attached = False
        self.tool_to_battery_transform: Optional[TransformData] = None
        self.last_requested_state: Optional[ServoState] = None
        self.service_warning_emitted = False
        self.attach_service_warning_emitted = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trajectory_pub = self.create_publisher(JointTrajectory, self.command_topic, 10)
        self.target_sub = self.create_subscription(PoseStamped, self.target_topic, self.target_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 20)

        self.ik_client = self.create_client(GetPositionIK, self.ik_service_name)
        self.set_entity_pose_client = self.create_client(SetEntityPose, self.set_entity_pose_service_name)

        self.control_timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info('IK visual servo controller started')

    def target_callback(self, msg: PoseStamped) -> None:
        self.last_target_pose = msg
        self.last_target_received_time = self.get_clock().now()

    def joint_state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.current_joint_state[name] = position

    def control_loop(self) -> None:
        self._update_attached_battery_pose()

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
            else:
                return

        if self.state == ServoState.SEARCH:
            if self._has_fresh_target():
                self._set_state(ServoState.ALIGN, 'AprilTag acquired, start closed-loop alignment')
            return

        if self.state == ServoState.ALIGN:
            if not self._has_fresh_target():
                self._set_state(ServoState.SEARCH, 'AprilTag lost during alignment')
                return

            if self._servo_to_tag(self.hover_distance):
                self._set_state(ServoState.APPROACH, 'Hover pose reached, start vertical approach')
            return

        if self.state == ServoState.APPROACH:
            if not self._has_fresh_target():
                self._set_state(ServoState.SEARCH, 'AprilTag lost during final approach')
                return

            if self._servo_to_tag(self.grasp_distance):
                self._set_state(ServoState.GRASP, 'Grasp pose reached')
            return

        if self.state == ServoState.GRASP:
            if self._attach_battery_to_tool():
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.LIFT, 'Battery attached in simulation, start lift')
            return

        if self.state == ServoState.LIFT:
            if self._drive_tool_from_anchor([0.0, 0.0, self.lift_height]):
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.TRANSFER, 'Lift complete, start transfer path')
            return

        if self.state == ServoState.TRANSFER:
            if self._drive_tool_from_anchor([0.0, self.transfer_y_offset, 0.0]):
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.PLACE, 'Transfer complete, start placement descent')
            return

        if self.state == ServoState.PLACE:
            if self._drive_tool_from_anchor([0.0, 0.0, -self.place_drop]):
                self._release_battery()
                self.path_anchor = self._lookup_frame_transform(self.tool_frame)
                self._set_state(ServoState.RETREAT, 'Battery released, retreating upward')
            return

        if self.state == ServoState.RETREAT:
            if self._drive_tool_from_anchor([0.0, 0.0, self.retreat_height]):
                self._set_state(ServoState.DONE, 'Closed-loop grasp and transfer sequence complete')
            return

    def _joint_state_ready(self) -> bool:
        return all(name in self.current_joint_state for name in self.joint_names)

    def _has_fresh_target(self) -> bool:
        if self.last_target_pose is None or self.last_target_received_time is None:
            return False

        age = (self.get_clock().now() - self.last_target_received_time).nanoseconds / 1e9
        return age <= self.target_timeout

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

    def _servo_to_tag(self, desired_distance: float) -> bool:
        if self.last_target_pose is None:
            return False

        world_tool = self._lookup_frame_transform(self.tool_frame)
        world_camera = self._lookup_frame_transform(self.camera_frame)
        if world_tool is None or world_camera is None:
            return False

        tag = self.last_target_pose.pose.position
        lateral_error = math.hypot(tag.x, tag.y)
        distance_error = tag.z - desired_distance

        if lateral_error < self.xy_tolerance and abs(distance_error) < self.z_tolerance:
            return True

        camera_delta = [
            clamp(tag.x * self.xy_gain, self.max_servo_step),
            clamp(tag.y * self.xy_gain, self.max_servo_step),
            clamp(distance_error * self.z_gain, self.max_servo_step),
        ]

        world_camera_rotation, _ = world_camera
        world_delta = apply_rotation(world_camera_rotation, camera_delta)
        desired_camera = offset_transform(world_camera, world_delta)
        tool_to_camera = compose_transforms(invert_transform(world_tool), world_camera)
        desired_tool = compose_transforms(desired_camera, invert_transform(tool_to_camera))
        return self._request_ik_for_transform(desired_tool)

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
        joint_positions = self._joint_position_vector()
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
            return

        if response.error_code.val != 1:
            self.get_logger().warn(
                f'IK solver returned error {response.error_code.val} in state {self.last_requested_state}'
            )
            return

        solution_map = {
            name: position
            for name, position in zip(response.solution.joint_state.name, response.solution.joint_state.position)
        }
        if not all(name in solution_map for name in self.joint_names):
            self.get_logger().warn('IK solution missing required manipulator joints')
            return

        ordered_positions = [solution_map[name] for name in self.joint_names]
        self._publish_trajectory(ordered_positions)

    def _publish_trajectory(self, joint_positions: List[float]) -> None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start.sec = int(self.trajectory_time)
        point.time_from_start.nanosec = int((self.trajectory_time - int(self.trajectory_time)) * 1e9)

        trajectory.points.append(point)
        self.trajectory_pub.publish(trajectory)

        self.commanded_joint_positions = list(joint_positions)
        self.last_command_time = self.get_clock().now()
        self.awaiting_motion = True

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
            return

        if not response.success:
            self.get_logger().warn('Gazebo rejected battery pose update')

    def _set_state(self, new_state: ServoState, reason: str) -> None:
        if self.state == new_state:
            return

        self.state = new_state
        self.awaiting_motion = False
        self.commanded_joint_positions = None
        self.last_command_time = None
        self.pending_ik_future = None
        if new_state in {ServoState.ALIGN, ServoState.APPROACH}:
            self.path_anchor = None
        self.get_logger().info(f'State -> {new_state.value}: {reason}')


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

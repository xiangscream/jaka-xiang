from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Int32

from jaka_a5_vision.experiment_logger import CsvExperimentLogger


class PregraspCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('pregrasp_coordinator')

        self.declare_parameter('pre_grasp_topic', '/task_frames/pre_grasp')
        self.declare_parameter('servo_enable_topic', '/visual_servo/enable')
        self.declare_parameter('servo_cycle_topic', '/visual_servo/cycle_id')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('move_action_name', '/move_action')
        self.declare_parameter('group_name', 'manipulator')
        self.declare_parameter('tool_frame', 'J6')
        self.declare_parameter('target_timeout', 1.0)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 3)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('orientation_tolerance', 0.08)
        self.declare_parameter('max_velocity_scaling_factor', 0.25)
        self.declare_parameter('max_acceleration_scaling_factor', 0.25)
        self.declare_parameter('plan_only', False)
        self.declare_parameter('retry_delay', 1.5)
        self.declare_parameter('max_moveit_failures_before_servo_fallback', 1)
        self.declare_parameter('event_log_path', '/tmp/jaka_a5_experiment_log.csv')

        self.pre_grasp_topic = str(self.get_parameter('pre_grasp_topic').value)
        self.servo_enable_topic = str(self.get_parameter('servo_enable_topic').value)
        self.servo_cycle_topic = str(self.get_parameter('servo_cycle_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        self.move_action_name = str(self.get_parameter('move_action_name').value)
        self.group_name = str(self.get_parameter('group_name').value)
        self.tool_frame = str(self.get_parameter('tool_frame').value)
        self.target_timeout = float(self.get_parameter('target_timeout').value)
        self.allowed_planning_time = float(self.get_parameter('allowed_planning_time').value)
        self.num_planning_attempts = int(self.get_parameter('num_planning_attempts').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.orientation_tolerance = float(self.get_parameter('orientation_tolerance').value)
        self.max_velocity_scaling_factor = float(self.get_parameter('max_velocity_scaling_factor').value)
        self.max_acceleration_scaling_factor = float(self.get_parameter('max_acceleration_scaling_factor').value)
        self.plan_only = bool(self.get_parameter('plan_only').value)
        self.retry_delay = float(self.get_parameter('retry_delay').value)
        self.max_moveit_failures_before_servo_fallback = max(
            1, int(self.get_parameter('max_moveit_failures_before_servo_fallback').value)
        )
        self.event_log_path = str(self.get_parameter('event_log_path').value)

        self.latest_pre_grasp: Optional[PoseStamped] = None
        self.latest_pre_grasp_time = None
        self.latest_pre_grasp_cycle_id = -1
        self.current_joint_state: Optional[JointState] = None
        self.move_group_goal_sent = False
        self.move_group_done = False
        self.goal_handle = None
        self.waiting_for_server_logged = False
        self.last_attempt_time = None
        self.consecutive_moveit_failures = 0
        self.fallback_servo_enabled = False
        self.current_cycle_id = 0
        self.last_goal_cycle_id = 0
        self.event_logger = CsvExperimentLogger(self.event_log_path)

        enable_qos = QoSProfile(depth=1)
        enable_qos.reliability = ReliabilityPolicy.RELIABLE
        enable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.servo_enable_pub = self.create_publisher(Bool, self.servo_enable_topic, enable_qos)

        self.pre_grasp_sub = self.create_subscription(PoseStamped, self.pre_grasp_topic, self.pre_grasp_callback, 10)
        self.servo_cycle_sub = self.create_subscription(Int32, self.servo_cycle_topic, self.servo_cycle_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, self.joint_state_topic, self.joint_state_callback, 20)
        self.move_group_client = ActionClient(self, MoveGroup, self.move_action_name)

        self.enable_timer = self.create_timer(0.5, self.publish_enable_state)
        self.control_timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info('Pre-grasp MoveIt coordinator started')
        self._log_event(0, 'startup', f'CSV logging enabled at {self.event_log_path}')

    def pre_grasp_callback(self, msg: PoseStamped) -> None:
        self.latest_pre_grasp = msg
        self.latest_pre_grasp_time = self.get_clock().now()
        self.latest_pre_grasp_cycle_id = self.current_cycle_id

    def joint_state_callback(self, msg: JointState) -> None:
        self.current_joint_state = msg

    def servo_cycle_callback(self, msg: Int32) -> None:
        if msg.data == self.current_cycle_id:
            return

        previous_cycle_id = self.current_cycle_id
        self.current_cycle_id = msg.data
        self.move_group_goal_sent = False
        self.move_group_done = False
        self.goal_handle = None
        self.last_attempt_time = None
        self.waiting_for_server_logged = False
        self.consecutive_moveit_failures = 0
        self.fallback_servo_enabled = False
        self.latest_pre_grasp = None
        self.latest_pre_grasp_time = None
        self.latest_pre_grasp_cycle_id = -1
        if self.current_cycle_id > 0:
            self.get_logger().info(
                f'收到新的视觉伺服周期 ID: {previous_cycle_id} -> {self.current_cycle_id}，重置粗定位协调状态'
            )
            self._log_event(
                self.current_cycle_id,
                'cycle_id_updated',
                f'coarse-position coordinator reset for new cycle {self.current_cycle_id}',
            )

    def publish_enable_state(self) -> None:
        msg = Bool()
        msg.data = self.move_group_done or self.fallback_servo_enabled
        self.servo_enable_pub.publish(msg)

    def control_loop(self) -> None:
        if self.move_group_done or self.fallback_servo_enabled or self.move_group_goal_sent:
            return

        if self.current_cycle_id <= 0:
            return

        if self.latest_pre_grasp is None or self.latest_pre_grasp_time is None:
            return

        if self.latest_pre_grasp_cycle_id != self.current_cycle_id:
            return

        age = (self.get_clock().now() - self.latest_pre_grasp_time).nanoseconds / 1e9
        if age > self.target_timeout:
            return

        if self.current_joint_state is None:
            return

        if self.last_attempt_time is not None:
            retry_age = (self.get_clock().now() - self.last_attempt_time).nanoseconds / 1e9
            if retry_age < self.retry_delay:
                return

        if not self.move_group_client.server_is_ready():
            if not self.waiting_for_server_logged:
                self.get_logger().warn(f'等待 MoveIt 动作服务就绪: {self.move_action_name}')
                self.waiting_for_server_logged = True
            return

        self.waiting_for_server_logged = False
        goal = self._build_goal(self.latest_pre_grasp)
        self.move_group_goal_sent = True
        self.last_goal_cycle_id = self.current_cycle_id
        self.last_attempt_time = self.get_clock().now()
        self.get_logger().info('发送 MoveIt 粗定位目标到 pre_grasp')
        self._log_event(self.current_cycle_id, 'moveit_goal_sent', '发送 MoveIt 粗定位目标到 pre_grasp')
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self._handle_goal_response)

    def _build_goal(self, target: PoseStamped) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = self.num_planning_attempts
        goal.request.allowed_planning_time = self.allowed_planning_time
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor
        goal.request.start_state.joint_state = self.current_joint_state
        goal.request.goal_constraints = [self._make_goal_constraints(target)]
        goal.planning_options.plan_only = self.plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        goal.planning_options.replan_delay = 0.5
        return goal

    def _make_goal_constraints(self, target: PoseStamped) -> Constraints:
        constraints = Constraints()
        constraints.name = 'pre_grasp_pose_goal'

        position_constraint = PositionConstraint()
        position_constraint.header = target.header
        position_constraint.link_name = self.tool_frame
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            max(self.position_tolerance * 2.0, 1e-4),
            max(self.position_tolerance * 2.0, 1e-4),
            max(self.position_tolerance * 2.0, 1e-4),
        ]
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(target.pose)
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target.header
        orientation_constraint.link_name = self.tool_frame
        orientation_constraint.orientation = target.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def _handle_goal_response(self, future) -> None:
        try:
            self.goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.move_group_goal_sent = False
            self.get_logger().warn(f'发送 MoveIt 目标失败: {exc}')
            self._register_moveit_failure(f'发送 MoveIt 目标失败: {exc}')
            return

        if not self.goal_handle.accepted:
            self.move_group_goal_sent = False
            self.get_logger().warn('MoveIt 拒绝了 pre_grasp 粗定位目标')
            self._register_moveit_failure('MoveIt 拒绝了 pre_grasp 粗定位目标')
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_move_result)

    def _handle_move_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.move_group_goal_sent = False
            self.get_logger().warn(f'获取 MoveIt 执行结果失败: {exc}')
            self._register_moveit_failure(f'获取 MoveIt 执行结果失败: {exc}')
            return

        if result.error_code.val == 1:
            self.move_group_done = True
            self.fallback_servo_enabled = False
            self.consecutive_moveit_failures = 0
            mode = 'plan-only' if self.plan_only else 'plan-and-execute'
            self.get_logger().info(f'MoveIt 粗定位成功（{mode}），开始使能视觉伺服')
            self._log_event(self.last_goal_cycle_id, 'moveit_success', f'MoveIt 粗定位成功（{mode}）')
            return

        self.move_group_goal_sent = False
        self.get_logger().warn(f'MoveIt 粗定位失败，错误码 {result.error_code.val}')
        self._register_moveit_failure(f'MoveIt 粗定位失败，错误码 {result.error_code.val}')

    def _register_moveit_failure(self, detail: str) -> None:
        self.consecutive_moveit_failures += 1
        self._log_event(self.last_goal_cycle_id, 'moveit_failure', detail)
        if self.consecutive_moveit_failures < self.max_moveit_failures_before_servo_fallback:
            return
        if self.fallback_servo_enabled:
            return

        self.fallback_servo_enabled = True
        self.move_group_goal_sent = False
        self.get_logger().warn(
            'MoveIt 粗定位连续失败，降级为直接视觉伺服（跳过粗定位）'
        )
        self._log_event(
            self.last_goal_cycle_id,
            'moveit_fallback_enable_servo',
            'MoveIt coarse positioning failed repeatedly; enabling direct visual servo fallback',
        )

    def _sim_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _log_event(self, cycle_id: int, event: str, detail: str) -> None:
        self.event_logger.log_event(
            node='pregrasp_coordinator',
            sim_time_sec=self._sim_time_sec(),
            cycle_id=cycle_id,
            state='pregrasp',
            event=event,
            detail=detail,
            target_fresh=self.latest_pre_grasp is not None,
            servo_enabled=(self.move_group_done or self.fallback_servo_enabled),
            battery_attached=None,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PregraspCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

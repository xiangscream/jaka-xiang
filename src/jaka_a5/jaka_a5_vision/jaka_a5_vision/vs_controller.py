import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np


class VisualServoController(Node):
    def __init__(self):
        super().__init__('visual_servo_controller')

        # Subscriptions
        self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_callback,
            10
        )

        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/trajectory',
            10
        )

        # Parameters
        self.declare_parameter('position_threshold', 0.005)  # 5mm
        self.declare_parameter('max_step', 0.02)  # 20mm per step
        self.declare_parameter('desired_z', 0.1)  # 目标高度 100mm

        self.target_pose = None
        self.current_joint_positions = [0.0] * 6
        self.joint_name_to_idx = {
            'joint_1': 0, 'joint_2': 1, 'joint_3': 2,
            'joint_4': 3, 'joint_5': 4, 'joint_6': 5
        }

    def joint_states_callback(self, msg):
        # Update current joint positions from actual robot state
        for i, name in enumerate(msg.name):
            if name in self.joint_name_to_idx:
                idx = self.joint_name_to_idx[name]
                self.current_joint_positions[idx] = msg.position[i]

    def target_callback(self, msg):
        self.target_pose = msg
        self.get_logger().info(
            f'Target: x={msg.pose.position.x:.3f} '
            f'y={msg.pose.position.y:.3f} '
            f'z={msg.pose.position.z:.3f}'
        )

        # Check if target is close enough
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
        """Compute delta joint positions based on visual error"""
        if self.target_pose is None:
            return [0.0] * 6

        x = self.target_pose.pose.position.x
        y = self.target_pose.pose.position.y
        z = self.target_pose.pose.position.z
        desired_z = self.get_parameter('desired_z').value
        max_step = self.get_parameter('max_step').value

        # Scale factor (needs tuning)
        scale = 0.5

        delta = [
            scale * x,
            scale * y,
            scale * (z - desired_z),
            0.0, 0.0, 0.0
        ]

        # Clip to max step
        for i in range(3):
            if abs(delta[i]) > max_step:
                delta[i] = np.sign(delta[i]) * max_step

        return delta

    def send_servo_command(self):
        delta = self.compute_delta()
        new_joints = [
            curr + d for curr, d in zip(self.current_joint_positions, delta)
        ]

        traj = JointTrajectory()
        traj.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        point = JointTrajectoryPoint()
        point.positions = new_joints
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.current_joint_positions = new_joints

    def send_stop(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]
        point = JointTrajectoryPoint()
        point.positions = self.current_joint_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0

        traj.points = [point]
        self.trajectory_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    controller = VisualServoController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

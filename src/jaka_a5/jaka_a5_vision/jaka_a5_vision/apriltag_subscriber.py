import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class AprilTagSubscriber(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_subscriber')

        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('source_frame', 'camera_optical_frame')
        self.declare_parameter('tag_family', '36h11')
        self.declare_parameter('target_tag_id', 1)
        self.declare_parameter('target_tag_frame', '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('transform_timeout', 0.2)

        self.target_pose_topic = str(self.get_parameter('target_pose_topic').value)
        self.source_frame = str(self.get_parameter('source_frame').value)
        self.tag_family = str(self.get_parameter('tag_family').value)
        self.target_tag_id = int(self.get_parameter('target_tag_id').value)
        self.target_tag_frame = str(self.get_parameter('target_tag_frame').value).strip()
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.transform_timeout = float(self.get_parameter('transform_timeout').value)

        if not self.target_tag_frame:
            self.target_tag_frame = f'tag{self.tag_family}:{self.target_tag_id}'

        self.publisher = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform_timeout_duration = Duration(seconds=self.transform_timeout)

        timer_period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_target_pose)
        self.last_publish_logged = False
        self.last_missing_logged = False

        self.get_logger().info(
            f'Publishing {self.source_frame} -> {self.target_tag_frame} to {self.target_pose_topic}'
        )

    def publish_target_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_tag_frame,
                rclpy.time.Time(),
                timeout=self.transform_timeout_duration,
            )
        except TransformException as exc:
            if not self.last_missing_logged:
                self.get_logger().warn(
                    f'Waiting for transform {self.source_frame} -> {self.target_tag_frame}: {exc}'
                )
                self.last_missing_logged = True
            self.last_publish_logged = False
            return

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.publisher.publish(pose)

        if not self.last_publish_logged:
            self.get_logger().info(
                'Publishing target pose from TF: '
                f'frame={pose.header.frame_id}, '
                f'x={pose.pose.position.x:.3f}, '
                f'y={pose.pose.position.y:.3f}, '
                f'z={pose.pose.position.z:.3f}'
            )
            self.last_publish_logged = True

        self.last_missing_logged = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

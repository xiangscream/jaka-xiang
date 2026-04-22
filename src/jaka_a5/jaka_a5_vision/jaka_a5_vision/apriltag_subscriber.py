import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class AprilTagSubscriber(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_subscriber')

        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('tag_pose_camera_topic', '/tag_pose_camera')
        self.declare_parameter('tag_pose_world_topic', '/tag_pose_world')
        self.declare_parameter('battery_center_topic', '/battery_center')
        self.declare_parameter('source_frame', 'camera_optical_frame')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('tag_family', '36h11')
        self.declare_parameter('target_tag_id', 1)
        self.declare_parameter('target_tag_frame', '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('transform_timeout', 0.2)
        self.declare_parameter('tag_to_battery_center', 0.011)
        self.declare_parameter('max_pose_age', 1.5)

        self.target_pose_topic = str(self.get_parameter('target_pose_topic').value)
        self.tag_pose_camera_topic = str(self.get_parameter('tag_pose_camera_topic').value)
        self.tag_pose_world_topic = str(self.get_parameter('tag_pose_world_topic').value)
        self.battery_center_topic = str(self.get_parameter('battery_center_topic').value)
        self.source_frame = str(self.get_parameter('source_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.tag_family = str(self.get_parameter('tag_family').value)
        self.target_tag_id = int(self.get_parameter('target_tag_id').value)
        self.target_tag_frame = str(self.get_parameter('target_tag_frame').value).strip()
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.transform_timeout = float(self.get_parameter('transform_timeout').value)
        self.tag_to_battery_center = float(self.get_parameter('tag_to_battery_center').value)
        self.max_pose_age = float(self.get_parameter('max_pose_age').value)

        if not self.target_tag_frame:
            self.target_tag_frame = f'tag{self.tag_family}:{self.target_tag_id}'

        self.target_pose_publisher = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.tag_pose_camera_publisher = self.create_publisher(PoseStamped, self.tag_pose_camera_topic, 10)
        self.tag_pose_world_publisher = self.create_publisher(PoseStamped, self.tag_pose_world_topic, 10)
        self.battery_center_publisher = self.create_publisher(PoseStamped, self.battery_center_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform_timeout_duration = Duration(seconds=self.transform_timeout)

        timer_period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_target_pose)
        self.last_publish_logged = False
        self.last_missing_logged = False
        self.last_stale_logged = False

        self.get_logger().info(
            'Publishing semantic AprilTag poses: '
            f'{self.source_frame} -> {self.target_tag_frame} to '
            f'{self.target_pose_topic}, {self.tag_pose_camera_topic}, {self.tag_pose_world_topic}'
        )

    def publish_target_pose(self) -> None:
        try:
            camera_transform = self.tf_buffer.lookup_transform(
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

        camera_pose = PoseStamped()
        camera_pose.header = camera_transform.header
        camera_pose.pose.position.x = camera_transform.transform.translation.x
        camera_pose.pose.position.y = camera_transform.transform.translation.y
        camera_pose.pose.position.z = camera_transform.transform.translation.z
        camera_pose.pose.orientation = camera_transform.transform.rotation

        transform_time = Time.from_msg(camera_transform.header.stamp)
        transform_age = (self.get_clock().now() - transform_time).nanoseconds / 1e9
        if transform_age > self.max_pose_age:
            if not self.last_stale_logged:
                self.get_logger().warn(
                    f'Skipping stale AprilTag transform at t={camera_transform.header.stamp.sec}.'
                    f'{camera_transform.header.stamp.nanosec:09d}; age={transform_age:.3f}s'
                )
                self.last_stale_logged = True
            self.last_publish_logged = False
            return

        self.target_pose_publisher.publish(camera_pose)
        self.tag_pose_camera_publisher.publish(camera_pose)

        world_pose = self._lookup_pose(self.world_frame, self.target_tag_frame, transform_time)
        if world_pose is not None:
            self.tag_pose_world_publisher.publish(world_pose)
            battery_center_pose = self._offset_pose_along_local_z(world_pose, -self.tag_to_battery_center)
            self.battery_center_publisher.publish(battery_center_pose)

        if not self.last_publish_logged:
            self.get_logger().info(
                'Publishing target pose from TF: '
                f'camera_frame={camera_pose.header.frame_id}, '
                f'x={camera_pose.pose.position.x:.3f}, '
                f'y={camera_pose.pose.position.y:.3f}, '
                f'z={camera_pose.pose.position.z:.3f}'
            )
            self.last_publish_logged = True

        self.last_missing_logged = False
        self.last_stale_logged = False

    def _lookup_pose(self, target_frame: str, source_frame: str, query_time: Time) -> PoseStamped | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                query_time,
                timeout=self.transform_timeout_duration,
            )
        except TransformException as exc:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=self.transform_timeout_duration,
                )
            except TransformException:
                self.get_logger().warn(
                    f'Waiting for transform {target_frame} -> {source_frame}: {exc}',
                    throttle_duration_sec=2.0,
                )
                return None

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _offset_pose_along_local_z(self, pose: PoseStamped, distance: float) -> PoseStamped:
        quat = pose.pose.orientation
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        rotation = [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]

        offset = [rotation[0][2] * distance, rotation[1][2] * distance, rotation[2][2] * distance]

        offset_pose = PoseStamped()
        offset_pose.header = pose.header
        offset_pose.pose.position.x = pose.pose.position.x + offset[0]
        offset_pose.pose.position.y = pose.pose.position.y + offset[1]
        offset_pose.pose.position.z = pose.pose.position.z + offset[2]
        offset_pose.pose.orientation = pose.pose.orientation
        return offset_pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

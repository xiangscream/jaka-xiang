import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped


class AprilTagSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_subscriber')
        self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )
        self.publisher = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        self.target_tag_id = 1

    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id[0] == self.target_tag_id:
                pose = PoseStamped()
                pose.header = detection.header
                pose.pose = detection.pose.pose
                self.publisher.publish(pose)
                self.get_logger().info(f'Tag {self.target_tag_id} detected at '
                                       f'x={pose.pose.position.x:.3f} '
                                       f'y={pose.pose.position.y:.3f} '
                                       f'z={pose.pose.position.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

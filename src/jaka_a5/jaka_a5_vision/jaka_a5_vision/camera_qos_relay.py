import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class CameraQosRelay(Node):
    def __init__(self):
        super().__init__('camera_qos_relay')

        self.declare_parameter('input_image_topic', '/camera/image_raw')
        self.declare_parameter('input_camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_image_topic', '/relay/image_raw')
        self.declare_parameter('output_camera_info_topic', '/relay/camera_info')

        image_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('input_camera_info_topic').get_parameter_value().string_value
        output_image_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value
        output_camera_info_topic = self.get_parameter('output_camera_info_topic').get_parameter_value().string_value

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._latest_camera_info = None
        self._image_pub = self.create_publisher(Image, output_image_topic, sensor_qos)
        self._camera_info_pub = self.create_publisher(CameraInfo, output_camera_info_topic, sensor_qos)

        self.create_subscription(Image, image_topic, self._image_callback, reliable_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, reliable_qos)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _image_callback(self, msg: Image) -> None:
        self._image_pub.publish(msg)

        if self._latest_camera_info is None:
            return

        synced_camera_info = copy.deepcopy(self._latest_camera_info)
        synced_camera_info.header = msg.header
        self._camera_info_pub.publish(synced_camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraQosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

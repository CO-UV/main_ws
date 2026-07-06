import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class ImageDecompressorNode(Node):
    def __init__(self):
        super().__init__("image_decompressor_node")

        self.declare_parameter("input_topic", "/uav/camera/color/image_raw/compressed")
        self.declare_parameter("output_topic", "/main/uav/camera/color/image_raw")
        self.declare_parameter("encoding", "bgr8")  # depth는 "16UC1"

        self.encoding = self.get_parameter("encoding").value
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image, self.get_parameter("output_topic").value, qos_profile_sensor_data
        )
        self.create_subscription(
            CompressedImage, self.get_parameter("input_topic").value, self._on_compressed, 10
        )

    def _on_compressed(self, msg):
        flag = cv2.IMREAD_UNCHANGED if self.encoding == "16UC1" else cv2.IMREAD_COLOR
        decoded = cv2.imdecode(np.frombuffer(msg.data, np.uint8), flag)
        if decoded is None:
            self.get_logger().warning("Failed to decode compressed frame", throttle_duration_sec=2.0)
            return

        image = self.bridge.cv2_to_imgmsg(decoded, encoding=self.encoding)
        image.header = msg.header
        self.publisher.publish(image)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDecompressorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

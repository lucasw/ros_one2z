"""
Lucas Walter
September 2023

Generate images with some moving graphics in them (e.g. a circle drawn with opencv that bounces around)
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ProcessImage(Node):
    def __init__(self):
        super().__init__("process_image")
        self.publisher = self.create_publisher(Image, "modified_image", 4)

        self.cv_bridge = CvBridge()

        self.subscriber = self.create_subscription(Image, "image", self.image_callback, 4)

    def image_callback(self, msg):
        t0 = self.get_clock().now()
        image_np = self.cv_bridge.imgmsg_to_cv2(msg)
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        # image_np = cv2.adaptiveThreshold(image_np, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
        #                                 cv2.THRESH_BINARY, 11, 2)
        rv, image_np = cv2.threshold(image_np, 127, 255, cv2.THRESH_BINARY_INV)

        modified_msg = self.cv_bridge.cv2_to_imgmsg(image_np, encoding="passthrough")
        modified_msg.header = msg.header
        modified_msg.encoding = "mono8"
        self.publisher.publish(modified_msg)

        # nanoseconds / 1e9 instead of to_sec()
        t1 = self.get_clock().now()
        self.get_logger().info(f"{(t1 - t0).nanoseconds / 1e9:0.3f}s", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ProcessImage()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

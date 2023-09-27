"""
Lucas Walter
September 2023

Generate images with some moving graphics in them (e.g. a circle drawn with opencv that bounces around)
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class GenerateImage(Node):
    def __init__(self):
        super().__init__("generate_image")
        self.publisher = self.create_publisher(Image, "image", 4)

        self.width = 2048
        self.height = 1024
        self.cv_bridge = CvBridge()

        period = 0.1  # seconds
        self.timer = self.create_timer(period, self.update)

    def update(self):
        image_np = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        msg = self.cv_bridge.cv2_to_imgmsg(image_np, encoding="passthrough")
        msg.encoding = "rgb8"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GenerateImage()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

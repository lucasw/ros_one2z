"""
Lucas Walter
September 2023

Generate images with some moving graphics in them (e.g. a circle drawn with opencv that bounces around)
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def update_value(value: float, velocity: float, min_value: float, max_value: float, margin: float):
    """
    bounce when reaching limit
    """
    value += velocity

    min_value = min_value + margin
    if value <= min_value:
        value = min_value
        velocity *= -1

    max_value = max_value - margin
    if value >= max_value:
        value = max_value
        velocity *= -1

    return value, velocity


class BouncingBall:
    def __init__(self):
        self.width = 2048
        self.height = 1024
        self.radius = 64
        self.x = 100
        self.y = 120
        self.vel_x = 14
        self.vel_y = 30

    def update(self) -> np.array:
        image_np = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        cv2.circle(image_np, (self.x, self.y), radius=self.radius,
                   color=(200, 190, 180), thickness=-1)
        self.x, self.vel_x = update_value(self.x, self.vel_x, 0, self.width, self.radius)
        self.y, self.vel_y = update_value(self.y, self.vel_y, 0, self.height, self.radius)
        return image_np


class GenerateImage(Node):
    def __init__(self):
        super().__init__("generate_image")
        self.publisher = self.create_publisher(Image, "image", 4)

        self.cv_bridge = CvBridge()
        self.bouncing_ball = BouncingBall()

        period = 0.0333  # seconds
        self.timer = self.create_timer(period, self.update)

    def update(self):
        t0 = self.get_clock().now()
        image_np = self.bouncing_ball.update()
        msg = self.cv_bridge.cv2_to_imgmsg(image_np, encoding="passthrough")
        msg.encoding = "rgb8"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        t1 = self.get_clock().now()
        # TODO(lucasw) is there a to_sec()?
        self.get_logger().info(f"{(t1 - t0)}", throttle_duration_sec=2.0)


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

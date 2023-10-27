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
    def __init__(self, logger: rclpy.impl.rcutils_logger.RcutilsLogger,
                 width=540, height=480, radius=32, x=50, y=50, vel_x=8, vel_y=16):
        self.width = width
        self.height = height
        self.radius = radius
        self.x = x
        self.y = y
        self.vel_x = vel_x
        self.vel_y = vel_y
        logger.info(f"{width}x{height} {radius}, {x} {y}, {vel_x} {vel_y}")

    def update(self) -> np.array:
        image_np = 255 * np.ones((self.height, self.width, 3), dtype=np.uint8)

        cv2.circle(image_np, (self.x, self.y), radius=self.radius,
                   color=(100, 90, 80), thickness=-1)
        self.x, self.vel_x = update_value(self.x, self.vel_x, 0, self.width, self.radius)
        self.y, self.vel_y = update_value(self.y, self.vel_y, 0, self.height, self.radius)
        return image_np


class GenerateImage(Node):
    def __init__(self):
        super().__init__("generate_image")

        self.declare_parameter("width", 2048)
        self.declare_parameter("height", 1024)
        self.declare_parameter("radius", 64)
        self.declare_parameter("x", 100)
        self.declare_parameter("y", 120)
        self.declare_parameter("vel_x", 10)
        self.declare_parameter("vel_y", 14)

        self.publisher = self.create_publisher(Image, "image", 4)

        self.cv_bridge = CvBridge()
        self.bouncing_ball = BouncingBall(
            logger=self.get_logger(),
            width=self.get_parameter("width").value,
            height=self.get_parameter("height").value,
            radius=self.get_parameter("radius").value,
            x=self.get_parameter("x").value,
            y=self.get_parameter("y").value,
            vel_x=self.get_parameter("vel_x").value,
            vel_y=self.get_parameter("vel_y").value,
        )

        period = 0.0333  # seconds
        self.timer = self.create_timer(period, self.update)

    def update(self):
        t0 = self.get_clock().now()
        image_np = self.bouncing_ball.update()
        msg = self.cv_bridge.cv2_to_imgmsg(image_np, encoding="passthrough")
        msg.encoding = "rgb8"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self.publisher.publish(msg)
        # nanoseconds / 1e9 instead of to_sec()
        t1 = self.get_clock().now()
        self.get_logger().info(f"{(t1 - t0).nanoseconds / 1e9:0.3f}s", throttle_duration_sec=2.0)


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

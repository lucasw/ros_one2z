"""
Lucas Walter
September 2023

Generate images with some moving graphics in them (e.g. a circle drawn with opencv that bounces around)
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from geometry_msgs.msg import (
    Point32,
    PolygonStamped,
)
from sensor_msgs.msg import Image


class ImageToContour(Node):
    def __init__(self):
        super().__init__("image_to_contour")
        self.publisher = self.create_publisher(PolygonStamped, "contour", 4)

        self.cv_bridge = CvBridge()

        self.subscriber = self.create_subscription(Image, "image", self.image_callback, 4)

    def image_callback(self, msg):
        t0 = self.get_clock().now()
        image_np = self.cv_bridge.imgmsg_to_cv2(msg)
        if len(image_np.shape) > 2:
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

        contours, hierarchy = cv2.findContours(image_np, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        polygon = PolygonStamped()
        polygon.header = msg.header

        for contour in contours:
            for i in range(contour.shape[0]):
                sc = 0.003
                x = contour[i, 0, 0] * sc
                y = contour[i, 0, 1] * sc
                pt = Point32(x=x, y=y)
                polygon.polygon.points.append(pt)

        self.publisher.publish(polygon)

        # nanoseconds / 1e9 instead of to_sec()
        t1 = self.get_clock().now()
        self.get_logger().info(f"{(t1 - t0).nanoseconds / 1e9:0.3f}s", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ImageToContour()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python
"""
Lucas Walter
September 2023

Generate images with some moving graphics in them (e.g. a circle drawn with opencv that bounces around)
"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


# TODO(lucasw) how to move this into location that both ros1 and ros2 can use?
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


class GenerateImage:
    def __init__(self):
        self.publisher = rospy.Publisher("image", Image, queue_size=4)

        self.cv_bridge = CvBridge()
        self.bouncing_ball = BouncingBall()

        period = 0.0333  # seconds
        self.timer = rospy.Timer(rospy.Duration(period), self.update)

    def update(self, event: rospy.timer.TimerEvent):
        t0 = rospy.Time.now()
        image_np = self.bouncing_ball.update()
        msg = self.cv_bridge.cv2_to_imgmsg(image_np, encoding="passthrough")
        msg.encoding = "rgb8"
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        self.publisher.publish(msg)
        t1 = rospy.Time.now()
        rospy.loginfo_throttle(2.0, f"{(t1 - t0).to_sec():0.3f}s")


def main():
    rospy.init_node("image_to_contour")
    _ = GenerateImage()
    rospy.spin()


if __name__ == "__main__":
    main()

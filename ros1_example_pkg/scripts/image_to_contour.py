#!/usr/bin/env python
"""
Lucas Walter
September 2023

Generate images with some moving graphics in them (e.g. a circle drawn with opencv that bounces around)
"""

import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Point32,
    PolygonStamped,
)
from sensor_msgs.msg import Image


class ImageToContour():
    def __init__(self):
        self.publisher = rospy.Publisher("contour", PolygonStamped, queue_size=4)

        self.cv_bridge = CvBridge()

        self.subscriber = rospy.Subscriber("image", Image, self.image_callback, queue_size=4)

    def image_callback(self, msg):
        t0 = rospy.Time.now()
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

        t1 = rospy.Time.now()
        rospy.loginfo_throttle(2.0, f"{(t1 - t0).to_sec():0.3f}s")


def main():
    rospy.init_node("generate_image")
    _ = ImageToContour()
    rospy.spin()


if __name__ == "__main__":
    main()

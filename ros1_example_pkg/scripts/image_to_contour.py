#!/usr/bin/env python
"""
Lucas Walter
September 2023

Generate images with some moving graphics in them (e.g. a circle drawn with opencv that bounces around)
"""

import rospy
from bouncing_ball import image_to_contour
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import Image


class ImageToContour():
    def __init__(self):
        self.publisher = rospy.Publisher("contour", PolygonStamped, queue_size=4)

        self.cv_bridge = CvBridge()

        self.subscriber = rospy.Subscriber("image", Image, self.image_callback, queue_size=4)
        rospy.loginfo("image to contour")

    def image_callback(self, image_msg: Image):
        t0 = rospy.Time.now()
        image_np = self.cv_bridge.imgmsg_to_cv2(image_msg)
        polygon = image_to_contour(image_msg.header, image_np)
        self.publisher.publish(polygon)

        t1 = rospy.Time.now()
        age = t0 - image_msg.header.stamp
        rospy.loginfo_throttle(1.0, f"msg {age.to_sec():0.3f}s old, processing {(t1 - t0).to_sec():0.3f}s")


def main():
    rospy.init_node("image_to_contour")
    _ = ImageToContour()
    rospy.spin()


if __name__ == "__main__":
    main()

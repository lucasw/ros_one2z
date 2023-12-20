#!/usr/bin/env python
"""
Lucas Walter
September 2023

Generate images with some moving graphics in them (e.g. a circle drawn with opencv that bounces around)
"""

import rospy
from bouncing_ball import BouncingBall
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class GenerateImage:
    def __init__(self):
        self.publisher = rospy.Publisher("image", Image, queue_size=4)

        self.cv_bridge = CvBridge()

        width = rospy.get_param("~width", 540)
        height = rospy.get_param("~height", 480)
        radius = rospy.get_param("~radius", 8)
        self.bouncing_ball = BouncingBall(loginfo=rospy.loginfo, width=width, height=height, radius=radius)

        period = rospy.get_param("~period", 0.0333)
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
        text = f"image {image_np.shape} {len(msg.data)} update {(t1 - t0).to_sec():0.3f}s"
        rospy.loginfo_throttle(2.0, text)


def main():
    rospy.init_node("image_to_contour")

    _ = GenerateImage()
    rospy.spin()


if __name__ == "__main__":
    main()

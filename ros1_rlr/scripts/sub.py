#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped


rospy.init_node("sub")


def callback(msg: PointStamped | rospy.AnyMsg):
    print(dir(msg))


# sub = rospy.Subscriber("point", PointStamped, callback, queue_size=2)
sub = rospy.Subscriber("point", rospy.AnyMsg, callback, queue_size=2)
rospy.spin()

#!/usr/bin/env python
# Copyright (c) 2022 Lucas Walter
#
# subscribe to multiple topics with headers and average their age with

import datetime
import sys
from threading import Lock

import numpy as np
import rclpy
from ros2topic.api import get_msg_class


class MsgAge(Node):
    def __init__(self, args):
        topics = []
        for arg in args:
            if arg[0] == '_':
                continue
            topics.append(arg)

        rospy.loginfo(f"getting age of topics: {topics}")
        self.verbose = rospy.get_param("~verbose", False)

        self.lock = Lock()

        self.LOCAL_TIMEZONE = datetime.datetime.now(datetime.timezone.utc).astimezone().tzinfo
        rospy.loginfo(self.LOCAL_TIMEZONE)
        utc_dt = datetime.datetime.now(datetime.timezone.utc)
        print("Local time {}".format(utc_dt.astimezone()))

        self.sync_sub = None
        self.topics = {}
        self.topic_types = {}
        self.classes = {}
        self.subs = {}
        self.ages = {}
        for index in range(len(topics)):
            self.topics[index] = topics[index]
            self.subs[index] = rospy.Subscriber(self.topics[index], rospy.AnyMsg,
                                                self.init_callback,
                                                callback_args=(index),
                                                queue_size=1)
            self.ages[index] = []

        period = rospy.get_param("~period", 1.0)

        self.timer = rospy.Timer(rospy.Duration(period), self.update)

    def init_callback(self, msg, args):
        index = args  # args[0]
        topic_type = msg._connection_header['type']
        self.topic_types[index] = topic_type
        self.subs[index].unregister()

        # TODO(lucasw) reject if message doesn't have a header
        topic_class = get_message_class(topic_type)
        rospy.loginfo(f"{index} found class for '{topic_type}': {topic_class}")
        self.subs[index] = rospy.Subscriber(self.topics[index], topic_class,
                                            self.callback,
                                            callback_args=(index),
                                            queue_size=10)

    def callback(self, msg, args):
        cur = rospy.Time.now()
        if self.verbose:
            cur_date = datetime.datetime.fromtimestamp(cur.to_sec())
            msg_date = datetime.datetime.fromtimestamp(msg.header.stamp.to_sec())
            # TODO(lucasw) how long is it from calling this to characters appearing on screen?
            print(f"{cur_date} | {msg_date} | {(cur - msg.header.stamp).to_sec():0.06f}s", flush=True)
        index = args
        age = cur - msg.header.stamp
        # TODO(lucasw) do something if age is negative?  Want to avoid looping bags messing these up
        if age < rospy.Duration(-5.0):
            return
        with self.lock:
            self.ages[index].append(age.to_sec())

    def update(self, event):
        text = "ages:"
        with self.lock:
            for index, ages in self.ages.items():
                if index not in self.topic_types.keys():
                    continue
                if len(self.topics) <= index or len(self.topic_types) <= index:
                    rospy.logwarn(f"{index} {self.topics} {self.topic_types}")
                    return
                if len(ages) == 0:
                    age_str = "no messages"
                else:
                    age_str = f" average age {np.mean(ages):0.4f}s over {len(ages)} messages"
                text += f"\n{index} {self.topics[index]} {self.topic_types[index]} {age_str}"
                self.ages[index] = []
        rospy.loginfo(text)


def main(args=None):
    rclpy.init(args=args)
    node = MsgAge()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

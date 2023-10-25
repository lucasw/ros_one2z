#!/usr/bin/env python
"""
Copyright (c) 2022 Lucas Walter

subscribe to multiple topics with headers and average their age with

ros2 run ros2_example_pkg msg_age /contour
"""

import datetime
import sys
from threading import Lock

import numpy as np
import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class


class MsgAge(Node):
    def __init__(self, args):
        super().__init__("msg_age")

        self.declare_parameter("verbose", False)
        self.declare_parameter("period", 1.0)

        topics = []
        for arg in args[1:]:
            if arg[0] == '_':
                continue
            topics.append(arg)

        self.get_logger().info(f"getting age of topic/s: {topics}")

        self.verbose = self.get_parameter("verbose").value
        self.get_logger().info(f"verbose: {self.verbose}")

        self.lock = Lock()

        self.LOCAL_TIMEZONE = datetime.datetime.now(datetime.timezone.utc).astimezone().tzinfo
        self.get_logger().info(f"{self.LOCAL_TIMEZONE}")
        utc_dt = datetime.datetime.now(datetime.timezone.utc)
        print("Local time {}".format(utc_dt.astimezone()))

        self.sync_sub = None
        self.topics = {}
        self.topic_types = {}
        self.classes = {}
        self.subs = {}
        self.ages = {}
        for index, topic in enumerate(topics):
            self.topics[index] = topic
            self.ages[index] = []

            topic_class = get_msg_class(self, topic, blocking=True)
            if topic_class is None:
                self.get_logger().warn(f"couldn't get type of topic '{topic}'")
                continue
            self.topic_types[index] = topic_class
            self.get_logger().info(f"{index} found class for '{topic}': {topic_class}")

            def callback(msg):
                return self.callback(msg, index)
            self.subs[index] = self.create_subscription(topic_class, topic,
                                                        callback,
                                                        qos_profile=10)

        period = self.get_parameter("period").value
        self.get_logger().info(f"update period: {period}s")
        self.timer = self.create_timer(period, self.update)

    def callback(self, msg, index):
        cur_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        age = cur_time - msg_time
        if self.verbose:
            cur_date = datetime.datetime.fromtimestamp(cur_time.nanoseconds / 1e9)
            msg_date = datetime.datetime.fromtimestamp(msg_time.nanoseconds / 1e9)
            # TODO(lucasw) how long is it from calling this to characters appearing on screen?
            print(f"{cur_date} | {msg_date} | {age.nanoseconds / 1e9:0.06f}s", flush=True)
        # TODO(lucasw) do something if age is negative?  Want to avoid looping bags messing these up
        if age < rclpy.duration.Duration(seconds=-5.0):
            return
        with self.lock:
            self.ages[index].append(age.nanoseconds / 1e9)

    def update(self):
        text = "ages:"
        with self.lock:
            for index, ages in self.ages.items():
                if index not in self.topic_types.keys():
                    continue
                if len(self.topics) <= index or len(self.topic_types) <= index:
                    self.get_logger().warn(f"{index} {self.topics} {self.topic_types}")
                    return
                if len(ages) == 0:
                    age_str = "no messages"
                else:
                    age_str = f" average age {np.mean(ages):0.4f}s over {len(ages)} messages"
                text += f"\n{index} {self.topics[index]} {self.topic_types[index]} {age_str}"
                self.ages[index] = []
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = MsgAge(sys.argv)
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

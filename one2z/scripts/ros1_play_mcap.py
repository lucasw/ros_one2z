#!/usr/bin/env python
"""
load a ros1 mcap file and play messages out of it
"""

import sys

import rospy
from mcap.reader import make_reader
from mcap_ros1.decoder import DecoderFactory
from roslib.message import get_message_class
from rospy.msg import AnyMsg


def main():
    rospy.init_node("play_mcap")
    with open(sys.argv[1], "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        pubs = {}
        for key, channel in summary.channels.items():
            print(channel.topic)
            # pubs[channel.topic] = None

        t0 = None
        t_old = None
        # iter_messages -> don't decode
        for ind, (schema, channel, message) in enumerate(reader.iter_messages()):
            # message.data has the encoded message

            if rospy.is_shutdown():
                break

            t_cur = message.log_time / 1e9
            if t0 is None:
                t0 = t_cur
                rospy.loginfo(f"{t0:0.3f}s")
                t_old = t0

            msg_type = get_message_class(schema.name)
            if channel.topic not in pubs:
                pubs[channel.topic] = rospy.Publisher(channel.topic, msg_type, queue_size=3)
            # second part of avoided decoding, is avoiding encoding via AnyMsg
            msg = AnyMsg()
            msg._buff = message.data
            pubs[channel.topic].publish(msg)

            dt = t_cur - t_old
            rospy.sleep(dt)
            t_old = t_cur


if __name__ == "__main__":
    main()

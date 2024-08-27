#!/usr/bin/env python
"""
load a ros1 mcap file and play messages out of it

pip install mcap mcap-protobuf-support mcap-ros1-support

rosrun one2z ros1_play_mcap.py demo_2023-11-18_06-06-59_image_only.mcap

"""

import sys
import time

import rospy
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory as ProtobufDecoderFactory
from mcap_ros1.decoder import DecoderFactory
from roslib.message import get_message_class
from rosgraph_msgs.msg import Clock
from rospy.msg import AnyMsg
# from tf2_msgs.msg import TFMessage


def main():
    rospy.init_node("play_mcap")
    msg_classes = {}
    count = 0
    while not rospy.is_shutdown():
        rate = rospy.get_param("~rate", 1.0)
        play_mcap(sys.argv[1], msg_classes, rate)
        rospy.loginfo(f"looping {count}")
        count += 1
        time.sleep(1.0)


def play_mcap(name: str, msg_classes, rate=1.0):
    # TODO(lucasw) make optional
    clock_pub = rospy.Publisher("/clock", Clock, queue_size=3)
    last_clock_msg = None

    # TODO(lucasw) just going to publisher tf static over and over, later decode and make a proper aggregated
    # TFMessage
    tf_static_messages = []
    last_tf_static_stamp = rospy.Time(0)

    decoder = DecoderFactory()
    protobuf_decoder = ProtobufDecoderFactory()
    with open(name, "rb") as f:
        reader = make_reader(f, decoder_factories=[decoder])
        schemas = reader.get_summary().schemas
        for schema_id, schema in schemas.items():
            rospy.loginfo(f"encoding: {schema.name} {schema.encoding}")

        summary = reader.get_summary()
        pubs = {}
        for key, channel in summary.channels.items():
            rospy.logdebug(channel.topic)
            # pubs[channel.topic] = None

        schema_map = {
            "foxglove.CompressedImage": "sensor_msgs/CompressedImage",  # "foxglove_msgs/CompressedImage",
        }

        t0 = None
        t_old = None
        # iter_messages -> don't decode
        for ind, (schema, channel, message) in enumerate(reader.iter_messages()):
            # message.data has the encoded message

            if rospy.is_shutdown():
                break

            t_cur = message.log_time / 1e9
            clock_msg = Clock()
            clock_msg.clock = rospy.Time.from_sec(t_cur)
            if t0 is None:
                t0 = t_cur
                rospy.loginfo(f"{t0:0.3f}s")
                t_old = t0

            # TODO(lucasw) this probably won't work in many cases, need a lookup table
            if schema.encoding == "protobuf":
                if schema.name in schema_map:
                    dst_schema_name = schema_map[schema.name]
                    msg_type = get_message_class(dst_schema_name)

                    topic = channel.topic
                    if schema.name == "foxglove.CompressedImage":
                        # TODO(lucasw) having this convention forced on topic names is annoying
                        topic += "/compressed"

                    if channel.topic not in pubs:
                        text = f"protobuf '{schema.name}' -> ros1msg '{dst_schema_name}' on {topic}, {msg_type}"
                        rospy.logdebug(text)
                        rospy.logdebug(schema)
                        pubs[topic] = rospy.Publisher(topic, msg_type, queue_size=3)

                    # TODO(lucasw) deserialize protobuf message
                    msg_decoder = protobuf_decoder.decoder_for("protobuf", schema)
                    if msg_decoder is None:
                        rospy.logerr_throttle(1.0, f"no decoder {schema.name}")
                        continue
                    proto_msg = msg_decoder(message.data)
                    # rospy.loginfo(proto_msg)

                    ros1_msg = msg_type()
                    # TODO(lucasw) is there a convenience method in the mcap library to do this?
                    # TODO(lucasw) make a conversion function
                    if schema.name == "foxglove.CompressedImage":
                        ros1_msg.header.stamp.secs = proto_msg.timestamp.seconds
                        ros1_msg.header.stamp.nsecs = proto_msg.timestamp.nanos
                        ros1_msg.header.frame_id = proto_msg.frame_id
                        # proto_msg.format is "image/jpeg"
                        # ros1_msg.format = proto_msg.format
                        ros1_msg.format = "jpeg"
                        ros1_msg.data = proto_msg.data
                    else:
                        rospy.logerr(f"can't translate message of type {schema.name}")
                    pubs[topic].publish(ros1_msg)
                else:
                    rospy.logwarn("can't translate message")

            elif schema.encoding == "ros1msg":
                if channel.topic not in pubs:
                    if schema.name not in msg_classes:
                        if True:
                            msg_type = get_message_class(schema.name)
                        else:
                            # TODO(lucasw) this only works with the md5sum
                            # TODO(lucasw) this is nav_msgs/Odometry,
                            # how to compute from message_definition?
                            # md5sum = "cd5e73d190d741a2f92e81eda573aca7"
                            md5sum = "*"
                            message_definition = schema.data

                            class MyShapeShifterMsg(rospy.AnyMsg):
                                _md5sum = md5sum
                                _type = schema.name
                                _full_text = message_definition

                                def __init__(self):
                                    self._buff = None

                            msg_type = MyShapeShifterMsg
                        msg_classes[schema.name] = msg_type
                    msg_type = msg_classes[schema.name]
                    text = f"ros1msg '{schema.name}' on {channel.topic}, {msg_type}"
                    rospy.logdebug(text)

                    # TODO(lucasw) is there any metadata to show if this should be static or not?
                    # also this will work incorrectly it will publish all the messages in a row
                    # and then only latch the last one, it needs to aggregate with more special case
                    # code here
                    latch = False
                    queue_size = 20
                    if channel.topic == "/tf_static":
                        queue_size == 100
                    pubs[channel.topic] = rospy.Publisher(channel.topic, msg_type,
                                                          latch=latch, queue_size=queue_size)

                if channel.topic == "/tf_static":
                    # TODO(lucasw) decode the message and make a single aggregated tf_static message
                    tf_static_messages.append(message)
                    continue

                # second part of avoided decoding, is avoiding encoding via AnyMsg
                msg = AnyMsg()
                # print(f"{[d for d in message.data]}")
                msg._buff = message.data
                pubs[channel.topic].publish(msg)

                if last_clock_msg is None or (clock_msg.clock - last_clock_msg.clock) > rospy.Duration(0.005):
                    clock_pub.publish(clock_msg)
                    last_clock_msg = clock_msg

                if (clock_msg.clock - last_tf_static_stamp) > rospy.Duration(0.5):
                    for message in tf_static_messages:
                        tf_static_msg = AnyMsg()
                        tf_static_msg._buff = message.data
                        pubs["/tf_static"].publish(tf_static_msg)
                    last_tf_static_stamp = clock_msg.clock

            dt = t_cur - t_old
            time.sleep(dt / rate)
            t_old = t_cur

        # TODO(lucasw) keep advancing /clock for a few seconds before looping
        for i in range(100):
            last_clock_msg.clock += rospy.Duration(0.05)
            clock_pub.publish(last_clock_msg)
            time.sleep(0.01)


if __name__ == "__main__":
    main()

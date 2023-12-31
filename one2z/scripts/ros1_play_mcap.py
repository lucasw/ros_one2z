#!/usr/bin/env python
"""
load a ros1 mcap file and play messages out of it

pip install mcap mcap-protobuf-support

rosrun one2z ros1_play_mcap.py demo_2023-11-18_06-06-59_image_only.mcap

"""

import sys

import rospy
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory as ProtobufDecoderFactory
from mcap_ros1.decoder import DecoderFactory
from roslib.message import get_message_class
from rospy.msg import AnyMsg


def main():
    rospy.init_node("play_mcap")
    decoder = DecoderFactory()
    protobuf_decoder = ProtobufDecoderFactory()
    with open(sys.argv[1], "rb") as f:
        reader = make_reader(f, decoder_factories=[decoder])
        schemas = reader.get_summary().schemas
        for schema_id, schema in schemas.items():
            rospy.loginfo(f"encoding: {schema.encoding}")

        summary = reader.get_summary()
        pubs = {}
        for key, channel in summary.channels.items():
            rospy.loginfo(channel.topic)
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
                        rospy.loginfo(text)
                        rospy.loginfo(schema)
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
                msg_type = get_message_class(schema.name)
                if channel.topic not in pubs:
                    text = f"ros1msg '{schema.name}' on {channel.topic}, {msg_type}"
                    rospy.loginfo(text)
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

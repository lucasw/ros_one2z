#!/usr/bin/env python
#
# Copyright (c) 2022 ZettaScale Technology
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors:
#   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
#

import argparse
import json
import sys
from io import BytesIO

# import numpy as np
import rospy
import zenoh
from bouncing_ball import image_to_contour
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from zenoh import Reliability, Sample


class ImageSub:
    def __init__(self, session: zenoh.session.Session, key: str):
        self.cv_bridge = CvBridge()

        # keys in zenoh maps to topics in ros
        self.contour_pub = session.declare_publisher("contour")
        rospy.loginfo(f"Declaring Subscriber on '{key}'...")
        self.sub = session.declare_subscriber(key, self.listener, reliability=Reliability.RELIABLE())

    def listener(self, sample: Sample):
        # text = f"kind: {sample.kind}, key: {sample.key_expr}"
        # rospy.loginfo_throttle(1.0, text)

        image_msg = Image()
        image_msg.deserialize(sample.payload)
        t0 = rospy.Time.now()
        age = t0 - image_msg.header.stamp

        image_np = self.cv_bridge.imgmsg_to_cv2(image_msg)
        polygon = image_to_contour(image_msg.header, image_np)

        buff = BytesIO()
        polygon.serialize(buff)
        self.contour_pub.put(buff.getvalue())

        t1 = rospy.Time.now()
        age = t0 - image_msg.header.stamp
        rospy.loginfo_throttle(1.0, f"msg {age.to_sec():0.3f}s old, processing {(t1 - t0).to_sec():0.3f}s")

    def __del__(self):
        pass
        # TODO(lucasw) this isn't needed, the sub will be undeclared when self goes away
        # rospy.loginfo("undeclaring subscriber")
        # self.sub.undeclare()


def main():
    parser = argparse.ArgumentParser(
        prog='z_pub',
        description='zenoh pub example')
    parser.add_argument('--mode', '-m', dest='mode',
                        choices=['peer', 'client'],
                        type=str,
                        help='The zenoh session mode.')
    parser.add_argument('--connect', '-e', dest='connect',
                        metavar='ENDPOINT',
                        action='append',
                        type=str,
                        help='Endpoints to connect to.')
    parser.add_argument('--listen', '-l', dest='listen',
                        metavar='ENDPOINT',
                        action='append',
                        type=str,
                        help='Endpoints to listen on.')
    parser.add_argument('--key', '-k', dest='key',
                        default='image',
                        type=str,
                        help='The key expression to publish onto.')
    parser.add_argument('--config', '-c', dest='config',
                        metavar='FILE',
                        type=str,
                        help='A configuration file.')

    args = parser.parse_args()
    conf = zenoh.Config.from_file(args.config) if args.config is not None else zenoh.Config()
    if args.mode is not None:
        conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
    if args.connect is not None:
        conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
    if args.listen is not None:
        conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))

    # TODO(lucasw) can I print log message to this?
    # initiate logging
    zenoh.init_logger()

    rospy.init_node("zenoh_image_sub")

    rospy.loginfo("Opening session...")
    session = zenoh.open(conf)
    z_info = session.info()
    rospy.loginfo(f"peers: {z_info.peers_zid()}, routers: {z_info.routers_zid()} {z_info.session} {z_info.zid()}")

    node = ImageSub(session, key=args.key)
    rospy.spin()
    rospy.loginfo("done with image sub")

    # TODO(lucasw) ctrl-c isn't stopping this, the listener persists
    node.sub.undeclare()
    # del node
    session.close()
    rospy.loginfo("done")
    sys.exit(0)


if __name__ == "__main__":
    main()

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

# import numpy as np
import rospy
import zenoh
from bouncing_ball import BouncingBall
# from zenoh import config


class GenerateImage:
    def __init__(self, session: zenoh.session.Session, key: str):
        # keys in zenoh maps to topics in ros
        rospy.loginfo(f"Declaring Publisher on '{key}'...")
        self.pub = session.declare_publisher(key)

        self.bouncing_ball = BouncingBall()

        period = 0.0333
        self.timer = rospy.Timer(rospy.Duration(period), self.update)

    def update(self, event: rospy.timer.TimerEvent):
        image_np = self.bouncing_ball.update()
        image_bytes = image_np.tobytes()
        self.pub.put(image_bytes)
        # rospy.loginfo_throttle(1.0, f"{len(image_bytes)} -> {len(np.frombuffer(image_bytes))}")

    def __del__(self):
        rospy.loginfo("undeclaring publisher")
        self.pub.undeclare()


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

    rospy.init_node("generate_image")

    rospy.loginfo("Opening session...")
    session = zenoh.open(conf)
    z_info = session.info()
    rospy.loginfo(f"peers: {z_info.peers_zid()}, routers: {z_info.routers_zid()} {z_info.session} {z_info.zid()}")

    node = GenerateImage(session, key=args.key)
    rospy.spin()

    del node
    session.close()


if __name__ == "__main__":
    main()

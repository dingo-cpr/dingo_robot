#!/usr/bin/env python3
# Software License Agreement (proprietary)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2020, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, is not permitted without the
# express permission of Clearpath Robotics.

import rospy
import argparse
import datetime

from puma_motor_msgs.msg import MultiFeedback
from puma_motor_msgs.msg import Feedback
from sensor_msgs.msg import JointState


class MockPuma:
    lastMillis = 0
    newCmd = False

    def millis(self):
        time_passed = datetime.datetime.now() - self.start_time
        return time_passed.total_seconds() * 1000

    def callback(self, msg):
        self.cmdMsg = msg
        self.newCmd = True

    def publishMultiFeedback(self):
        while True:
            # Motor spinning
            for i in range(0, self.numMotors):
                self.travel[i] += self.velocity[i] * (self.millis() - self.lastMillis) / 1000
                print("Distance traveled on motor", i, ":", self.travel[i], sep=" ")
            self.lastMillis = self.millis()

            if self.newCmd:
                self.newCmd = False
                # Set new velocity
                for i in range(0, self.numMotors):
                    if self.velocity[i] is not self.cmdMsg.velocity[i]:
                        self.velocity[i] = self.cmdMsg.velocity[i]


            msg = MultiFeedback()
            msg.drivers_feedback = [Feedback()] * self.numMotors

            for i in range (0, self.numMotors):
                msg.drivers_feedback[i].device_number = i
                msg.drivers_feedback[i].device_name = "Puma"
                msg.drivers_feedback[i].travel = self.travel[i]
                msg.drivers_feedback[i].speed = self.velocity[i]

            if not rospy.is_shutdown():
                self.pub.publish(msg)
                self.rate.sleep()

    def __init__(self, config, rate):
        self.start_time = datetime.datetime.now()

        self.config = config

        if config is 'd':
            self.numMotors = 2
        elif config is 'o':
            self.numMotors = 4
        else:
            print("Invalid Dingo config")
            return

        self.velocity = [0] * self.numMotors
        self.travel = [0] * self.numMotors
        # Rospy node
        rospy.init_node('mock_puma')
        self.rate = rospy.Rate(rate)
        # Subscriber and publisher
        rospy.Subscriber("/cmd", JointState, self.callback)
        self.pub = rospy.Publisher('/feedback', MultiFeedback, queue_size=1)

        self.publishMultiFeedback()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config', help='Dingo config')
    parser.add_argument('r', default=10, nargs='?', help='Publish rate')
    args = parser.parse_args()

    try:
        puma = MockPuma(args.config, int(args.r))
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        raise

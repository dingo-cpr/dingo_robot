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
import random
import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class MockOdomMocap:
    odomEnabled = False
    mocapEnabled = False
    odomMessage = Odometry()
    mocapMessage = PoseStamped()

    def publish(self):
        while True:
            if (rospy.Time.now() - self.startTime).to_sec() > self.odomStart:
                self.odomEnabled = True
            if (rospy.Time.now() - self.startTime).to_sec() > self.mocapStart:
                self.mocapEnabled = True

            if self.odomEnabled:
                self.odomPos += 0.1
                self.odomMessage.header.stamp = rospy.Time.now() - self.startTime
                print(self.odomMessage.header.stamp)
                self.odomMessage.pose.pose.position.x = self.odomPos
                self.odomMessage.pose.pose.position.y = self.odomPos
                self.odomMessage.pose.pose.position.z = self.odomPos
                self.odomMessage.pose.pose.orientation.w = self.odomPos * -1
                self.odomMessage.pose.pose.orientation.x = self.odomPos * -2
                self.odomMessage.pose.pose.orientation.y = self.odomPos * -3
                self.odomMessage.pose.pose.orientation.z = self.odomPos * -4
                self.odomPub.publish(self.odomMessage)

            if self.mocapEnabled:
                self.mocapPos += 0.11
                self.mocapMessage.header.stamp = rospy.Time.now() - self.startTime
                self.mocapMessage.pose.position.x = self.mocapPos
                self.mocapMessage.pose.position.y = self.mocapPos
                self.mocapMessage.pose.position.z = self.mocapPos
                self.mocapMessage.pose.orientation.w = self.mocapPos * 1
                self.mocapMessage.pose.orientation.x = self.mocapPos * 2
                self.mocapMessage.pose.orientation.y = self.mocapPos * 3
                self.mocapMessage.pose.orientation.z = self.mocapPos * 4
                self.mocapPub.publish(self.mocapMessage)

            self.rate.sleep()


    def __init__(self):
        rospy.init_node('mock_odom_mocap')
        self.rate = rospy.Rate(10)

        self.odomPub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.mocapPub = rospy.Publisher('/mocap', PoseStamped, queue_size=1)

        self.odomStart = random.randint(0, 10)
        self.odomPos = self.odomStart

        self.mocapStart = random.randint(0, 10)
        self.mocapPos = self.mocapStart

        self.startTime = rospy.Time.now()

        print("Odom starts at " + str(self.odomStart))
        print("Mocap starts at " + str(self.mocapStart))

        self.publish()


if __name__ == '__main__':
    try:
        mock = MockOdomMocap()
    except KeyboardInterrupt:
        sys.exit()

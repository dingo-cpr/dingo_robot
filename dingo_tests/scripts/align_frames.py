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
import time
import sys

import matplotlib.pyplot as plt
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class Aligner:
    odomMessages = []
    odomTimes = []
    mocapMessages = []
    mocapTimes = []
    commonTimes = []
    firstOdom = False
    firstMocap = False
    plotnum = 1

# Convert orientation to roll, pitch, yaw
    def quaternion_to_euler(self, orientation):
        w = orientation.w
        x = orientation.x
        y = orientation.y
        z = orientation.z

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)

        t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return X, Y, Z

# Odometry callback
    def odomCallback(self, msg):
        if not self.firstOdom:
            print("Odometry received")
            self.firstOdom = True

        self.odomMessages.append(msg)
        self.odomTimes.append(msg.header.stamp)

# Motion capture callback
    def mocapCallback(self, msg):
        if not self.firstMocap:
            print("Mocap received")
            self.firstMocap = True

        self.mocapMessages.append(msg)
        self.mocapTimes.append(msg.header.stamp)
# Add subplot
    def subplot(self, lines, ylabel='', xlabel='', title='', legend=True):
        TIME = 0
        DATA = 1
        LABEL = 2
        #Plotting data and save to image
        sp = plt.subplot(2, 3, self.plotnum)

        for i in range(0, len(lines)):
                plt.plot(lines[i][TIME], lines[i][DATA], label=lines[i][LABEL])

        ymin,ymax = plt.ylim()
        plt.ylim(ymin - 0.5, ymax + 0.5)
        plt.grid(True, which='both')
        plt.ylabel(ylabel)
        plt.xlabel(xlabel)
        plt.title(title)
        if legend:
            sp.legend(loc='center left', bbox_to_anchor=(1.05, 0.5), fontsize='small')
        box = sp.get_position()
        sp.set_position([box.x0, box.y0, box.width * 0.8, box.height * 0.9])
        plt.subplots_adjust(right=0.85, hspace=0.6, left = 0.05, wspace = 0.55, bottom=0.05, top=0.95)

        self.plotnum += 1
# Align odom/mocap frames and plot
    def plot(self):
        i = 0
        # Find which topic runs first and which is second in terms of time stamps
        if self.odomMessages[0].header.stamp.to_sec() < self.mocapMessages[0].header.stamp.to_sec():
            while self.odomMessages[i].header.stamp < self.mocapMessages[0].header.stamp:
                i += 1
            self.odomMessages = self.odomMessages[i:]
        else:
            while self.mocapMessages[i].header.stamp < self.odomMessages[0].header.stamp:
                i += 1
            self.mocapMessages = self.mocapMessages[i:]

        odomTimes = []
        odomX = []
        odomY = []
        odomZ = []
        odomRoll = []
        odomPitch = []
        odomYaw = []

        mocapTimes = []
        mocapX = []
        mocapY = []
        mocapZ = []
        mocapRoll = []
        mocapPitch = []
        mocapYaw = []

        # Separate messages into plottable lists
        for message in self.odomMessages:
            odomTimes.append(message.header.stamp.to_sec())

            odomX.append(message.pose.pose.position.x)
            odomY.append(message.pose.pose.position.y)
            odomZ.append(message.pose.pose.position.z)

            r, p, y = self.quaternion_to_euler(message.pose.pose.orientation)
            odomRoll.append(r)
            odomPitch.append(p)
            odomYaw.append(y)

        for message in self.mocapMessages:
            mocapTimes.append(message.header.stamp.to_sec())

            mocapX.append(message.pose.position.x)
            mocapY.append(message.pose.position.y)
            mocapZ.append(message.pose.position.z)

            r, p, y = self.quaternion_to_euler(message.pose.orientation)
            mocapRoll.append(r)
            mocapPitch.append(p)
            mocapYaw.append(y)

        self.subplot([[odomTimes, odomX, 'Odom'],
                      [mocapTimes, mocapX, 'Mocap']],
                      ylabel='Position (m?)',
                      xlabel='Time (s)',
                      title='Position (X)')

        self.subplot([[odomTimes, odomY, 'Odom'],
                      [mocapTimes, mocapY, 'Mocap']],
                      ylabel='Position (m?)',
                      xlabel='Time (s)',
                      title='Position (Y)')

        self.subplot([[odomTimes, odomZ, 'Odom'],
                      [mocapTimes, mocapZ, 'Mocap']],
                      ylabel='Position (m?)',
                      xlabel='Time (s)',
                      title='Position (Z)')

        self.subplot([[odomTimes, odomRoll, 'Odom'],
                      [mocapTimes, mocapRoll, 'Mocap']],
                      ylabel='Angle (deg)',
                      xlabel='Time (s)',
                      title='Roll (X)')

        self.subplot([[odomTimes, odomPitch, 'Odom'],
                      [mocapTimes, mocapPitch, 'Mocap']],
                      ylabel='Angle (deg)',
                      xlabel='Time (s)',
                      title='Pitch (Y)')

        self.subplot([[odomTimes, odomYaw, 'Odom'],
                      [mocapTimes, mocapYaw, 'Mocap']],
                      ylabel='Angle (deg)',
                      xlabel='Time (s)',
                      title='Yaw (Z)')

        plt.show()

    def __init__(self, duration):
        rospy.init_node('odom_mocap_aligner')
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.Subscriber("/mocap", PoseStamped, self.mocapCallback)

        # Wait for first message
        print("Waiting for first messages")
        while not self.firstOdom or not self.firstMocap:
            pass

        startTime = rospy.Time.now()

        print("Capturing /odom and /mocap for " + str(duration) + " seconds")
        while (rospy.Time.now() - startTime).to_sec() < duration:
            pass

        self.plot()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('duration', default=60, nargs='?', help='Capture duration')
    args = parser.parse_args()

    try:
        aligner = Aligner(int(args.duration))
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

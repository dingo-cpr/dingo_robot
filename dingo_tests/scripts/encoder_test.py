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
import math
import time
import datetime
import sys

import matplotlib.pyplot as plt

from sensor_msgs.msg import JointState
from puma_motor_msgs.msg import MultiFeedback

def startTimer():
    global start_time
    start_time = datetime.datetime.now()

def millis():
    time_passed = datetime.datetime.now() - start_time
    return time_passed.total_seconds() * 1000

class Motor:
    state = 0
    count = 0
    stopUntilMs = 0
    delayMs = 5000
    initialRotation = 0
    maxMotorRotation = math.pi # 180 degrees
    constMotorSpeed = 1 # 1 rad/s

    def update(self, feedback):
        STOPPED = 0
        MOVING = 1

        if self.state == STOPPED:
            if millis() >= self.stopUntilMs:
                self.state = MOVING
                self.velocity = self.constMotorSpeed
                self.initialRotation = feedback.travel
        else:
            # If motor has reached desired rotation
            if feedback.travel - self.initialRotation >= self.maxMotorRotation:
                self.state = STOPPED
                self.velocity = 0
                self.stopUntilMs = millis() + self.delayMs
                self.count += 1
                print("Stop", self.count)

    def __init__(self, count):
        self.velocity = 0
        self.travel = 0
        self.maxCount = count

class EncoderTest:
    firstFeedback = True
    jointStateMsg = JointState()
    newFeedback = False
    finishedTestCount = 0
    plotnum = 1

    def callback(self, msg):
        self.feedbackMsg = msg
        self.newFeedback = True

    def allTestsDone(self):
        for done in self.testsDone:
            if not done:
                return False
        return True

    def publishJointState(self):
        while True:
            if not self.allTestsDone():
                if self.newFeedback:
                    self.newFeedback = False

                    # Start time on first feedback
                    if self.firstFeedback:
                        startTimer()
                        self.firstFeedback = False
                        print("Receiving Feedback...")

                    for i, feedback in enumerate(self.feedbackMsg.drivers_feedback, start=0):
                        # If the motor has not finished all of its test loops
                        if not self.testsDone[i]:
                                self.travelTime[i].append(millis() / 1000)
                                self.travel[i].append(feedback.travel)
                                self.velocity[i].append(feedback.speed)

                                self.motor[i].update(feedback)

                                if self.motor[i].count == self.testCount:
                                    self.testsDone[i] = True
                                    print("Motor", i, "done")

                                self.jointStateMsg.name[i] = feedback.device_name
                                self.jointStateMsg.velocity[i] = self.motor[i].velocity

                    # Publish jointState message
                    if not rospy.is_shutdown():
                        self.pub.publish(self.jointStateMsg)
                        self.rate.sleep()

            else:
                self.plotResults()
                return

    def plot(self, lines, ylabel='', xlabel='', title='', legend=False):
        TIME = 0
        DATA = 1
        LABEL = 2

        sp = plt.subplot(self.numMotors, 2, self.plotnum)

        for i in range(0, len(lines)):
                plt.plot(lines[i][TIME], lines[i][DATA], label=lines[i][LABEL])

        ymin,ymax = plt.ylim()
        xmin,xmax = plt.xlim()
        plt.ylim(ymin - 0.5, ymax + 0.5)
        plt.xlim(0, xmax)
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

    def plotResults(self):
        for i in range(0, self.numMotors):
            self.plot([[self.travelTime[i], self.travel[i], 'Feedback']],
                                    ylabel='Travel (rad)',
                                    xlabel='Time (s)',
                                    title='Motor ' + str(i),
                                    legend=False)

            self.plot([[self.travelTime[i], self.velocity[i], 'Feedback']],
                                    ylabel='Velocity (rad/s)',
                                    xlabel='Time (s)',
                                    title='Motor ' + str(i),
                                    legend=False)
        plt.show()

    def __init__(self, config, count, rate):
        # Must do at least 1 loop
        if count == 0:
            count = 1

        self.config = config
        self.testCount = count

        if config is 'd':
            self.numMotors = 2
        elif config is 'o':
            self.numMotors = 4
        else:
            print("Invalid Dingo config")
            return

        self.motor = [Motor(count)] * self.numMotors

        self.travelTime = [[0] for i in range(self.numMotors)]
        self.travel = [[0] for i in range(self.numMotors)]
        self.expectedTravel = [[0] for i in range(self.numMotors)]
        self.velocity = [[0] for i in range(self.numMotors)]

        self.jointStateMsg.name = [''] * self.numMotors
        self.jointStateMsg.position = [0] * self.numMotors
        self.jointStateMsg.velocity = [0] * self.numMotors
        self.jointStateMsg.effort = [0] * self.numMotors

        self.testsDone = [False] * self.numMotors

        rospy.init_node('puma_feedback_listener')
        self.rate = rospy.Rate(rate)

        rospy.Subscriber("/feedback", MultiFeedback, self.callback)
        self.pub = rospy.Publisher('/cmd', JointState, queue_size=1)

        print("Beginning tests in configuration", config, "for", count, "loops...")

        self.publishJointState()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config', help='Dingo config')
    parser.add_argument('count', default=0, nargs='?', help='Number of loops to run')
    parser.add_argument('r', default=10, nargs='?', help='Publish rate')
    args = parser.parse_args()

    try:
        test = EncoderTest(args.config, int(args.count), int(args.r))
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

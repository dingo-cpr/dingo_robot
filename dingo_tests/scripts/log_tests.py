#!/usr/bin/env python3
# Software License Agreement (proprietary)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2020, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, is not permitted without the
# express permission of Clearpath Robotics.

import socket
import sys
import time
from test_tools import Tester, printTestResults, parseLog

def logTest():
    results = []

    print("Checking that logs are being received")

    results.append(parseLog("Log check", "[INFO]", 60))

    printTestResults("Log Check Test", results)

def imuDetectTest():
    results = []

    print("Reset the MCU to detect the IMU")

    results.append(parseLog("Imu Detect", "ICM20948 detected", 60))

    printTestResults("Imu Detect Test", results)

def batteryDetectTest():
    results = []
    logs = []

    print("Reset the MCU to detect the batteries")

    results.append(parseLog("Battery Detect", "Battery Chemistry:", 60))

    printTestResults("Battery Detect Test", results)

if __name__ == '__main__':
    print("Ensure that the Dingo is on and connected via Ethernet")
    print("Set the PC's IP to 192.168.131.1")

    tester = Tester()

    tester.addTest("Log check", logTest)
    tester.addTest("Imu Detect", imuDetectTest)
    tester.addTest("Battery Detect", batteryDetectTest)

    tester.run()

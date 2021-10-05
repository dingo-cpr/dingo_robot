#!/usr/bin/env python
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
import datetime

class TestResult:
    # Test results: PASSED, FAILED, WARNING, N/A
    def __init__(self, name, result):
        self.name = name
        self.result = result

class Test:
    def __init__(self, name, func):
        self.name = name
        self.func = func

class Tester:
    tests = []

    def addTest(self, name, func):
        self.tests.append(Test(name, func))

    def runTest(self, index):
        test = self.tests[index]

        # Format
        dash = '-' * 30
        print(dash)
        print('{:^30s}'.format('Running Test: ' + test.name))
        print(dash)

        status = False

        # Enables automatic rerunning of test if test execution failed
        while not status:
            status = test.func()

    def showTestOptions(self):
        # Format
        dash = '-' * 30
        print(dash)
        print('{:^30s}'.format('Test Options'))
        print(dash)

        for i, test in enumerate(self.tests):
            print('{:d}. {}'.format(i + 1, test.name))
        print('{:d}. All Tests'.format(len(self.tests) + 1))
        print('{:d}. Exit'.format(len(self.tests) + 2))
        option = int(raw_input('Select an option: '))

        # Exit option
        if option == len(self.tests) + 2:
            sys.exit()
        # All tests option
        elif option == len(self.tests) + 1:
            for index in range(len(self.tests)):
                self.runTest(index)
        elif option > 0 and option <= len(self.tests):
            self.runTest(option - 1)
        else:
            print('Invalid input')

        time.sleep(1)
        self.showTestOptions()

    def run(self):
        self.showTestOptions()

# Displays user prompt message, then requests user input of (y/n).
# Returns TestResult with 'PASSED' or 'FAILED' results corresponding to user input.
def userInputTestResults(prompt, test_case):
    print(prompt)
    response = raw_input('y/n: ')

    if response == 'y' or response == 'Y':
        return TestResult(test_case, 'PASSED')
    elif response == 'n' or response == 'N':
        return TestResult(test_case, 'FAILED')
    else:
        print('Invalid input')
        return userInputTestResults(prompt, test_case)

# Returns TestResult with 'PASSED' or 'FAILED' results based on value of boolean input
def boolTestResults(bool_input, test_case):
    result = ''

    if bool_input:
        result = 'PASSED'
    else:
        result = 'FAILED'

    return TestResult(test_case, result)

# Returns TestResults with 'N/A' for test runs that failed to execute
def notApplicableTestResult():
    result = 'N/A'
    message = 'Test failed to execute'

    return TestResult(message, result)

# Determines the ADC test case's result based on actual value, target value, and preset tolerances.
def getAdcTestResults(value, target_value, test_case):
    lower_tolerance = 1
    upper_tolerance = 2

    if value <= target_value + lower_tolerance and value >= target_value - lower_tolerance:
        return TestResult(test_case, 'PASSED')
    elif (value <= target_value + upper_tolerance and value > target_value + lower_tolerance) or (value >= target_value - upper_tolerance and value < target_value - lower_tolerance):
        return TestResult(test_case, 'WARNING')
    elif value > target_value + upper_tolerance or value < target_value - upper_tolerance:
        return TestResult(test_case, 'FAILED')
    else:
        print('Invalid input')
        return getAdcTestResults(value, target_value, test_case)

def printTestResults(name, results):
    # Format
    dash = '-' * 30
    print('\033[0;37;40m')
    print(dash)
    print('{:^30s}'.format(name + ' Results'))
    print(dash)

    for test in results:
        name_string = '\033[0;37;40m' + test.name
        if test.result == 'PASSED':
            result_string = '\033[1;32;40m PASSED'
        elif test.result == 'FAILED':
            result_string = '\033[1;31;40m FAILED'
        elif test.result == 'WARNING': 
            result_string = '\033[1;33;40m WARNING'
        elif test.result == 'N/A':
            result_string = '\033[1;34;40m N/A'
        else:
            print('Invalid test result for {}'.format(name_string))        
            continue 
        print('{:<30s}{:>20s}\033[0;39;49m'.format(name_string, result_string))

# @param[in]: Message to parse for, timeout in seconds
def parseLog(test_case, log, timeout):
    HOST = '192.168.131.1'
    PORT = 11413
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind the socket to the port
    server_address = (HOST, PORT)
    sock.bind(server_address)

    start_time = datetime.datetime.now()

    print('Parsing logger...')

    while True:
        try:
            data, address = sock.recvfrom(4096)
            if log in str(data):
                print('Matching log found: ' + str(data))
                return TestResult(test_case, 'PASSED')
            if (datetime.datetime.now() - start_time).total_seconds() >= timeout:
                return TestResult(test_case, 'FAILED')
        except KeyboardInterrupt:
            sys.exit()
#!/usr/bin/env python3

import os
import sys
import datetime, time
import rosbag
import rospy
import matplotlib.pyplot as plt
import argparse

class MenuOption():
    def __init__(self, name, func):
        self.name = name
        self.func = func

class Menu():
    bagName = ""
    entries = []

    def addMenuOption(self, name, func):
        self.entries.append(MenuOption(name, func))

    def requestTopic(self, option):
        print("Enter the ", option, " topic")
        return input()

    def showMenu(self):
        print("Menu")
        for i, entry in enumerate(self.entries):
            print('{:d}.'.format(i+1), entry.name)
        print('{:d}. Exit'.format(len(self.entries) + 1))
        option = int(input())
        # Exit option
        if option == len(self.entries) + 1:
            sys.exit()
        elif option > 0 and option <= len(self.entries):
            self.entries[option - 1].func()
        else:
            print("Invalid input")

        # Reset plotnum
        self.plotter.plotnum = 1

        time.sleep(1)
        self.showMenu()

    def run(self):
        self.showMenu()

    def read_odom(self, topic):
        self.times_odom = []
        self.linear_odom_x = []
        self.linear_odom_y = []
        self.angular_odom = []

        for topic, msg, t in self.bag.read_messages(topics=[topic]):
            #build lists from msgs in bag
            self.times_odom.append(msg.header.stamp.to_sec() - self.start_time)
            self.linear_odom_x.append(msg.twist.twist.linear.x)
            self.linear_odom_y.append(msg.twist.twist.linear.y)
            self.angular_odom.append(msg.twist.twist.angular.z)

    def read_voltage(self, topic):
        self.times_voltage = []
        self.voltage = []
        for topic, msg, t in self.bag.read_messages(topics=[topic]):
            #build lists from msgs in bag
            self.times_voltage.append(msg.header.stamp.to_sec() - self.start_time)
            self.voltage.append(msg.voltage)

    def read_current(self, topic):
        self.times_current = []
        self.current = []
        for topic, msg, t in self.bag.read_messages(topics=[topic]):
            #build lists from msgs in bag
            self.times_current.append(msg.header.stamp.to_sec() - self.start_time)
            self.current.append(msg.current)

    def read_puma(self, topic, numMotors):
        self.numMotors = numMotors
        self.times_puma = [[0] for i in range(numMotors)]
        self.current = [[0] for i in range(numMotors)]
        self.speed = [[0] for i in range(numMotors)]
        for topic, msg, t in self.bag.read_messages(topics=[topic]):
            #build lists from msgs in bag
            for i in range(numMotors):
                self.times_puma[i].append(msg.header.stamp.to_sec() - self.start_time)
                self.current[i].append(msg.drivers_feedback[i].current)
                self.speed[i].append(msg.drivers_feedback[i].speed)

    def plotOdom(self):
        # Get topic string
        topic = self.requestTopic("Odom")
        # Open bag
        self.bag = rosbag.Bag(self.bagName)
        self.start_time = self.bag.get_start_time()
        # Read data
        self.read_odom(topic)
        # Close bag
        self.bag.close()

        self.plotter.plot(row=2, col=1,
                          lines=[[self.times_odom, self.linear_odom_x, 'X'],
                                 [self.times_odom, self.linear_odom_y, 'Y']],
                          ylabel='Velocity (m/s)',
                          xlabel='Time (s)',
                          title='Linear Velocity vs Time',
                          subplot=True)

        self.plotter.plot(row=2, col=1,
                          lines=[[self.times_odom, self.angular_odom, 'Z']],
                          ylabel='Velocity (m/s)',
                          xlabel='Time (s)',
                          title='Angular Velocity vs Time',
                          subplot=True)
        plt.show()

    def plotPuma(self):
        # Get topic string
        topic = self.requestTopic("Puma")
        # Open bag
        self.bag = rosbag.Bag(self.bagName)
        self.start_time = self.bag.get_start_time()
        # Read data
        self.read_puma(topic, int(self.requestTopic("Number of motors")))
        # Close bag
        self.bag.close()

        for i in range(self.numMotors):
            self.plotter.plot(row=1, col=4,
                              lines=[[self.times_puma[i], self.speed[i], 'Speed'],
                                     [self.times_puma[i], self.current[i], 'Current']],
                              ylabel='Speed (rad/s) | Current (A)',
                              xlabel='Time (s)',
                              title='Motor {:d}'.format(i),
                              subplot=True,
                              legend=True)
        plt.show()

    def plotVoltage(self):
        # Get topic string
        topic = self.requestTopic("Voltage")
        # Open bag
        self.plotter.bag = rosbag.Bag(self.bagName)
        self.plotter.start_time = self.plotter.bag.get_start_time()
        # Read data
        self.plotter.read_voltage(topic)
        # Close bag
        self.plotter.bag.close()

        self.plotter.plot(row=1, col=1,
                          lines=[[self.times_voltage, self.voltage, 'Voltage']],
                          ylabel='Voltage (V)',
                          xlabel='Time (s)',
                          title='Voltage vs Time',
                          subplot=True,
                          legend=False)
        plt.show()

    def plotCurrent(self):
        # Get topic string
        topic = self.requestTopic("Current")
        # Open bag
        self.plotter.bag = rosbag.Bag(self.bagName)
        self.plotter.start_time = self.plotter.bag.get_start_time()
        # Read data
        self.plotter.read_current(topic)
        # Close bag
        self.plotter.bag.close()

        self.plotter.plot(row=1, col=1,
                          lines=[[self.times_current, self.current, 'Current']],
                          ylabel='Current (A)',
                          xlabel='Time (s)',
                          title='Current vs Time',
                          subplot=True,
                          legend=False)
        plt.show()

    def __init__(self, bagName):
        self.bagName = bagName
        self.plotter = Plotter()

        self.addMenuOption("Odom", self.plotOdom)
        self.addMenuOption("Puma", self.plotPuma)
        self.addMenuOption("Voltage", self.plotVoltage)
        self.addMenuOption("Current", self.plotCurrent)

class Plotter():
    plotnum = 1

    def plot(self, row, col, lines, ylabel='', xlabel='', title='', legend=True, subplot=False):
        TIME = 0
        DATA = 1
        LABEL = 2
        #Plotting data and save to image
        if subplot:
            sp = plt.subplot(row, col, self.plotnum)
        else:
            fig = plt.figure(figsize=(16,9), dpi=96)
            sp = plt.subplot(1, 1, 1)

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
            sp.legend(loc='center left', bbox_to_anchor=(1.0, 0.5), fontsize='small')
        box = sp.get_position()
        sp.set_position([box.x0, box.y0, box.width * 0.8, box.height * 0.9])
        plt.subplots_adjust(right=0.9, hspace=0.6, left = 0.05, wspace = 0.55, bottom=0.05, top=0.8)

        if not subplot:
            fig.savefig(self.fileName[:-4]+ "-" + title + ".png",  bbox_inches='tight')

        self.plotnum += 1



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('file', help='bag file')
    args = parser.parse_args()

    menu = Menu(args.file)
    menu.run()

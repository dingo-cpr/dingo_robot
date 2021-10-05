# dingo_tests
**Test package for Dingo.**

## Overview
This ROS package contains a set of test scripts that can be used to verify the functionality of the Dingo from component and system levels. 

## Prerequisite
Please ensure that the Dingo bringup install script is run first. Amongst other important reasons, this step ensures that the `DINGO_OMNI` environment variable is set correctly, which is required to run some tests in this package.
```
rosrun dingo_bringup install
source /etc/ros/setup.bash
```

## Installation
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch melodic-devel http://gitlab.clearpathrobotics.com/research/dingo_tests.git # Use noetic-devel for ROS Noetic Dingo
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
catkin_make install
source install/setup.bash
```

## Usage

### CAN Bus Interface Check
This script is useful in verifying the Puma motors and encoders are sending and receiving data between the Dingo's PC, via the CAN bus interface. This script verifies that the `can0` interface is detected and activated, then proceeds to check the output of `candump` to verify that good CAN packets are being transmitted. Based on the Dingo configuration, either Dingo-D or Dingo-O, this script will know to check for good CAN packets from 2 or 4 encoders, respectively.
```
rosrun dingo_tests check_can_bus_interface
```

### ROS Tests
This script exposes a set of useful tests at the ROS-level, via ROS topics, to verify the functionality of core features. In addition, these tests together serve as a useful robot-level diagnostic tool, be identifying the root cause of problems, or at the very least, narrowing down on where the root cause(s) may be. 
```
rosrun dingo_tests ros_tests
```

#### Lighting Test
This test publishes lighting commands to `/cmd_lights` to verify that all 4 lights are working properly.

#### Cooling Test
This test requires that there is a fan connected to the Dingo, and publishes fan states to `/mcu/cmd_fans` to verify that the fan spins at the expected speeds.

#### E-Stop Test
This test requires that the E-Stop is engaged on the Dingo, and subscribes to `/mcu/status` to check that the MCU, ROS, and lights reflect the engaged E-Stop correctly.

#### ADC Test
This test subscribes to `/mcu/status` and checks the voltage and current values across the Dingo.

#### Rotate Test
This test subscribes to `/imu/data` and `/odometry/filtered`, and publishes to `/cmd_vel`. This test attempts to rotate the Dingo for 2 full revolutions based on the IMU's Gyroscope angular displacement data (derived from angular velocity measurements). In addition, it also tracks the angular displacement from the `robot_localization` EKF's odometry data for comparison.

#### Drive Test
This test subscribes to `/feedback` and `/dingo_velocity_controller/odom`, and publishes to `/cmd_vel`. This test attempts to drive the Dingo forward 1 metre based on the encoder-fused odometry data. In addition, it also tracks each individual encoder's linear displacement data for comparison.
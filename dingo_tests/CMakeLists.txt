cmake_minimum_required(VERSION 3.0.2)
project(dingo_tests)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dingo_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_python_setup()

catkin_package()

install(
  PROGRAMS scripts/ros_tests scripts/check_can_bus_interface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
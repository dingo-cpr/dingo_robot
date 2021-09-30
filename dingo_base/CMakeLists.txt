################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dingo_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(puma_motor_driver REQUIRED)
find_package(puma_motor_msgs REQUIRED)
find_package(socketcan_interface REQUIRED)

################################################################################
# Build
################################################################################
# include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add_executable(dingo_node
#   src/dingo_base.cpp
#   src/dingo_cooling.cpp
#   src/dingo_diagnostic_updater.cpp
#   src/dingo_hardware.cpp
#   src/dingo_lighting.cpp
#   src/dingo_logger.cpp)
# target_link_libraries(dingo_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(dingo_node ${catkin_EXPORTED_TARGETS})

add_library(
  ${PROJECT_NAME}
  SHARED
  src/dingo_hardware.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
	puma_motor_msgs
	socketcan_interface
	puma_motor_driver
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
	# ${puma_motor_driver_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(hardware_interface dingo_hardware.xml)
################################################################################
# Install
################################################################################
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
	DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# install(PROGRAMS scripts/compute_calibration
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(TARGETS dingo_node
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# if (CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(launch/base.launch)

#   find_package(roslint REQUIRED)
#   roslint_cpp()
#   roslint_add_test()
# endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()

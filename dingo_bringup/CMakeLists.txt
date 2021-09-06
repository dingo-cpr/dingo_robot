################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dingo_bringup)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
	DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install(FILES scripts/can-udp-bridge.conf scripts/can-udp-bridge.sh
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(PROGRAMS scripts/install scripts/set-dingo-o
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# if (CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(launch/accessories.launch)
# endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()

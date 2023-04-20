^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dingo_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2023-04-20)
------------------
* Adding Sick TIM551 lidar
* Updated realsense initial_reset
  Missing the prefix `realsense\_`
* Contributors: Hilary Luo, luis-camero

0.3.0 (2023-01-30)
------------------
* Added Wibotic charging status to dingo_lighting
  Renamed PUMA can interface to vcan0
  Added Wibotic launch to accessories.launch
* [dingo_bringup] Switched flir_camera_driver to spinnaker_camera_driver as dep.
* Microstrain uses microstrain.yaml instead of deprecated ROS launch arguments
* Contributors: Joey Yang, Roni Kreinin, Tony Baltovski

0.2.4 (2022-11-23)
------------------
* Fixed point_cloud parameter
* Contributors: Luis Camero

0.2.3 (2022-11-22)
------------------
* Added Blackfly Camera
* Replace ros_mscl with microstrain_inertial_driver
* Added secondary realsense
* Contributors: Joey Yang, Luis Camero

0.2.2 (2022-05-17)
------------------
* Fix realsense double namespace error
* Updated Bringup (`#14 <https://github.com/dingo-cpr/dingo_robot/issues/14>`_)
  * Added Velodyne HDL32E and Microstrain GX5
  * Added ros_mscl as run_depend in package.xml
  * Alphabetically sort package.xml
* Update realsense launch file based on changes from realsense2_camera
* Remove unused rs_model argument
* Contributors: Joey Yang, luis-camero

0.2.1 (2022-03-21)
------------------
* [dingo_bringup] Updated install script to explicitly use Python3.
* Switch usage of ifconfig (net-tools) to ip (iproute2) (`#12 <https://github.com/dingo-cpr/dingo_robot/issues/12>`_)
  * Switch usage of ifconfig (net-tools) to ip (iproute2)
  * Remove unnecessary "sudo"
* Contributors: Joey Yang, Tony Baltovski

0.2.0 (2022-01-19)
------------------

0.1.5 (2022-01-16)
------------------
* Add "_secondary" suffix to secondary laser node
  "RLException: roslaunch file contains multiple nodes named [/hokuyo]." is thrown if primary and secondary lasers are both enabled. This is because both envars launch an instance of the urg_node with the same name "hokuyo". Adding a "_secondary" suffix to the secondary laser node resolves this issue.
* Contributors: jyang-cpr

0.1.4 (2021-10-01)
------------------
* Setup scripts to automatically set DINGO_OMNI and DINGO_WIRELESS_INTERFACE envars (`#8 <https://github.com/dingo-cpr/dingo_robot/issues/8>`_)
  * Automatically triggering setup scripts in post_install()
  * Add wireless interface and dingo config scripts
  * Find setup scripts using find_in_workspaces, and trigger automatically
  * Fix script names
  Co-authored-by: Joey Yang <jyang@clearpathrobotics.com>
* Fix the name of teh VLP16 launch file that gets included
* Contributors: Chris Iverach-Brereton, jyang-cpr

0.1.3 (2021-03-08)
------------------
* Move the VLP16 to a new DINGO_LASER_3D family of vars, add DINGO_LASER_SECONDARY
* Add vlp16 to the commend block explaining allowed values
* Add support for the VLP16 as a standard laser sensor for Dingo
* Contributors: Chris Iverach-Brereton

0.1.2 (2020-11-26)
------------------
* [dingo_bringup] Made ros service start after can-udp-bridge service.
* Contributors: Tony Baltovski

0.1.1 (2020-11-23)
------------------
* Added missing test dependencies.
* Contributors: Tony Baltovski

0.1.0 (2020-11-13)
------------------
* Bump CMake version to avoid CMP0048 warning.
* [dingo_bringup] Increased CAN TX queue size.
* Merge pull request `#1 <https://github.com/dingo-cpr/dingo_robot/issues/1>`_ from dingo-cpr/rs-l515-update
  Remove the work-around for the RealSense L515 pointcloud data
* Remove the work-around for the RealSense L515 pointcloud data; the driver's been updated and this is no longer needed.
* [dingo_bringup] Added missing export.
* Merge branch 'no-realsense-to-laser' into 'melodic-devel'
  Remove the realsense's depthimage-to-laserscan node
  See merge request dingo/dingo_robot!4
* Remove the realsense's depthimage-to-laserscan node; the realsense is not well-suited to use as a lidar, so just remove this
* Move the depthimage to laserscan node inside the right group
* Merge branch 'melodic-devel' of gitlab.clearpathrobotics.com:dingo/dingo_robot into melodic-devel
* Add the missing enable_infra flag that's necessary to get the L515 to publish data correctly
* Merge branch 'tb-rework' into 'melodic-devel'
  Initial rework
  See merge request dingo/dingo_robot!2
* [dingo_bringup] Symlinked launch files.
* [dingo_bringup] Update instrall script and made it executable.
* [dingo_bringup] Set dingo_bringup as executable.
* [dingo_bringup] Maded script executable.
* Swapped dependency location since it is used by dingo_bringup and not dingo_base.
* [dingo_bringup] Added script to set environment variable for dingo-o
* Unified dingo launch files.  Also, removed connman file in install.
* Merge branch 'cib-accessories' into 'melodic-devel'
  Enable Hokuyo Lidar & Realsense in dingo_bringup
  See merge request dingo/dingo_robot!3
* Enable Hokuyo Lidar & Realsense in dingo_bringup
* Merge branch 'jh-melodic-devel' into 'melodic-devel'
  Initial Dingo robot implementation; not yet tested on real board
  See merge request dingo/dingo_robot!1
* Initial Dingo robot implementation; not yet tested on real board
* Contributors: Chris Iverach-Brereton, Jason Higgins, Tony Baltovski

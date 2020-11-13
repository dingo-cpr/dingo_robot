^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dingo_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2020-11-13)
------------------
* Bump CMake version to avoid CMP0048 warning.
* [dingo_base] Reduced log spamming.
* [dingo_base] Fixed not using function for ros::ok().
* [dingo_base] Updated encoder count for mmp-s14 motor config.
* [dingo_base] Added motor parameters as rosparams to be loaded as a YAML which is set by an enviroment variable.
* [dingo_base] Updated I gain.
* Merge branch 'melodic-devel' of gitlab.clearpathrobotics.com:dingo/dingo_robot into melodic-devel
* Merge branch 'tb-rework' into 'melodic-devel'
  Initial rework
  See merge request dingo/dingo_robot!2
* [dingo_base] roslint fixes.
* [dingo_base] Fixed mag_config_default.yaml location.
* [dingo_base] Updated mag config params.
* [dingo_base] Moved network logger before rosserial server.
* [dingo_base] Updated logger to use ROS based logging for its own logging.
* [dingo_base] Updated CAN IDs and directions for joints.
* [dingo_base] Added missing join for logger thread.
* [dingo_base] Flipped motor signs.
* [dinog_hardware] Fixed typo.
* [dingo_base] Added network based logger.
* [dingo_lighting] Updated drive lighting patterns.
* [dingo_hardware] Reduced logging by throttling.
* [dingo_hardware] Updated motor parameters.
* [dingo_base] Reworkd Dingo model selection.
* Swapped dependency location since it is used by dingo_bringup and not dingo_base.
* Unified dingo launch files.  Also, removed connman file in install.
* Merge branch 'jh-melodic-devel' into 'melodic-devel'
  Initial Dingo robot implementation; not yet tested on real board
  See merge request dingo/dingo_robot!1
* Initial Dingo robot implementation; not yet tested on real board
* Contributors: Chris Iverach-Brereton, Jason Higgins, Tony Baltovski

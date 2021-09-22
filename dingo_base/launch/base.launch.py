# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	description_path = os.path.join(get_package_share_directory('dingo_description'), 'launch', 'description.launch.py')

	# Specify the actions
	description_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(description_path),
		launch_arguments={'physical_robot' : 'true'}.items()
	)

	start_dingo_control = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(
			get_package_share_directory('dingo_control'), 'launch', 'control.launch.py')),
		launch_arguments={'physical_robot' : 'true'}.items()
	)

	ld = LaunchDescription()

	# Add any conditioned actions
	ld.add_action(description_cmd)
	ld.add_action(start_dingo_control)

	return ld
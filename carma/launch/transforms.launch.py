# Copyright (C) 2022 LEIDOS.
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

from launch import LaunchDescription
from launch_ros.actions import Node
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
import os


def generate_launch_description():
    """
    Launch robot_state_publisher to read in URDF file
    """

    # Since the file needs to be actually read a substition does not work here
    # For now the path must be hardcoded so it can be read into a string and passed as a parameter
    with open('/opt/carma/vehicle/calibration/urdf/carma.urdf', 'r') as in_file:
        robot_description = in_file.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=GetCurrentNamespace(),
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        start_robot_state_publisher_cmd
    ])
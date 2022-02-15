# Copyright (C) 2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from ament_index_python import get_package_share_directory
from launch.actions import Shutdown
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from launch.substitutions import LaunchConfiguration

import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import set_remap

def generate_launch_description():

    # Launch file for launching the nodes in the CARMA hardware interface stack
    vehicle_ssc_param_dir = LaunchConfiguration('vehicle_ssc_param_dir')
    mock_drivers = LaunchConfiguration('mock_drivers')
    launch_drivers = LaunchConfiguration('launch_drivers')

    # Driver Launch File if Using Actual Drivers
    # If launch_arg - launch drivers.launch
    drivers_node =  GroupAction(
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/drivers.launch.py']),
            ),
            launch_arguments={
                'mock_drivers' : mock_drivers,
                'vehicle_ssc_param_dir' : vehicle_ssc_param_dir
            }.items()
        ]
    )



    return LaunchDescription([        
        if launch_arg:
            drivers_node
    ])
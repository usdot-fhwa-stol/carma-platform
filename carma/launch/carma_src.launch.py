# Copyright (C) 2021 LEIDOS.
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

from ament_index_python import get_package_share_directory
from launch.actions import Shutdown
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import EnvironmentVariable
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    """
    Launch CARMA System.
    """

    system_controller_param_file = os.path.join(
        get_package_share_directory('system_controller'), 'config/config.yaml')

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    vehicle_characteristics_dir = [vehicle_calibration_dir, "/identifiers/UniqueVehicleParams.yaml"]

    # Nodes

    environment_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_ENV_NS', default_value='environment')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/environment.launch.py'])
            ),
        ]
    )

    v2x_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_MSG_NS', default_value='message')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/message.launch.py']),
                launch_arguments = { 'vehicle_characteristics_dir': vehicle_characteristics_dir }.items()
            ),
        ]
    )

    system_controller = Node(
        package='system_controller',
        name='system_controller',
        executable='system_controller',
        parameters=[ system_controller_param_file ],
        on_exit = Shutdown(), # Mark the subsystem controller as required for segfaults
        arguments=['--ros-args', '--log-level', GetLogLevel('system_controller', env_log_levels)]
    )

    return LaunchDescription([
        environment_group,
        v2x_group,
        system_controller
    ])

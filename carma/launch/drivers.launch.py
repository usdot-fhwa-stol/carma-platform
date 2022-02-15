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
from ament_index_python import get_package_share_directory
from launch.actions import Shutdown
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    Launch the subsystem controller for the hardware interface subsystem.
    The actual drivers will not be launched in this file and will instead be launched in the carma-config.
    """
    
    vehicle_config_param_file = LaunchConfiguration('vehicle_config_param_file')
    declare_vehicle_config_param_file_arg = DeclareLaunchArgument(
        name = 'vehicle_config_param_file',
        default_value = "/opt/carma/vehicle/config/VehicleConfigParams.yaml",
        description = "Path to file contain vehicle configuration parameters"
    )

    subsystem_controller_param_file = os.path.join(
        get_package_share_directory('subsystem_controllers'), 'config/drivers_controller_config.yaml')
    
    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    # subsystem_controller which orchestrates the lifecycle of this subsystem's components
    subsystem_controller = Node(
        package='subsystem_controllers',
        name='drivers_controller',
        executable='drivers_controller',
        parameters=[ subsystem_controller_param_file ],  # TODO add the vehicle_config_param_file
        on_exit= Shutdown(), # Mark the subsystem controller as required
        arguments=['--ros-args', '--log-level', GetLogLevel('subsystem_controllers', env_log_levels)]
    )

    return LaunchDescription([
        declare_vehicle_config_param_file_arg,
        subsystem_controller
    ])
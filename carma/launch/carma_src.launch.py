# Copyright (C) 2021-2022 LEIDOS.
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
from launch.actions import DeclareLaunchArgument

import os

def generate_launch_description():
    """
    Launch CARMA System.
    """

    system_controller_param_file = os.path.join(
        get_package_share_directory('system_controller'), 'config/config.yaml')

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    # Declare the vehicle_calibration_dir launch argument
    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    declare_vehicle_calibration_dir_arg = DeclareLaunchArgument(
        name = 'vehicle_calibration_dir', 
        default_value = "/opt/carma/vehicle/calibration",
        description = "Path to folder containing vehicle calibration directories"
    )

    vehicle_config_dir = LaunchConfiguration('vehicle_config_dir')
    declare_vehicle_config_dir_arg = DeclareLaunchArgument(
        name = 'vehicle_config_dir', 
        default_value = "/opt/carma/vehicle/config",
        description = "Path to file containing vehicle config directories"
    )

    vehicle_characteristics_param_file = LaunchConfiguration('vehicle_characteristics_param_file')
    declare_vehicle_characteristics_param_file_arg = DeclareLaunchArgument(
        name = 'vehicle_characteristics_param_file', 
        default_value = [vehicle_calibration_dir, "/identifiers/UniqueVehicleParams.yaml"],
        description = "Path to file containing unique vehicle characteristics"
    )

    # Declare the vehicle_config_param_file launch argument
    vehicle_config_param_file = LaunchConfiguration('vehicle_config_param_file')
    declare_vehicle_config_param_file_arg = DeclareLaunchArgument(
        name = 'vehicle_config_param_file',
        default_value = [vehicle_config_dir, "/VehicleConfigParams.yaml"],
        description = "Path to file contain vehicle configuration parameters"
    )

    #Declare the route file folder launch argument
    route_file_folder = LaunchConfiguration('route_file_folder')
    declare_route_file_folder = DeclareLaunchArgument(
        name = 'route_file_folder',
        default_value='/opt/carma/routes/',
        description = 'Path of folder containing routes to load'
    )

    # Declare enable_guidance_plugin_validate
    enable_guidance_plugin_validator = LaunchConfiguration('enable_guidance_plugin_validator')
    declare_enable_guidance_plugin_validator = DeclareLaunchArgument(
        name = 'enable_guidance_plugin_validator', 
        default_value='false', 
        description='Flag indicating whether the Guidance Plugin Validator node will actively validate guidance strategic, tactical, and control plugins'
    )

    # Declare strategic_plugins_to_validate
    strategic_plugins_to_validate = LaunchConfiguration('strategic_plugins_to_validate')
    declare_strategic_plugins_to_validate = DeclareLaunchArgument(
        name = 'strategic_plugins_to_validate',
        default_value = '[]',
        description = 'List of String: Guidance Strategic Plugins that will be validated by the Guidance Plugin Validator Node if enabled'
    )

    # Declare tactical_plugins_to_validate
    tactical_plugins_to_validate = LaunchConfiguration('tactical_plugins_to_validate')
    declare_tactical_plugins_to_validate = DeclareLaunchArgument(
        name = 'tactical_plugins_to_validate',
        default_value='[]',
        description='List of String: Guidance Tactical Plugins that will be validated by the Guidance Plugin Validator Node if enabled'
    )

    # Declare strategic_plugins_to_validate
    control_plugins_to_validate = LaunchConfiguration('control_plugins_to_validate')
    declare_control_plugins_to_validate = DeclareLaunchArgument(
        name = 'control_plugins_to_validate',
        default_value= '[]',
        description='List of String: Guidance Control Plugins that will be validated by the Guidance Plugin Validator Node if enabled'
    )
    
    # Declare port
    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        name = 'port',
        default_value= "9090",
        description='The default port for rosbridge is 909'
    )

    # Nodes

    transform_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_TF_NS', default_value='/')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/transforms.launch.py'])
            ),
        ]
    )

    environment_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_ENV_NS', default_value='environment')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/environment.launch.py']),
                launch_arguments = { 
                    'subsystem_controller_param_file' : [vehicle_config_dir, '/SubsystemControllerParams.yaml'],
                    'vehicle_config_param_file' : vehicle_config_param_file,
                    'vehicle_characteristics_param_file' : vehicle_characteristics_param_file
                    }.items()
            ),
        ]
    )

    localization_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_LOCZ_NS', default_value='localization')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/localization.launch.py']),
                launch_arguments = { 
                    'subsystem_controller_param_file' : [vehicle_config_dir, '/SubsystemControllerParams.yaml'],
                }.items()
            )
        ]
    )

    v2x_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_MSG_NS', default_value='message')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/message.launch.py']),
                launch_arguments = { 
                    'vehicle_characteristics_param_file' : vehicle_characteristics_param_file,
                    'vehicle_config_param_file' : vehicle_config_param_file,
                    'subsystem_controller_param_file' : [vehicle_config_dir, '/SubsystemControllerParams.yaml']
                }.items()
            ),
        ]
    )

    guidance_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_GUIDE_NS', default_value='guidance')),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/guidance.launch.py']),
                launch_arguments={
                    'route_file_folder' : route_file_folder,
                    'vehicle_characteristics_param_file' : vehicle_characteristics_param_file, 
                    'vehicle_config_param_file' : vehicle_config_param_file,
                    'enable_guidance_plugin_validator' : enable_guidance_plugin_validator,
                    'strategic_plugins_to_validate' : strategic_plugins_to_validate,
                    'tactical_plugins_to_validate' : tactical_plugins_to_validate,
                    'control_plugins_to_validate' : control_plugins_to_validate,
                    'subsystem_controller_param_file' : [vehicle_config_dir, '/SubsystemControllerParams.yaml'],
                }.items()
            ),
        ]
    )

    drivers_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/drivers.launch.py']),
                launch_arguments = { 
                    'vehicle_config_param_file' : vehicle_config_param_file,
                    'subsystem_controller_param_file' : [vehicle_config_dir, '/SubsystemControllerParams.yaml'],
                }.items()
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

    ui_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_UI_NS', default_value='ui')),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ui.launch.py']),
                launch_arguments={
                'port' : port
                }.items()
            ),
        ]
    )

    return LaunchDescription([
        declare_vehicle_calibration_dir_arg,
        declare_vehicle_config_dir_arg,
        declare_vehicle_characteristics_param_file_arg,
        declare_vehicle_config_param_file_arg,
        declare_route_file_folder,
        declare_enable_guidance_plugin_validator,
        declare_strategic_plugins_to_validate,
        declare_tactical_plugins_to_validate,
        declare_control_plugins_to_validate,
        declare_port,
        drivers_group,
        transform_group,
        environment_group,
        localization_group,
        v2x_group,
        guidance_group, 
        ui_group,
        system_controller
    ])

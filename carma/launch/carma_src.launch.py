# Copyright (C) 2021-2023 LEIDOS.
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
import launch
from launch.actions import Shutdown
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import EnvironmentVariable
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

from tracetools_launch.action import Trace

from datetime import datetime
import os

def create_ros2_tracing_action(context, *args, **kwargs):
    """
    Opaque Function for generating a 'Trace' ROS 2 launch action, which is dependent on the
    'ROS_LOG_DIR' EnvironmentVariable and the 'is_ros2_tracing_enabled' LaunchConfiguration.

    NOTE: This Opaque Function is required in order to evaluate the 'ROS_LOG_DIR' environment
    variable as a string.
    """
    log_directory_string = launch.substitutions.EnvironmentVariable('ROS_LOG_DIR').perform(context)

    trace = GroupAction( 
        condition=IfCondition(LaunchConfiguration('is_ros2_tracing_enabled')),
        actions=[
            Trace(
                base_path = log_directory_string,
                session_name='my-tracing-session-' + str(datetime.now().strftime('%Y-%m-%d_%H%M%S')),
                events_kernel = [], # Empty since kernel tracing is not enabled for CARMA Platform
            )
        ]
    )

    return [trace]

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

    # Declare control_plugins_to_validate
    control_plugins_to_validate = LaunchConfiguration('control_plugins_to_validate')
    declare_control_plugins_to_validate = DeclareLaunchArgument(
        name = 'control_plugins_to_validate',
        default_value= '[]',
        description='List of String: Guidance Control Plugins that will be validated by the Guidance Plugin Validator Node if enabled'
    )

    # Declare enable_opening_tunnels
    enable_opening_tunnels = LaunchConfiguration('enable_opening_tunnels')
    declare_enable_opening_tunnels = DeclareLaunchArgument(
        name = 'enable_opening_tunnels',
        default_value= 'False',
        description='Flag to enable opening http tunnesl to CARMA Cloud'
    )
    
    # Declare port
    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        name = 'port',
        default_value= "9090",
        description='The default port for rosbridge is 909'
    )

    # Declare launch arguments for points_map_loader
    load_type = LaunchConfiguration('load_type')
    declare_load_type= DeclareLaunchArgument(name = 'load_type', default_value = "noupdate")

    single_pcd_path = LaunchConfiguration('single_pcd_path')
    declare_single_pcd_path = DeclareLaunchArgument(name='single_pcd_path', default_value="['/opt/carma/maps/pcd_map.pcd']")

    area = LaunchConfiguration('area')
    declare_area = DeclareLaunchArgument(name='area', default_value="1x1")

    arealist_path = LaunchConfiguration('arealist_path')
    declare_arealist_path = DeclareLaunchArgument(name='arealist_path', default_value="/opt/carma/maps/arealist.txt")

    vector_map_file = LaunchConfiguration('vector_map_file')
    declare_vector_map_file = DeclareLaunchArgument(name='vector_map_file', default_value='/opt/carma/maps/vector_map.osm')

    simulation_mode = LaunchConfiguration('simulation_mode')
    declare_simulation_mode = DeclareLaunchArgument(name='simulation_mode', default_value = 'False', description = 'True if CARMA Platform is launched with CARLA Simulator')

    is_ros2_tracing_enabled = LaunchConfiguration('is_ros2_tracing_enabled')
    declare_is_ros2_tracing_enabled = DeclareLaunchArgument(
        name='is_ros2_tracing_enabled', 
        default_value = 'False', 
        description = 'True if user wants ROS 2 Tracing logs to be generated from CARMA Platform.')

    # Launch ROS2 rosbag logging
    ros2_rosbag_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ros2_rosbag.launch.py']),
                launch_arguments = {
                    'vehicle_config_dir' : vehicle_config_dir,
                    'vehicle_config_param_file' : vehicle_config_param_file
                    }.items()
            )
        ]
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
                    'vehicle_characteristics_param_file' : vehicle_characteristics_param_file,
                    'vector_map_file' : vector_map_file
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
                    'load_type' : load_type,
                    'single_pcd_path' : single_pcd_path,
                    'area' : area,
                    'arealist_path' : arealist_path,
                    'vector_map_file' : vector_map_file,
                    'vehicle_calibration_dir': vehicle_calibration_dir,
                    'simulation_mode': simulation_mode,
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
                    'enable_opening_tunnels'  : enable_opening_tunnels,
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
        declare_enable_opening_tunnels,
        declare_port,
        declare_load_type,
        declare_single_pcd_path,
        declare_area,
        declare_arealist_path,
        declare_vector_map_file,
        declare_simulation_mode,
        declare_is_ros2_tracing_enabled,
        drivers_group,
        transform_group,
        environment_group,
        localization_group,
        v2x_group,
        guidance_group, 
        ros2_rosbag_launch,
        ui_group,
        system_controller,
        OpaqueFunction(function=create_ros2_tracing_action)
    ])

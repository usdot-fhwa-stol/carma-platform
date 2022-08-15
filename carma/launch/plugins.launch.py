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
from launch.actions import DeclareLaunchArgument

# Launch file for launching the nodes in the CARMA guidance stack

def generate_launch_description():

    route_file_folder = LaunchConfiguration('route_file_folder')
    vehicle_characteristics_param_file = LaunchConfiguration('vehicle_characteristics_param_file')
    enable_guidance_plugin_validator = LaunchConfiguration('enable_guidance_plugin_validator')
    strategic_plugins_to_validate = LaunchConfiguration('strategic_plugins_to_validate')
    tactical_plugins_to_validate = LaunchConfiguration('tactical_plugins_to_validate')
    control_plugins_to_validate = LaunchConfiguration('control_plugins_to_validate')
    
    vehicle_config_param_file = LaunchConfiguration('vehicle_config_param_file')

    inlanecruising_plugin_file_path = os.path.join(
        get_package_share_directory('inlanecruising_plugin'), 'config/parameters.yaml')

    route_following_plugin_file_path = os.path.join(
        get_package_share_directory('route_following_plugin'), 'config/parameters.yaml')

    stop_and_wait_plugin_param_file = os.path.join(
        get_package_share_directory('stop_and_wait_plugin'), 'config/parameters.yaml')        

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    carma_plugins_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_guidance_plugins_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                    package='inlanecruising_plugin',
                    plugin='inlanecruising_plugin::InLaneCruisingPluginNode',
                    name='inlanecruising_plugin',
                    extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('inlanecruising_plugin', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] ),
                ],
                parameters=[
                    inlanecruising_plugin_file_path,
                    vehicle_config_param_file
                ]
            ),
            ComposableNode(
                    package='stop_and_wait_plugin',
                    plugin='stop_and_wait_plugin::StopandWaitNode',
                    name='stop_and_wait_plugin',
                    extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('stop_and_wait_plugin', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] ),
                ],
                parameters=[
                    stop_and_wait_plugin_param_file,
                    vehicle_config_param_file
                ]
            ),
            ComposableNode(
                    package='route_following_plugin',
                    plugin='route_following_plugin::RouteFollowingPlugin',
                    name='route_following_plugin',
                    extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('route_following_plugin', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] ),
                    ("current_velocity", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ] ),
                    ("maneuver_plan", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/final_maneuver_plan" ] ),
                    ("upcoming_lane_change_status", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/upcoming_lane_change_status" ] ),
                ],
                parameters=[
                    route_following_plugin_file_path,
                    vehicle_config_param_file
                ]
            )
        ]
    )

    return LaunchDescription([    
        carma_plugins_container
    ]) 

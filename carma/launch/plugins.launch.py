# Copyright (C) 2022-2023 LEIDOS.
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
    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    vehicle_characteristics_param_file = LaunchConfiguration('vehicle_characteristics_param_file')
    enable_guidance_plugin_validator = LaunchConfiguration('enable_guidance_plugin_validator')
    strategic_plugins_to_validate = LaunchConfiguration('strategic_plugins_to_validate')
    tactical_plugins_to_validate = LaunchConfiguration('tactical_plugins_to_validate')
    control_plugins_to_validate = LaunchConfiguration('control_plugins_to_validate')
    
    vehicle_config_param_file = LaunchConfiguration('vehicle_config_param_file')

    yield_plugin_file_path = os.path.join(
        get_package_share_directory('yield_plugin'), 'config/parameters.yaml')        

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    carma_yield_plugin_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_yield_plugin_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                    package='yield_plugin',
                    plugin='yield_plugin::YieldPluginNode',
                    name='yield_plugin',
                    extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('yield_plugin', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("external_object_predictions", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/external_object_simulation_list" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] ),
                    ("outgoing_mobility_response", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/outgoing_mobility_response" ] ),
                    ("incoming_mobility_request", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_request" ] ),
                    ("cooperative_lane_change_status", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/cooperative_lane_change_status" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/map_param_loader/georeference"]),
                    ("bsm_outbound", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/bsm_outbound" ] ),
                ],
                parameters=[
                    yield_plugin_file_path,
                    vehicle_config_param_file
                ]
            ),
        ]
    )


    return LaunchDescription([    
        #carma_inlanecruising_plugin_container, 
        #carma_route_following_plugin_container, 
        #carma_approaching_emergency_vehicle_plugin_container,
        #carma_stop_and_wait_plugin_container, 
        #carma_sci_strategic_plugin_container, 
        #carma_stop_and_dwell_strategic_plugin_container,
        #carma_lci_strategic_plugin_container, 
        #carma_stop_controlled_intersection_tactical_plugin_container, 
        #carma_cooperative_lanechange_plugins_container, 
        carma_yield_plugin_container
        #carma_light_controlled_intersection_plugins_container, 
        #carma_pure_pursuit_wrapper_container, 
        #platooning_strategic_plugin_container, 
        #platooning_tactical_plugin_container,
        #intersection_transit_maneuvering_container
    ])

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
    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
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

    light_controlled_intersection_tactical_plugin_param_file = os.path.join(
        get_package_share_directory('light_controlled_intersection_tactical_plugin'), 'config/parameters.yaml')          

    cooperative_lanechange_param_file = os.path.join(
        get_package_share_directory('cooperative_lanechange'), 'config/parameters.yaml')      

    platoon_strategic_ihp_param_file = os.path.join(
        get_package_share_directory('platoon_strategic_ihp'), 'config/parameters.yaml')    

    sci_strategic_plugin_file_path = os.path.join(
        get_package_share_directory('sci_strategic_plugin'), 'config/parameters.yaml')     

    yield_plugin_file_path = os.path.join(
        get_package_share_directory('yield_plugin'), 'config/parameters.yaml')        

    platoon_tactical_ihp_param_file = os.path.join(
        get_package_share_directory('platooning_tactical_plugin'), 'config/parameters.yaml') 
    
    stop_controlled_intersection_tactical_plugin_file_path = os.path.join(
        get_package_share_directory('stop_controlled_intersection_tactical_plugin'), 'config/parameters.yaml') 

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    pure_pursuit_tuning_parameters = [vehicle_calibration_dir, "/pure_pursuit/calibration.yaml"]

    carma_plugins_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_guidance_core_plugins_container',
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
            ),
            ComposableNode(
                package='sci_strategic_plugin',
                plugin='sci_strategic_plugin::SCIStrategicPlugin',
                name='sci_strategic_plugin',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('sci_strategic_plugin', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] ),
                    ("maneuver_plan", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/final_maneuver_plan" ] ),
                    ("outgoing_mobility_operation", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/outgoing_mobility_operation" ] ),
                    ("incoming_mobility_operation", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_operation" ] ),
                    ("bsm_outbound", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/bsm_outbound" ] ),
                    ("current_pose", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/current_pose" ] ),
                ],
                parameters=[
                    sci_strategic_plugin_file_path,
                    vehicle_config_param_file
                ]
            ),
            ComposableNode(
                package='cooperative_lanechange',
                plugin='cooperative_lanechange::CooperativeLaneChangePlugin',
                name='cooperative_lanechange',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('cooperative_lanechange', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] ),
                    ("current_velocity", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ] ),
                    ("cooperative_lane_change_status", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/cooperative_lane_change_status" ] ),
                    ("bsm_outbound", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/bsm_outbound" ] ),
                    ("outgoing_mobility_request", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/outgoing_mobility_request" ] ),
                    ("incoming_mobility_response", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_response" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/map_param_loader/georeference" ] ),
                    ("current_pose", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/current_pose" ] )
                ],
                parameters=[
                    cooperative_lanechange_param_file,
                    vehicle_characteristics_param_file,
                    vehicle_config_param_file
                ]
            ),
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
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
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
            ComposableNode(
                package='light_controlled_intersection_tactical_plugin',
                plugin='light_controlled_intersection_tactical_plugin::LightControlledIntersectionTransitPluginNode',
                name='light_controlled_intersection_tactical_plugin',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('light_controlled_intersection_tactical_plugin', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] )
                ],
                parameters=[
                    vehicle_config_param_file,
                    light_controlled_intersection_tactical_plugin_param_file
                ]     
            ),
            ComposableNode(
                package='stop_controlled_intersection_tactical_plugin',
                plugin='stop_controlled_intersection_tactical_plugin::StopControlledIntersectionTacticalPlugin',
                name='stop_controlled_intersection_tactical_plugin',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('stop_controlled_intersection_tactical_plugin', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] )
                ],
                parameters=[
                    stop_controlled_intersection_tactical_plugin_file_path,
                    vehicle_config_param_file
                ]
            ),
            ComposableNode(
                    package='pure_pursuit_wrapper',
                    plugin='pure_pursuit_wrapper::PurePursuitWrapperNode',
                    name='pure_pursuit_wrapper',
                    extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('pure_pursuit_wrapper', env_log_levels) }
                ],
                remappings = [
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("ctrl_raw", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/ctrl_raw" ] ),
                    ("pure_pursuit_wrapper/plan_trajectory", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/pure_pursuit/plan_trajectory" ] ),
                    ("current_pose", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/current_pose" ] ),
                    ("vehicle/twist", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ] ),
                ],
                parameters=[
                    vehicle_characteristics_param_file, #vehicle_response_lag
                    pure_pursuit_tuning_parameters #pure_pursuit calibration parameters
                ]
            ),
        ]
    )
    
    platooning_plugins_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='platooning_plugins_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='platoon_strategic_ihp',
                plugin='platoon_strategic_ihp::Node',
                name='platoon_strategic_ihp_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('platoon_strategic_ihp', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/map_param_loader/georeference" ] ),
                    ("outgoing_mobility_response", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/outgoing_mobility_response" ] ),
                    ("outgoing_mobility_request", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/outgoing_mobility_request" ] ),
                    ("outgoing_mobility_operation", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/outgoing_mobility_operation" ] ),
                    ("incoming_mobility_request", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_request" ] ),
                    ("incoming_mobility_response", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_response" ] ),
                    ("incoming_mobility_operation", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_operation" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("twist_raw", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/twist_raw" ] ),
                    ("platoon_info", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/platoon_info" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] ),
                    ("current_velocity", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ] ),
                    ("current_pose", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/current_pose" ] ),
                ],
                parameters=[ 
                    platoon_strategic_ihp_param_file,
                    vehicle_config_param_file
                ]
            ),      
            ComposableNode(
                package='platooning_tactical_plugin',
                plugin='platooning_tactical_plugin::Node',
                name='platooning_tactical_plugin_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('platooning_tactical_plugin', env_log_levels) }
                ],
                remappings = [
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/map_param_loader/georeference" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("plugin_discovery", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugin_discovery" ] ),
                    ("route", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route" ] ),
                ],
                parameters=[ platoon_tactical_ihp_param_file, vehicle_config_param_file ]
            ),
        ]
    )

    return LaunchDescription([    
        carma_plugins_container,
        platooning_plugins_container
    ]) 

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

# Launch file for launching the nodes in the CARMA guidance stack

def generate_launch_description():

    route_file_folder = LaunchConfiguration('route_file_folder')
    vehicle_characteristics_param_file = LaunchConfiguration('vehicle_characteristics_param_file')
    enable_guidance_plugin_validator = LaunchConfiguration('enable_guidance_plugin_validator')
    strategic_plugins_to_validate = LaunchConfiguration('strategic_plugins_to_validate')
    tactical_plugins_to_validate = LaunchConfiguration('tactical_plugins_to_validate')
    control_plugins_to_validate = LaunchConfiguration('control_plugins_to_validate')

    subsystem_controller_param_file = os.path.join(
        get_package_share_directory('subsystem_controllers'), 'config/guidance_controller_config.yaml')

    mobilitypath_visualizer_param_file = os.path.join(
        get_package_share_directory('mobilitypath_visualizer'), 'config/params.yaml')
    
    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')


    # Nodes
    carma_guidance_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_guidance_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[

            ComposableNode(
                package='mobilitypath_visualizer',
                plugin='mobilitypath_visualizer::MobilityPathVisualizer',
                name='mobilitypath_visualizer_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('mobilitypath_visualizer', env_log_levels) }
                ],
                remappings = [
                    ("mobility_path_msg", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/mobility_path_msg" ] ),
                    ("incoming_mobility_path", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_path" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NZ', default_value=''), "/map_param_loader/georeference"])
                ],
                parameters=[
                    vehicle_characteristics_param_file,
                    mobilitypath_visualizer_param_file
                ]
            ),
        ]
    )

    # subsystem_controller which orchestrates the lifecycle of this subsystem's components
    subsystem_controller = Node(
        package='subsystem_controllers',
        name='guidance_controller',
        executable='guidance_controller',
        parameters=[ subsystem_controller_param_file ],
        on_exit= Shutdown(), # Mark the subsystem controller as required
        arguments=['--ros-args', '--log-level', GetLogLevel('subsystem_controllers', env_log_levels)]
    )

    return LaunchDescription([        
        carma_guidance_container,
        subsystem_controller
    ]) 

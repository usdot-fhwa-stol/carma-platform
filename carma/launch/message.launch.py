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


def generate_launch_description():
    """
    Launch V2X subsystem nodes.
    """

    subsystem_controller_param_file = os.path.join(
        get_package_share_directory('subsystem_controllers'), 'config/v2x_controller_config.yaml')

    mobilitypath_publisher_param_file = os.path.join(
        get_package_share_directory('mobilitypath_publisher'), 'config/parameters.yaml')

    vehicle_characteristics_dir = LaunchConfiguration('vehicle_characteristics_dir')

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    # Nodes
    carma_v2x_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_v2x_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[

            ComposableNode(
                package='mobilitypath_publisher',
                plugin='mobilitypath_publisher::MobilityPathPublication',
                name='mobilitypath_publisher_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('mobilitypath_publisher', env_log_levels) }
                ],
                remappings=[
                    ("plan_trajectory", "/guidance/plan_trajectory"),               # TODO: Use environment variable
                    ("georeference", "/localization/map_param_loader/georeference") # TODO: Use environment variable
                ],
                parameters=[ 
                    mobilitypath_publisher_param_file,
                    vehicle_characteristics_dir
                ]
            ),
        ]
    )

    # subsystem_controller which orchestrates the lifecycle of this subsystem's components
    subsystem_controller = Node(
        package='subsystem_controllers',
        name='v2x_controller',
        executable='v2x_controller',
        parameters=[ subsystem_controller_param_file ],
        on_exit= Shutdown(), # Mark the subsystem controller as required
        arguments=['--ros-args', '--log-level', GetLogLevel('subsystem_controllers', env_log_levels)]
    )

    return LaunchDescription([
        carma_v2x_container,
        subsystem_controller
    ]) 

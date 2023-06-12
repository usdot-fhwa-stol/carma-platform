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
from launch.actions import DeclareLaunchArgument

import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import set_remap


def generate_launch_description():
    """
    Launch Localization subsystem nodes
    """

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    subsystem_controller_default_param_file = os.path.join(
        get_package_share_directory('subsystem_controllers'), 'config/localization_controller_config.yaml')

    subsystem_controller_param_file = LaunchConfiguration('subsystem_controller_param_file')
    declare_subsystem_controller_param_file_arg = DeclareLaunchArgument(
        name = 'subsystem_controller_param_file',
        default_value = subsystem_controller_default_param_file,
        description = "Path to file containing override parameters for the subsystem controller"
    )
    

    # Nodes
    # TODO add ROS2 localization nodes here
    
    gnss_to_map_convertor_param_file = os.path.join(
    get_package_share_directory('gnss_to_map_convertor'), 'config/parameters.yaml')

    localization_manager_convertor_param_file = os.path.join(
    get_package_share_directory('localization_manager'), 'config/parameters.yaml')

    # Declare launch arguments for points_map_loader
    load_type = LaunchConfiguration('load_type')
    declare_load_type= DeclareLaunchArgument(name = 'load_type', default_value = "noupdate")

    single_pcd_path = LaunchConfiguration('single_pcd_path')
    declare_single_pcd_path = DeclareLaunchArgument(name='single_pcd_path', default_value="['/opt/carma/maps/pcd_map.pcd']", description='Path to the map pcd file if using the noupdate load type')

    area = LaunchConfiguration('area')
    declare_area = DeclareLaunchArgument(name='area', default_value="1x1")

    arealist_path = LaunchConfiguration('arealist_path')
    declare_arealist_path = DeclareLaunchArgument(name='arealist_path', default_value="/opt/carma/maps/arealist.txt")

    vector_map_file = LaunchConfiguration('vector_map_file')
    declare_map_file = DeclareLaunchArgument(name='vector_map_file', default_value='vector_map.osm', description='Path to the map osm file if using the noupdate load type')

    gnss_to_map_convertor_container = ComposableNodeContainer(
    package='carma_ros2_utils',
    name='gnss_to_map_convertor_container',
    executable='carma_component_container_mt',
    namespace=GetCurrentNamespace(),
    composable_node_descriptions=[

        ComposableNode(
                package='gnss_to_map_convertor',
                plugin='gnss_to_map_convertor::Node',
                name='gnss_to_map_convertor',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('gnss_to_map_convertor', env_log_levels) }
                ],
                remappings=[
                    ("gnss_fix_fused",  [EnvironmentVariable('CARMA_INTR_NS', default_value=''),"/gnss_fix_fused"]),
                    ("georeference", "map_param_loader/georeference"),
                ],
                parameters=[ gnss_to_map_convertor_param_file ]
        )
    ])

    localization_manager_container = ComposableNodeContainer(
    package='carma_ros2_utils',
    name='localization_manager_container',
    executable='carma_component_container_mt',
    namespace=GetCurrentNamespace(),
    composable_node_descriptions=[
        ComposableNode(
                package='localization_manager',
                plugin='localization_manager::Node',
                name='localization_manager',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('localization_manager', env_log_levels) }
                ],
                remappings=[
                    
                ],
                parameters=[ localization_manager_convertor_param_file ]
        )
    ])
    
    ###Point Cloud Map file location and parameter loading process

    # map param/tf loader
    map_param_loader_container = ComposableNodeContainer(
    package='carma_ros2_utils',
    name='map_param_loader_container',
    executable='carma_component_container_mt',
    namespace=GetCurrentNamespace(),
    composable_node_descriptions=[
        ComposableNode(
                package='map_file_ros2',
                plugin='map_param_loader::MapParamLoader',
                name='map_param_loader',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('map_param_loader', env_log_levels) }
                ],
                remappings=[
                    ("georeference", "map_param_loader/georeference"),
                ],
                parameters=[ {'file_name' : vector_map_file } ]
        )
    ])

    # Point Cloud map file loading process
    pcd_map_file_loader_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='map_file_nodes_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[

            ComposableNode(
                package='map_file_ros2',
                plugin='points_map_loader::PointsMapLoader',
                name='points_map_loader',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('points_map_loader', env_log_levels) }
                ],
                parameters=[
                    {'load_type' : load_type },
                    {'pcd_path_parameter' : single_pcd_path },
                    {'area' : area },
                    {'path_area_list' : arealist_path }
                ]
            ),
        ]
    )

    # Dead Reckoner 
    dead_reckoner_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='dead_reckoner_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[

            ComposableNode(
                package='dead_reckoner_ros2',
                plugin='dead_reckoner::DeadReckoner',
                name='dead_reckoner',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('dead_reckoner_ros2', env_log_levels) }
                ],
                remappings=[
                    ("current_twist", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ]),
                    ("current_odom", "vehicle/odom")  
                ],
            ),
        ]
    )

    
    # subsystem_controller which orchestrates the lifecycle of this subsystem's components
    subsystem_controller = Node(
        package='subsystem_controllers',
        name='localization_controller',
        executable='localization_controller',
        parameters=[ subsystem_controller_default_param_file, subsystem_controller_param_file ], # Default file is loaded first followed by config file
        on_exit= Shutdown(), # Mark the subsystem controller as required
        arguments=['--ros-args', '--log-level', GetLogLevel('subsystem_controllers', env_log_levels)]
    )

    return LaunchDescription([
        declare_subsystem_controller_param_file_arg,       
        declare_load_type,
        declare_single_pcd_path,
        declare_area,
        declare_arealist_path,
        declare_map_file,
        gnss_to_map_convertor_container,
        localization_manager_container,
        dead_reckoner_container,
        map_param_loader_container,
        pcd_map_file_loader_container,
        subsystem_controller
    ]) 

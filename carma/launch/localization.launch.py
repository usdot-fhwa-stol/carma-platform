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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


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

    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    ndt_matching_param_file = [vehicle_calibration_dir, "/lidar_localizer/ndt_matching/params.yaml"]

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

    simulation_mode = LaunchConfiguration('simulation_mode')

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
                package='dead_reckoner',
                plugin='dead_reckoner::DeadReckoner',
                name='dead_reckoner',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('dead_reckoner', env_log_levels) }
                ],
                remappings=[
                    ("current_twist", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ]),
                    ("current_odom", "vehicle/odom")  
                ],
            ),
        ]
    )

   # NDT Matching
   # This namesapce sets the parameters which are not set by default in the ndt_matching.launch file
   # These parameters are not in the ndt_matching node private namespace

    ndt_matching_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='ndt_matching_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[

            ComposableNode(
                package='lidar_localizer_ros2',
                plugin='ndt_matching::NDTMatching',
                name='ndt_matching',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('ndt_matching', env_log_levels) }
                ],
                remappings=[
                    ("/config/ndt", "config/ndt"),
                    ("/imu_raw", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/imu_raw" ] ),
                    ('filtered_points', 'random_points'),
                    ('initialpose','managed_initialpose'),
                ],
                parameters=[
                    ndt_matching_param_file,
                    {'get_height' : True },
                    {'use_odom' : True },
                    {'use_gnss' : 0 },
                    {'gnss_reinit_fitness' : 10000.0 }, # Set to unreasonably high value to ensure no reinitialization occurs as it rarely works
                    {'base_frame': "base_link"}
                ]
            )
        ]
    )

    # EKF Localizer
    # Comment out to remove and change marked line in waypoint following.launch
    ekf_localizer_container = ComposableNodeContainer(
        condition=IfCondition(PythonExpression(['not ', simulation_mode])),
        package='carma_ros2_utils',
        name='ekf_localizer_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[

            ComposableNode(
                package='ekf_localizer',
                plugin='ekf_localizer::EKFLocalizer',
                name='ekf_localizer',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('ekf_localizer', env_log_levels) }
                ],
                remappings=[
                    ("in_pose","selected_pose"),
                    ("in_pose_with_covariance", "input_pose_with_cov_UNUSED"),
                    ("in_twist",  [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ]),
                    ("in_twist_with_covariance", "input_twist_with_covariance_UNUSED"),
                    ("initialpose", "managed_initialpose"),
                    ("ekf_pose", "current_pose"),
                    ("ekf_pose_with_covariance", "current_pose_with_covariance"),
                    # remap to namespace/nodename/topic_name
                    ("debug", "~/debug"),
                    ("debug/measured_pose", "~/debug/measured_pose"),
                    ("estimated_yaw_bias", "~/estimated_yaw_bias")
                ],
                parameters=[
                    {'show_debug_info': False},
                    {'predict_frequency': 50.0},
                    {'enable_yaw_bias_estimation': True},
                    {'extend_state_step': 50},
                    {'pose_frame_id': 'map'},
                    {'child_frame_id': 'base_link'},
                    {'pose_additional_delay': 0.0},
                    {'pose_measure_uncertainty_time': 0.01},
                    {'pose_rate': 10.0},
                    {'pose_gate_dist': 10000.0},
                    {'pose_stddev_x': 0.05},
                    {'pose_stddev_y': 0.05},
                    {'pose_stddev_yaw': 0.025},
                    {'use_pose_with_covariance': False},
                    {'twist_additional_delay': 0.0},
                    {'twist_rate': 30.0},
                    {'twist_gate_dist': 10000.0},
                    {'twist_stddev_vx': 0.2},
                    {'twist_stddev_wz': 0.03},
                    {'proc_stddev_yaw_c': 0.005},
                    {'proc_stddev_yaw_bias_c': 0.001},
                    {'proc_stddev_vx_c': 0.1},
                    {'proc_stddev_wz_c': 0.05}
                ],
            )
        ]
    )

    ### Lidar stack

    voxel_grid_filter_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='voxel_grid_filter_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                package='points_downsampler',
                plugin='voxel_grid_filter::VoxelGridFilter',
                name='voxel_grid_filter_node',
                extra_arguments=[
                     {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('voxel_grid_filter', env_log_levels) }
                ],
                parameters=[
                    {"points_topic": [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/lidar/points_raw" ]},
                    {"output_log": False},
                    {"measurement_range": 200.0},
                    {"voxel_leaf_size": 3.0}
                ],
            ),
        ]
    )

    random_filter_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='random_filter_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                package='points_downsampler',
                plugin='random_filter::RandomFilter',
                name='random_filter_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('random_filter', env_log_levels) }
                ],
                parameters=[
                    {"points_topic": "filtered_points"},  
                    {"output_log": False},
                    {"measurement_range": 200.0},
                    {"sample_num": 700}
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
        voxel_grid_filter_container,
        random_filter_container,
        map_param_loader_container,
        pcd_map_file_loader_container,
        ndt_matching_container,
        ekf_localizer_container,
        subsystem_controller
    ]) 
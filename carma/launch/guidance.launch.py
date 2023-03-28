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
from launch.substitutions import ThisLaunchFileDir
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from launch.substitutions import LaunchConfiguration

import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import set_remap
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import PushRosNamespace


# Launch file for launching the nodes in the CARMA guidance stack

def generate_launch_description():

    route_file_folder = LaunchConfiguration('route_file_folder')
    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    vehicle_characteristics_param_file = LaunchConfiguration('vehicle_characteristics_param_file')
    enable_guidance_plugin_validator = LaunchConfiguration('enable_guidance_plugin_validator')
    strategic_plugins_to_validate = LaunchConfiguration('strategic_plugins_to_validate')
    tactical_plugins_to_validate = LaunchConfiguration('tactical_plugins_to_validate')
    control_plugins_to_validate = LaunchConfiguration('control_plugins_to_validate')
    vehicle_config_dir = LaunchConfiguration('vehicle_config_dir')
    vehicle_config_param_file = LaunchConfiguration('vehicle_config_param_file')
    declare_vehicle_config_param_file_arg = DeclareLaunchArgument(
        name = 'vehicle_config_param_file',
        default_value = "/opt/carma/vehicle/config/VehicleConfigParams.yaml",
        description = "Path to file contain vehicle configuration parameters"
    )

    subsystem_controller_default_param_file = os.path.join(
        get_package_share_directory('subsystem_controllers'), 'config/guidance_controller_config.yaml')

    mobilitypath_visualizer_param_file = os.path.join(
        get_package_share_directory('mobilitypath_visualizer'), 'config/params.yaml')

    trajectory_executor_param_file = os.path.join(
        get_package_share_directory('trajectory_executor'), 'config/parameters.yaml')
    
    route_param_file = os.path.join(
        get_package_share_directory('route'), 'config/parameters.yaml')

    trajectory_visualizer_param_file = os.path.join(
        get_package_share_directory('trajectory_visualizer'), 'config/parameters.yaml')

    guidance_param_file = os.path.join(
        get_package_share_directory('guidance'), 'config/parameters.yaml')
    
    arbitrator_param_file_path = os.path.join(
        get_package_share_directory('arbitrator'), 'config/arbitrator_params.yaml')

    plan_delegator_param_file = os.path.join(
        get_package_share_directory('plan_delegator'), 'config/plan_delegator_params.yaml')
        
    port_drayage_plugin_param_file = os.path.join(
        get_package_share_directory('port_drayage_plugin'), 'config/parameters.yaml')

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    subsystem_controller_param_file = LaunchConfiguration('subsystem_controller_param_file')
    declare_subsystem_controller_param_file_arg = DeclareLaunchArgument(
        name = 'subsystem_controller_param_file',
        default_value = subsystem_controller_default_param_file,
        description = "Path to file containing override parameters for the subsystem controller"
    )

    # Below nodes are separated to individual container such that the nodes with reentrant services are within their separate container.
    # When all nodes are within single container, it is prone to fail throwing runtime_error, and it is currently hypothesized to be
    # because of this issue: https://github.com/ros2/rclcpp/issues/1212, where fix in the rclcpp library, so not able to be integrated at this moment:
    # https://github.com/ros2/rclcpp/pull/1241. This issue was first discovered in this carma issue: https://github.com/usdot-fhwa-stol/carma-platform/issues/1961  

    # Nodes
    carma_guidance_visualizer_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_guidance_visualizer_container',
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
                    ("mobility_path_msg", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/outgoing_mobility_path" ] ),
                    ("incoming_mobility_path", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_path" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/map_param_loader/georeference"])
                ],
                parameters=[
                    vehicle_characteristics_param_file,
                    mobilitypath_visualizer_param_file,
                    vehicle_config_param_file
                ]
            ),
            ComposableNode(
                package='trajectory_visualizer',
                plugin='trajectory_visualizer::TrajectoryVisualizer',
                name='trajectory_visualizer_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('trajectory_visualizer', env_log_levels) }
                ],
                parameters=[
                    trajectory_visualizer_param_file
                ]
            )
        ]
    )

    carma_plan_delegator_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_plan_delegator_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='plan_delegator',
                plugin='plan_delegator::PlanDelegator',
                name='plan_delegator',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('plan_delegator', env_log_levels) }
                ],
                remappings = [
                    ("current_velocity", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ] ),
                    ("current_pose", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/current_pose" ] ),
                    ("vehicle_status", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle_status" ] ),
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] ),
                    ("guidance_state", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/state" ] ),
                ],
                parameters=[
                    plan_delegator_param_file,
                    vehicle_config_param_file
                ]
            )
        ]
    )
    
    carma_trajectory_executor_and_route_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_trajectory_executor_and_route_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='route',
                plugin='route::Route',
                name='route_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('route', env_log_levels) }
                ],
                remappings = [
                    ("current_velocity", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/map_param_loader/georeference" ] ),
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] )
                ],
                parameters=[
                    {'route_file_path': route_file_folder},
                    route_param_file,
                    vehicle_config_param_file
                ]
            ),
            ComposableNode(
                package='trajectory_executor',
                plugin='trajectory_executor::TrajectoryExecutor',
                name='trajectory_executor_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('trajectory_executor', env_log_levels) }
                ],
                remappings = [
                    ("trajectory", "plan_trajectory"),
                ],
                parameters=[
                    trajectory_executor_param_file,
                    vehicle_config_param_file
                ]
            )
        ]
    )
    
    carma_arbitrator_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_arbitrator_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[    
            ComposableNode(
                package='arbitrator',
                plugin='arbitrator::ArbitratorNode',
                name='arbitrator',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('arbitrator', env_log_levels) }
                ],
                remappings = [
                    ("current_velocity", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist" ] ),
                    ("guidance_state", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/state" ] ),
                    ("semantic_map", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/semantic_map" ] ),
                    ("map_update", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/map_update" ] ),
                    ("roadway_objects", [ EnvironmentVariable('CARMA_ENV_NS', default_value=''), "/roadway_objects" ] ),
                    ("incoming_spat", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_spat" ] )
                ],
                parameters=[ 
                    arbitrator_param_file_path,
                    vehicle_config_param_file
                ]
            )
        ]
    )
    carma_guidance_worker_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_guidance_worker_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='guidance',
                plugin='guidance::GuidanceWorker',
                name='guidance_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('route', env_log_levels) }
                ],
                remappings = [
                    ("vehicle_status", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle_status" ] ),
                    ("robot_status", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/controller/robot_status" ] ),
                    ("enable_robotic", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/controller/enable_robotic" ] ),
                ],
                parameters=[
                    guidance_param_file
                ]
            )
        ]
    )
    
    carma_port_drayage_plugin_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_port_drayage_plugin_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='port_drayage_plugin',
                plugin='port_drayage_plugin::PortDrayagePlugin',
                name='port_drayage_plugin_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('port_drayage_plugin', env_log_levels) }
                ],
                remappings = [
                    ("guidance_state", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/state" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/map_param_loader/georeference" ] ),
                    ("current_pose", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/current_pose" ] ),
                    ("incoming_mobility_operation", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_operation" ] ),  
                    ("outgoing_mobility_operation", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/outgoing_mobility_operation" ] ),     
                    ("ui_instructions", [ EnvironmentVariable('CARMA_UI_NS', default_value=''), "/ui_instructions" ] )       
                ],
                parameters=[
                    port_drayage_plugin_param_file,
                    vehicle_characteristics_param_file
                ]     
            )
        ]
    )

    twist_filter_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='twist_filter_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='twist_filter',
                plugin='twist_filter::TwistFilter',
                name='twist_filter_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('twist_filter', env_log_levels) }
                ],
                remappings = [
                    ("/accel_cmd", ["accel_cmd" ] ),
                    ("/brake_cmd", ["brake_cmd" ] ),
                    ("/gear_cmd", ["gear_cmd" ] ),
                    ("/mode_cmd", ["mode_cmd" ] ),
                    ("/remote_cmd", ["remote_cmd" ] ),
                    ("/steer_cmd", ["steer_cmd" ] ),
                    ("/emergency_stop", ["emergency_stop" ] ),
                    ("/state_cmd", ["state_cmd" ] )
                ],
                parameters=[
                    vehicle_config_param_file,
                    {'lowpass_gain_linear_x':0.1},
                    {'lowpass_gain_angular_z':0.0},
                    {'lowpass_gain_steering_angle':0.1}
                ]     
            ),
            ComposableNode(
                package='twist_gate',
                plugin='TwistGate',
                name='twist_gate_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('twist_gate', env_log_levels) }
                ],
                remappings = [
                    ("vehicle_cmd", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle_cmd" ] ),
                    ("/lamp_cmd", ["lamp_cmd" ] ),
                    ("/twist_cmd", ["twist_cmd" ] ),
                    ("/decision_maker/state", ["decision_maker/state" ] ),
                    ("/ctrl_cmd",   ["ctrl_cmd" ] ),
                ],
                parameters = [
                    {'loop_rate':30.0},
                    {'use_decision_maker':False}
                ]
            )
        ]
    )

    # Launch plugins
    plugins_group = GroupAction(
        actions=[
            PushRosNamespace("plugins"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/plugins.launch.py']),
                launch_arguments={
                    'route_file_folder' : route_file_folder,
                    'vehicle_calibration_dir' : vehicle_calibration_dir,
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
      
    # subsystem_controller which orchestrates the lifecycle of this subsystem's components
    subsystem_controller = Node(
        package='subsystem_controllers',
        name='guidance_controller',
        executable='guidance_controller',
        parameters=[ subsystem_controller_default_param_file, subsystem_controller_param_file ],
        on_exit= Shutdown(), # Mark the subsystem controller as required
        arguments=['--ros-args', '--log-level', GetLogLevel('subsystem_controllers', env_log_levels)]
    )

    return LaunchDescription([  
        declare_vehicle_config_param_file_arg,
        declare_subsystem_controller_param_file_arg,  
        carma_trajectory_executor_and_route_container,
        carma_guidance_visualizer_container,    
        carma_guidance_worker_container,
        carma_plan_delegator_container,
        carma_arbitrator_container,
        carma_port_drayage_plugin_container,
        twist_filter_container,
        plugins_group,
        subsystem_controller
    ]) 

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

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import Shutdown
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
import os


def generate_launch_description():
    """
    Launch the subsystem controller for the hardware interface subsystem.
    The actual drivers will not be launched in this file and will instead be launched in the carma-config.
    """
    
    vehicle_config_param_file = LaunchConfiguration('vehicle_config_param_file')
    declare_vehicle_config_param_file_arg = DeclareLaunchArgument(
        name = 'vehicle_config_param_file',
        default_value = "/opt/carma/vehicle/config/VehicleConfigParams.yaml",
        description = "Path to file contain vehicle configuration parameters"
    )
    
    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    subsystem_controller_default_param_file = os.path.join(
        get_package_share_directory('subsystem_controllers'), 'config/drivers_controller_config.yaml')

    subsystem_controller_param_file = LaunchConfiguration('subsystem_controller_param_file')
    declare_subsystem_controller_param_file_arg = DeclareLaunchArgument(
        name = 'subsystem_controller_param_file',
        default_value = subsystem_controller_default_param_file,
        description = "Path to file containing override parameters for the subsystem controller"
    )

    lightbar_manager_param_file = os.path.join(
        get_package_share_directory('lightbar_manager'), 'config/params.yaml')

    lightbar_manager_container = ComposableNodeContainer(
        package='carma_ros2_utils', # rclcpp_components
        name='lightbar_manager_container',
        executable='lifecycle_component_wrapper_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='lightbar_manager',
                plugin='lightbar_manager::LightBarManager',
                name='lightbar_manager',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('lightbar_manager', env_log_levels) },
                    {'is_lifecycle_node': True} # Flag to allow lifecycle node loading in lifecycle wrapper
                ],
                remappings=[
                    ("state", [ EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/state" ] ),
                    ("set_lights", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/lightbar/set_lights" ] )
                ],
                parameters=[ lightbar_manager_param_file ]
            )
        ]
    )

    # subsystem_controller which orchestrates the lifecycle of this subsystem's components
    subsystem_controller = Node(
        package='subsystem_controllers',
        name='drivers_controller',
        executable='drivers_controller',
        parameters=[ subsystem_controller_default_param_file, subsystem_controller_param_file  ], 
        on_exit= Shutdown(), # Mark the subsystem controller as required
        arguments=['--ros-args', '--log-level', GetLogLevel('subsystem_controllers', env_log_levels)]
    )

    return LaunchDescription([
        declare_subsystem_controller_param_file_arg,
        declare_vehicle_config_param_file_arg,
        lightbar_manager_container,
        subsystem_controller
    ])
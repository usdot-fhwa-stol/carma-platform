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


def generate_launch_description():
    """
    UI/Ros_bridge nodes.
    """
    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(
        name = 'port', 
        default_value = "9090",
        description = "The default port for rosbridge is 9090"
    )
   
    carma_param_file = os.path.join(
        get_package_share_directory('carma'), 'ui/config/CommandAPIParams.yaml')

    ui_container = ComposableNodeContainer(
        package='carma_ros2_utils', # rclcpp_components
        name='ui_container',
        executable='lifecycle_component_wrapper_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='rosbridge_server',
                plugin='rosbridge_websocket',
                name='rosbridge_server',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('rosbridge_server', env_log_levels) },
                    {'is_lifecycle_node': True} # Flag to allow lifecycle node loading in lifecycle wrapper
                ],
                remappings=[
                    ("get_available_routes",[EnvironmentVariable('CARMA_GUIDE_NS', default_value=''),"/get_available_routes"]),
                    ("set_active_route",[EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/set_active_route"]), 
                    ("start_active_route",[EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/start_active_route"]),
                    ("get_available_routes", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/get_available_routes"]),
                    ("route_state", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route_state]"), 
                    ("route_event", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route_event]"),
                    ("route", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route"]),
                    ("get_system_version", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/get_system_version"]), 
                    ("state", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/state"]),  
                    ("ui_platoon_vehicle_info", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/ui_platoon_vehicle_info"]), 
                    ("plugins/available_plugins", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/available_plugins"]),
                    ("plugins/get_registered_plugins", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/get_registered_plugins"]), 
                    ("plugins/activate_plugin", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/activate_plugin"]),
                    ("get_available_routes", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/get_available_routes"]),
                    ("set_guidance_active", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/set_guidance_active"]), 
                    ("plugins/controlling_plugins", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/controlling_plugins"]),
                    ("traffic_signal_info", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/traffic_signal_info"]),
                    ("platooning_info", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/platooning_info"]), 
                    ("traffic_signal_info", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/traffic_signal_info"]),  
                    ("system_alert", "/system_alert"),  
                  
                    ("bsm", [EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_bsm"]),
                    
                    ("nav_sat_fix", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/gnss/nav_sat_fix"]), 
                    ("velocity", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist"]),
                    ("driver_discovery", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/driver_discovery"]),
                    ("get_drivers_with_capabilities", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/get_drivers_with_capabilities"]), 
                    ("controller/robotic_status", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/controller/robot_status"]),
                    ("controller/vehicle_cmd" , [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/controller/vehicle_cmd"]),
                    ("comms/outbound_binary_msg",[EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/comms/outbound_binary_msg"]), 
                    ("comms/inbound_binary_msg", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/comms/inbound_binary_msg"]),  
                    ("can/engine_speed", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/can/engine_speed"]),
                    ("can/speed", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/can/speed"]),
                    ("can/acc_engaged", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/can/acc_engaged"])        
                ],
                parameters=[ carma_param_file
                              ]
        )
    ])

    return LaunchDescription([
       declare_port,
       ui_container
    ])



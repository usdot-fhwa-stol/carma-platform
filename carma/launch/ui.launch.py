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
        name='perception_points_filter_container',
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
                    ("get_available_routes", "$(optenv CARMA_GUIDE_NS)/get_available_routes"),
                    ("set_active_route", "$(optenv CARMA_GUIDE_NS)/set_active_route"), 
                    ("start_active_route", "$(optenv CARMA_GUIDE_NS)/start_active_route"),
                    ("get_available_routes", "$(optenv CARMA_GUIDE_NS)/get_available_routes"),
                    ("route_state", "$(optenv CARMA_GUIDE_NS)/route_state"), 
                    ("route_event", "$(optenv CARMA_GUIDE_NS)/route_event"),
                    ("route", "$(optenv CARMA_GUIDE_NS)/route"),
                    ("get_system_version", "$(optenv CARMA_GUIDE_NS)/get_system_version"), 
                    ("state", "$(optenv CARMA_GUIDE_NS)/state"),  
                    ("ui_platoon_vehicle_info", "$(optenv CARMA_GUIDE_NS)/ui_platoon_vehicle_info"), 
                    ("plugins/available_plugins", "$(optenv CARMA_GUIDE_NS)/plugins/available_plugins"),
                    ("plugins/get_registered_plugins", "$(optenv CARMA_GUIDE_NS)/plugins/get_registered_plugins"), 
                    ("plugins/activate_plugin", "$(optenv CARMA_GUIDE_NS)/plugins/activate_plugin"),
                    ("get_available_routes", "$(optenv CARMA_GUIDE_NS)/get_available_routes"),
                    ("set_guidance_active", "$(optenv CARMA_GUIDE_NS)/set_guidance_active"), 
                    ("plugins/controlling_plugins", "$(optenv CARMA_GUIDE_NS)/plugins/controlling_plugins"),
                    ("traffic_signal_info", "$(optenv CARMA_GUIDE_NS)/traffic_signal_info"),
                    ("platooning_info", "$(optenv CARMA_GUIDE_NS)/platooning_info"), 
                    ("traffic_signal_info", "$(optenv CARMA_GUIDE_NS)/traffic_signal_info"),  
                    ("system_alert", "/system_alert"),  
                    ("bsm", "$(optenv CARMA_MSG_NS)/incoming_bsm"),
                    ("nav_sat_fix", "$(optenv CARMA_INTR_NS)/gnss/nav_sat_fix"), 
                    ("velocity", "$(optenv CARMA_INTR_NS)/vehicle/twist"),
                    ("driver_discovery", "$(optenv CARMA_INTR_NS)/driver_discovery"),
                    ("get_drivers_with_capabilities", "$(optenv CARMA_INTR_NS)/get_drivers_with_capabilities"), 
                    ("controller/robotic_status", "$(optenv CARMA_INTR_NS)/controller/robot_status"),
                    ("controller/vehicle_cmd" , "$(optenv CARMA_INTR_NS)/controller/vehicle_cmd"),
                    ("comms/outbound_binary_msg", "$(optenv CARMA_INTR_NS)/comms/outbound_binary_msg"), 
                    ("comms/inbound_binary_msg", "$(optenv CARMA_INTR_NS)/comms/inbound_binary_msg"),  
                    ("can/engine_speed", "$(optenv CARMA_INTR_NS)/can/engine_speed"),
                    ("can/speed", "$(optenv CARMA_INTR_NS)/can/speed"),
                    ("can/acc_engaged", "$(optenv CARMA_INTR_NS)/can/acc_engaged")         
                ],
                parameters=[ carma_param_file
                              ]
            )

    return LaunchDescription([
       declare_port,
       ui_container
    ])



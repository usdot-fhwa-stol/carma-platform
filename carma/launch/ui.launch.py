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
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.actions import GroupAction
from launch_ros.actions.set_remap import SetRemap

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

    rosbridge_server_group = GroupAction(
        actions=[
            SetRemap("get_available_routes",[EnvironmentVariable('CARMA_GUIDE_NS', default_value=''),"/get_available_routes"]),
            SetRemap("set_active_route",[EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/set_active_route"]), 
            SetRemap("start_active_route",[EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/start_active_route"]),
            SetRemap("get_available_routes", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/get_available_routes"]),
            SetRemap("route_state", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route_state"]), 
            SetRemap("route_event", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route_event"]),
            SetRemap("route", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/route"]),
            SetRemap("get_system_version", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/get_system_version"]), 
            SetRemap("state", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/state"]),  
            SetRemap("ui_platoon_vehicle_info", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/ui_platoon_vehicle_info"]), 
            SetRemap("plugins/available_plugins", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/available_plugins"]),
            SetRemap("plugins/get_registered_plugins", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/get_registered_plugins"]), 
            SetRemap("plugins/activate_plugin", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/activate_plugin"]),
            SetRemap("get_available_routes", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/get_available_routes"]),
            SetRemap("set_guidance_active", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/set_guidance_active"]), 
            SetRemap("plugins/controlling_plugins", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/plugins/controlling_plugins"]),
            SetRemap("traffic_signal_info", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/traffic_signal_info"]),
            SetRemap("platooning_info", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/platooning_info"]), 
            SetRemap("traffic_signal_info", [EnvironmentVariable('CARMA_GUIDE_NS', default_value=''), "/traffic_signal_info"]),  
            SetRemap("system_alert", "/system_alert"),         
            SetRemap("bsm", [EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_bsm"]),
            SetRemap("nav_sat_fix", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/gnss/nav_sat_fix"]), 
            SetRemap("velocity", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/vehicle/twist"]),
            SetRemap("driver_discovery", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/driver_discovery"]),
            SetRemap("get_drivers_with_capabilities", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/get_drivers_with_capabilities"]), 
            SetRemap("controller/robotic_status", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/controller/robot_status"]),
            SetRemap("controller/vehicle_cmd" , [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/controller/vehicle_cmd"]),
            SetRemap("comms/outbound_binary_msg",[EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/comms/outbound_binary_msg"]), 
            SetRemap("comms/inbound_binary_msg", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/comms/inbound_binary_msg"]),  
            SetRemap("can/engine_speed", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/can/engine_speed"]),
            SetRemap("can/speed", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/can/speed"]),
            SetRemap("can/acc_engaged", [EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/can/acc_engaged"]),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    [ get_package_share_directory('rosbridge_server'), '/launch/rosbridge_websocket_launch.xml']
                ),
                launch_arguments = { 
                    'port' : port,
                }.items()
            ),
        ]
    )

    return LaunchDescription([
       declare_port,
       rosbridge_server_group
    ])



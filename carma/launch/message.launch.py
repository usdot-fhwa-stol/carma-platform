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
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import set_remap


def generate_launch_description():
    """
    Launch V2X subsystem nodes.
    """

    subsystem_controller_param_file = os.path.join(
        get_package_share_directory('subsystem_controllers'), 'config/environment_perception_controller_config.yaml')

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    # TODO add nodes

    # subsystem_controller which orchestrates the lifecycle of this subsystem's components
    subsystem_controller = Node(
        package='subsystem_controllers',
        name='v2x_controller',
        executable='v2x_controller',
        parameters=[ subsystem_controller_param_file ],
        on_exit= Shutdown(), # Mark the subsystem controller as required
        arguments=['--ros-args', '--log-level', GetLogLevel('subsystem_controllers', env_log_levels)]
    )

    # Add j2735_convertor node
    j2735_convertor_pkg = get_package_share_directory('j2735_convertor')
    j2735_convertor_group = GroupAction(
        actions = [
            # Launch SSC
            set_remap.SetRemap('outgoing_bsm','bsm_outbound'),
                        
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(['/', j2735_convertor_pkg, '/launch','/carma_speed_steering_control.launch.py']),
            ),
        ]
    )

    return LaunchDescription([
        subsystem_controller,
        j2735_convertor_group
    ])
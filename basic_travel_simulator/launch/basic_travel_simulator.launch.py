# Copyright (C) 2024 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction, ExecuteProcess
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os


"""
This file is can be used to launch the CARMA basic_travel_simulator node.
  Though in carma-platform it may be launched directly from the base launch file.
"""


def generate_launch_description():

    # Get parameter file path
    basic_travel_simulator_param_file = os.path.join(
        get_package_share_directory("basic_travel_simulator"), "config/parameters.yaml"
    )

    basic_travel_simulator_container = ComposableNodeContainer(
        package="carma_ros2_utils",
        name="basic_travel_simulator_container",
        executable="carma_component_container_mt",
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package="basic_travel_simulator",
                plugin="basic_travel_simulator::Node",
                name="basic_travel_simulator",
                extra_arguments=[
                    {"use_intra_process_comms": True},
                    {"--log-level": "WARN"},
                ],
                remappings=[
                    (
                        "vehicle/twist",
                        "/hardware_interface/vehicle/twist",
                    ),
                    (
                        "current_pose",
                        "/localization/selected_pose",
                    ),
                    (
                        "plan_trajectory",
                        "/guidance/plan_trajectory",
                    ),
                ],
                parameters=[basic_travel_simulator_param_file],
            )
        ],
    )

    configure_node = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/basic_travel_simulator", "configure"],
                output="screen",
            )
        ],
    )

    activate_node = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/basic_travel_simulator", "activate"],
                output="screen",
            )
        ],
    )

    return LaunchDescription([basic_travel_simulator_container, configure_node, activate_node])

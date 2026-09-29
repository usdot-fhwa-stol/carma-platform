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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os

'''
This file is can be used to launch the CARMA Pure Pursuit Wrapper Node.
  Though in carma-platform it may be launched directly from the base launch file.
'''


def generate_launch_description():

    carma_pure_pursuit_wrapper_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_pure_pursuit_wrapper_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='pure_pursuit_wrapper',
                plugin='pure_pursuit_wrapper::PurePursuitWrapperNode',
                name='pure_pursuit_wrapper',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                ]
            )
        ]
    )

    return LaunchDescription([
        carma_pure_pursuit_wrapper_container
    ])

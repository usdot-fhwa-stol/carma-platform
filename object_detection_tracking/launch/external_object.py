# Copyright (C) 2021 LEIDOS.
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

import os

'''
This file is can be used to launch the CARMA object_detection_tracking_node.
  Though in carma-platform is may be loaded as a component instead of a separate node
'''

def generate_launch_description():


    param_file_path = os.path.join(
        get_package_share_directory('object_detection_tracking'), 'config/parameters.yaml')

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='external_object_container',
        namespace='/',
        executable='component_container_mt',
        composable_node_descriptions=[
 
            ComposableNode(
                    package='object_detection_tracking',
                    plugin='object::ObjectDetectionTrackingNode',
                    name='external_object',
                    namespace="/",
                    extra_arguments=[{'use_intra_process_comms': True}],
                    parameters=[ param_file_path ]
            ),
        ]
    )

    return LaunchDescription([
        container
    ])

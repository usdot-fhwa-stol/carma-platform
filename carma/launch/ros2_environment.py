# Copyright 2021 the Autoware Foundation
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os


def generate_launch_description():
    """
    Launch perception nodes.

     * euclidean_cluster
     * ray_ground_classifier
    """
    autoware_auto_launch_pkg_prefix = get_package_share_directory(
        'autoware_auto_launch')

    euclidean_cluster_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/component_style/euclidean_cluster.param.yaml')

    ray_ground_classifier_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/component_style/ray_ground_classifier.param.yaml')

    # Nodes
    perception_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='perception_points_filter_container',
        executable='lifecycle_component_wrapper_st',
        namespace="/environment",
        composable_node_descriptions=[
            ComposableNode(
                package='ray_ground_classifier_nodes',
                name='ray_ground_filter',
                plugin='autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode',
                namespace="/environment",
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ("points_in", "/hardware_interface/lidar/points_raw"),
                    ("points_nonground", "points_no_ground")
                ],
                parameters=[ ray_ground_classifier_param_file]
            ),
            ComposableNode(
                package='euclidean_cluster_nodes',
                name='euclidean_cluster',
                plugin='autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode',
                namespace="/environment",
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ("points_in", "points_no_ground"),
                ],
                parameters=[ euclidean_cluster_param_file ]
            )
        ]
    )

    return LaunchDescription([
        perception_container
    ])

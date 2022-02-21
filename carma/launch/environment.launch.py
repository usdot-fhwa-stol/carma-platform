# Copyright (C) 2021 LEIDOS.
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


def generate_launch_description():
    """
    Launch perception nodes.
    """

    autoware_auto_launch_pkg_prefix = get_package_share_directory(
        'autoware_auto_launch')

    euclidean_cluster_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/component_style/euclidean_cluster.param.yaml')

    ray_ground_classifier_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/component_style/ray_ground_classifier.param.yaml')
    
    tracking_nodes_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/component_style/tracking_nodes.param.yaml')

    object_detection_tracking_param_file = os.path.join(
        get_package_share_directory('object_detection_tracking'), 'config/parameters.yaml')

    subsystem_controller_param_file = os.path.join(
        get_package_share_directory('subsystem_controllers'), 'config/environment_perception_controller_config.yaml')

    frame_transformer_param_file = os.path.join(
        get_package_share_directory('frame_transformer'), 'config/parameters.yaml')

    object_visualizer_param_file = os.path.join(
        get_package_share_directory('object_visualizer'), 'config/parameters.yaml')

    points_map_filter_param_file = os.path.join(
        get_package_share_directory('points_map_filter'), 'config/parameters.yaml')
    
    motion_computation_param_file = os.path.join(
        get_package_share_directory('motion_computation'), 'config/parameters.yaml')

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    # lidar_perception_container contains all nodes for lidar based object perception
    # a failure in any one node in the chain would invalidate the rest of it, so they can all be 
    # placed in the same container without reducing fault tolerance
    # a lifecycle wrapper container is used to ensure autoware.auto nodes adhere to the subsystem_controller's signals
    lidar_perception_container = ComposableNodeContainer(
        package='carma_ros2_utils', # rclcpp_components
        name='perception_points_filter_container',
        executable='lifecycle_component_wrapper_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='frame_transformer',
                plugin='frame_transformer::Node',
                name='lidar_to_map_frame_transformer',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('frame_transformer', env_log_levels) },
                    {'is_lifecycle_node': True} # Flag to allow lifecycle node loading in lifecycle wrapper
                ],
                remappings=[
                    ("input", [ EnvironmentVariable('CARMA_INTR_NS', default_value=''), "/lidar/points_raw" ] ),
                    ("output", "points_in_map"),
                    ("change_state", "disabled_change_state"), # Disable lifecycle topics since this is a lifecycle wrapper container
                    ("get_state", "disabled_get_state")        # Disable lifecycle topics since this is a lifecycle wrapper container  
                ],
                parameters=[ 
                    { "target_frame" : "map"},
                    { "message_type" : "sensor_msgs/PointCloud2"},
                    { "queue_size" : 1},
                    { "timeout" : 50 },
                ]
            ),
            ComposableNode(
                package='points_map_filter',
                plugin='points_map_filter::Node',
                name='points_map_filter',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('points_map_filter', env_log_levels) },
                    {'is_lifecycle_node': True} # Flag to allow lifecycle node loading in lifecycle wrapper
                ],
                remappings=[
                    ("points_raw", "points_in_map" ),
                    ("filtered_points", "map_filtered_points"),
                    ("lanelet2_map", "semantic_map"),
                    ("change_state", "disabled_change_state"), # Disable lifecycle topics since this is a lifecycle wrapper container
                    ("get_state", "disabled_get_state")        # Disable lifecycle topics since this is a lifecycle wrapper container  
                ],
                parameters=[ points_map_filter_param_file ]
            ),
            ComposableNode(
                package='frame_transformer',
                plugin='frame_transformer::Node',
                name='lidar_frame_transformer',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('frame_transformer', env_log_levels) },
                    {'is_lifecycle_node': True} # Flag to allow lifecycle node loading in lifecycle wrapper
                ],
                remappings=[
                    ("input", "map_filtered_points" ),
                    ("output", "points_in_base_link"),
                    ("change_state", "disabled_change_state"), # Disable lifecycle topics since this is a lifecycle wrapper container
                    ("get_state", "disabled_get_state")        # Disable lifecycle topics since this is a lifecycle wrapper container  
                ],
                parameters=[ frame_transformer_param_file ]
            ),
            ComposableNode(
                package='ray_ground_classifier_nodes',
                name='ray_ground_filter',
                plugin='autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('ray_ground_classifier_nodes', env_log_levels) }
                ],
                remappings=[
                    ("points_in", "points_in_base_link"), 
                    ("points_nonground", "points_no_ground")
                ],
                parameters=[ ray_ground_classifier_param_file]
            ),
            ComposableNode(
                package='euclidean_cluster_nodes',
                name='euclidean_cluster',
                plugin='autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('euclidean_cluster_nodes', env_log_levels) }
                ],
                remappings=[
                    ("points_in", "points_no_ground")
                ],
                parameters=[ euclidean_cluster_param_file ]
            ),
            ComposableNode(
                package='object_detection_tracking',
                plugin='bounding_box_to_detected_object::Node',
                name='bounding_box_converter',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('object_detection_tracking', env_log_levels) },
                    {'is_lifecycle_node': True} # Flag to allow lifecycle node loading in lifecycle wrapper
                ],
                remappings=[
                    ("bounding_boxes", "lidar_bounding_boxes"),
                    ("lidar_detected_objects", "detected_objects"),
                ]
            ),
            ComposableNode(
                    package='tracking_nodes',
                    plugin='autoware::tracking_nodes::MultiObjectTrackerNode',
                    name='tracking_nodes_node',
                    extra_arguments=[
                        {'use_intra_process_comms': True}, 
                        {'--log-level' : GetLogLevel('tracking_nodes', env_log_levels) }
                    ],
                    remappings=[
                        ("ego_state", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/current_pose_with_covariance" ] ),
                        # TODO note classified_rois1 is the default single camera input topic 
                        # TODO when camera detection is added, we will wan to separate this node into a different component to preserve fault tolerance 
                    ],
                    parameters=[ tracking_nodes_param_file ]
            ),
 
        ]
    )

    # carma_external_objects_container contains nodes for object detection and tracking
    # since these nodes can use different object inputs they are a separate container from the lidar_perception_container
    # to preserve fault tolerance
    carma_external_objects_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='external_objects_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
 
            ComposableNode(
                    package='object_detection_tracking',
                    plugin='object::ObjectDetectionTrackingNode',
                    name='external_object',
                    extra_arguments=[
                        {'use_intra_process_comms': True}, 
                        {'--log-level' : GetLogLevel('object_detection_tracking', env_log_levels) }
                    ],
                    remappings=[
                        ("detected_objects", "tracked_objects"),
                    ],
                    parameters=[ object_detection_tracking_param_file ]
            ),
            ComposableNode(
                    package='object_visualizer',
                    plugin='object_visualizer::Node',
                    name='object_visualizer_node',
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level' : GetLogLevel('object_visualizer', env_log_levels) }
                    ],
                    parameters=[ object_visualizer_param_file ]
            ),
            ComposableNode(
                package='motion_computation',
                plugin='motion_computation::MotionComputationNode',
                name='motion_computation_node',
                extra_arguments=[
                    {'use_intra_process_comms': True}, 
                    {'--log-level' : GetLogLevel('motion_computation', env_log_levels) }
                ],
                remappings=[
                    ("incoming_mobility_path", [ EnvironmentVariable('CARMA_MSG_NS', default_value=''), "/incoming_mobility_path" ] ),
                    ("georeference", [ EnvironmentVariable('CARMA_LOCZ_NS', default_value=''), "/map_param_loader/georeference" ] )
                ],
                parameters=[ 
                    motion_computation_param_file,
                ]
            )
        ]
    )

    # subsystem_controller which orchestrates the lifecycle of this subsystem's components
    subsystem_controller = Node(
        package='subsystem_controllers',
        name='environment_perception_controller',
        executable='environment_perception_controller',
        parameters=[ subsystem_controller_param_file ],
        on_exit= Shutdown(), # Mark the subsystem controller as required
        arguments=['--ros-args', '--log-level', GetLogLevel('subsystem_controllers', env_log_levels)]
    )

    return LaunchDescription([
        lidar_perception_container,
        carma_external_objects_container,
        subsystem_controller
    ])

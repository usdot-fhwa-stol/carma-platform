
# Copyright (C) 2020-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.5)
project(object_detection_tracking)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(worker_lib object_detection_tracking_worker_lib)
set(node_exec object_detection_tracking_node)
set(node_lib object_detection_tracking_node_lib)
set(bounding_box_lib bounding_box_node_lib)
set(bounding_box_exec bounding_box_node_exec)

# Includes
include_directories(
  include
)

# Build
ament_auto_add_library(${worker_lib}
        src/object_detection_tracking_worker.cpp
)

ament_auto_add_library(${node_lib} SHARED
        src/object_detection_tracking_node.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

ament_auto_add_library(${bounding_box_lib} SHARED
        src/bounding_box_to_detected_object/bounding_box_to_detected_object_node.cpp
)

ament_auto_add_executable(${bounding_box_exec} 
        src/bounding_box_to_detected_object/main.cpp
)

rclcpp_components_register_nodes(${node_lib} "object::ObjectDetectionTrackingNode")
rclcpp_components_register_nodes(${bounding_box_lib} "bounding_box_to_detected_object::Node")

target_link_libraries(${node_lib}
        ${worker_lib}
)

target_link_libraries(${node_exec}
        ${node_lib}
)

target_link_libraries(${bounding_box_exec}
        ${bounding_box_lib}
)

# Install
ament_auto_package(
        INSTALL_TO_SHARE config launch
)

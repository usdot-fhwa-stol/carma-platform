#
# Copyright (C) 2023 LEIDOS.
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
#

cmake_minimum_required(VERSION 3.5)
project(intersection_transit_maneuvering)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec intersection_transit_maneuvering_exec)
set(node_lib intersection_transit_maneuvering_lib)

## System dependencies are found with CMake's conventions
find_package(Boost)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/intersection_transit_maneuvering.cpp
        src/itm_service.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "intersection_transit_maneuvering::IntersectionTransitManeuveringNode")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_exec}
        ${node_lib}
)

# Testing
if(BUILD_TESTING)  

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

ament_add_gtest(test_intersection_transit_maneuvering test/test_intersection_transit_maneuvering.cpp test/call_test.cpp test/test_main.cpp)
ament_target_dependencies(test_intersection_transit_maneuvering ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
target_link_libraries(test_intersection_transit_maneuvering ${node_lib})

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE launch
)

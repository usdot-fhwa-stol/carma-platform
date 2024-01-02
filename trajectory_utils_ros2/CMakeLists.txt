
# Copyright (C) 2022 LEIDOS.
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
project(trajectory_utils_ros2)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_lib trajectory_utils_ros2_lib)

# Includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/trajectory_utils/quintic_coefficient_calculator.cpp
        src/trajectory_utils/trajectory_utils.cpp
        src/trajectory_utils/conversions/conversions.cpp
)

target_link_libraries(
        ${node_lib}
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_trajectory_utils_ros2 
    test/quintic_coefficient_calculator_test.cpp
    test/test_conversions.cpp
    test/test_trajectory_utils.cpp
  )

  ament_target_dependencies(test_trajectory_utils_ros2 ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_trajectory_utils_ros2 ${node_lib})

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE
)
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
project(localization_manager)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec localization_manager_node_exec)
set(node_lib localization_manager_node)

# Includes
include_directories(
  include
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/LocalizationTypes.cpp
        src/LocalizationTransitionTable.cpp
        src/LocalizationManager.cpp
        src/localization_manager_node.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "localization_manager::Node")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_exec}
        ${node_lib}
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_localization_manager 
  test/TestLocalizationManager.cpp
  test/TestLocalizationTransitionTable.cpp
  test/TestLocalizationTypes.cpp
  test/TestMain.cpp

  )

  ament_target_dependencies(test_localization_manager ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_localization_manager ${node_lib})

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE config launch
)
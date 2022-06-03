
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
project(carma_wm_ctrl)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Name build targets
set(node_lib carma_wm_ctrl_node_lib)
set(worker_lib carma_wm_ctrl_worker_lib)
set(node_exec carma_wm_ctrl_node_exec)

# Build

ament_auto_add_library(${worker_lib} SHARED
        src/WMBroadcaster.cpp
        src/GeofenceScheduler.cpp
        src/GeofenceSchedule.cpp
)

ament_auto_add_library(${node_lib} SHARED
        src/WMBroadcasterNode.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "carma_wm_ctrl::WMBroadcasterNode")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_lib}
        ${worker_lib}
)

target_link_libraries(${node_exec}
        ${node_lib}
)


# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(${PROJECT_NAME}-test 
        test/TestMain.cpp
        test/GeofenceSchedulerTest.cpp
        test/GeofenceScheduleTest.cpp
        test/WMBroadcasterTest.cpp
        test/MapToolsTest.cpp
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
  )

  ament_target_dependencies(${PROJECT_NAME}-test ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(${PROJECT_NAME}-test ${node_lib})
  
endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE config launch
)

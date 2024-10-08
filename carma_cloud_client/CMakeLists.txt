
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
project(carma_cloud_client)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(CURL REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(v2xhubWebAPI REQUIRED)
find_package(ZLIB)

# Name build targets
set(node_exec carma_cloud_client_node_exec)
set(node_lib carma_cloud_client_node)

# Includes
include_directories(
  include
  ${CURL_INCLUDE_DIR}
)

include_directories(${Qt5Widgets_INCLUDE_DIRS}) 

include_directories(${EXTERNAL_INSTALL_LOCATION}/include)
link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}
    qhttpengine
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/carma_cloud_client_node.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "carma_cloud_client::CarmaCloudClient")
target_link_libraries(${node_lib}
        ${CURL_LIBRARIES}
        ${QHttpEngine_LIBRARY}  
        Qt5Widgets Qt5Core Qt5Network qhttpengine v2xhubWebAPI ZLIB::ZLIB
)

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_exec}
        ${node_lib}
        ${CURL_LIBRARIES}
        ${QHttpEngine_LIBRARY}  
        Qt5Widgets Qt5Core Qt5Network qhttpengine v2xhubWebAPI ZLIB::ZLIB
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_carma_cloud_client test/node_test.cpp)

  ament_target_dependencies(test_carma_cloud_client ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_carma_cloud_client ${node_lib} ${CURL_LIBRARIES} ${QHttpEngine_LIBRARY} curl)

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE config launch
)

#
# Copyright (C) 2018-2020 LEIDOS.
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

cmake_minimum_required(VERSION 2.8.3)
project(autoware_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cav_msgs
  cav_srvs
  carma_utils
  autoware_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cav_srvs carma_utils cav_msgs autoware_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

file(GLOB_RECURSE headers */*.hpp */*.h)

add_executable( ${PROJECT_NAME}
  ${headers}
  src/autoware_plugin.cpp
  src/main.cpp)
add_library(autoware_plugin_library src/autoware_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(autoware_plugin_library ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install C++
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
catkin_add_gmock(${PROJECT_NAME}-test test/test_autoware_plugin.cpp)
target_link_libraries(${PROJECT_NAME}-test autoware_plugin_library ${catkin_LIBRARIES})

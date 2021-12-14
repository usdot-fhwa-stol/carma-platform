#
# Copyright (C) 2018-2021 LEIDOS.
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

cmake_minimum_required(VERSION 3.5.1)
project(mobilitypath_publisher)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cav_msgs
  carma_utils
  lanelet2_extension
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cav_msgs carma_utils lanelet2_extension
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable( ${PROJECT_NAME}
  src/mobilitypath_publisher.cpp
  src/main.cpp)

target_link_libraries(${PROJECT_NAME} mobilitypath_publisher_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(mobilitypath_publisher_lib src/mobilitypath_publisher.cpp)

target_link_libraries(mobilitypath_publisher_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(mobilitypath_publisher_lib ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
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
catkin_add_gmock(${PROJECT_NAME}-test
  test/test_mobility_path_publisher.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(${PROJECT_NAME}-test mobilitypath_publisher_lib ${catkin_LIBRARIES})


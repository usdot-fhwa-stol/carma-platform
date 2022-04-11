cmake_minimum_required(VERSION 2.8.3)

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

project(gnss_to_map_convertor)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  wgs84_utils
  cav_msgs
  cav_srvs
  message_filters
  carma_utils
  lanelet2_extension
  lanelet2_core
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gnss_to_map_convertor
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2_msgs wgs84_utils message_filters carma_utils tf2_geometry_msgs lanelet2_extension
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/GNSSToMapConvertor.cpp
)

## Add cmake target dependencies of the library
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable

add_executable(${PROJECT_NAME}_node src/gnss_to_map_convertor/main.cpp src/${PROJECT_NAME}/GNSSToMapConvertor.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/${PROJECT_NAME}/GNSSToMapConvertorTest.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

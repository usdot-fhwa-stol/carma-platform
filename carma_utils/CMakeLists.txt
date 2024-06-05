
# Copyright (C) 2019-2021 LEIDOS.
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

cmake_minimum_required(VERSION 2.8.3)
project(carma_utils)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(ROS_DEPS
  roscpp
  cav_msgs
  cav_srvs
  message_filters
)
find_package(catkin REQUIRED COMPONENTS
  ${ROS_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${ROS_DEPS}
  DEPENDS Boost
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
  src/${PROJECT_NAME}/CARMANodeHandle.cpp
  src/${PROJECT_NAME}/timers/ROSTimer.cpp
  src/${PROJECT_NAME}/timers/ROSTimerFactory.cpp
  src/${PROJECT_NAME}/timers/testing/TestTimer.cpp
  src/${PROJECT_NAME}/timers/testing/TestTimerFactory.cpp
)
add_dependencies( ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add gtest based cpp test target and link libraries
  add_rostest_gtest(${PROJECT_NAME}_CARMANodeHandle_test test/${PROJECT_NAME}/CARMANodeHandle.test
    test/${PROJECT_NAME}/CARMANodeHandleTest.cpp
    test/${PROJECT_NAME}/timers/TestingTimers.cpp
    test/${PROJECT_NAME}/containers/containers_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_CARMANodeHandle_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_dependencies( ${PROJECT_NAME}_CARMANodeHandle_test ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()


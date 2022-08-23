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

cmake_minimum_required(VERSION 3.0.2)
project(carla_sensors_integration)

set(CATKIN_DEPS
  roscpp
  rospy
  std_msgs
  cav_msgs
  carma_utils
  sensor_msgs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
 ${CATKIN_DEPS}
)


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
  CATKIN_DEPENDS ${CATKIN_DEPS}
)


## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME}_library ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/carla_sensors_integration_worker.cpp
  src/carla_sensors_integration_node.cpp
)
## Declare a C++ library
add_library(carla_sensors_integration_library
  src/main.cpp
  src/carla_sensors_integration_worker.cpp
  src/carla_sensors_integration_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} carla_sensors_integration_library ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} carla_sensors_integration_library ${catkin_EXPORTED_TARGETS})

target_link_libraries(carla_sensors_integration_library ${catkin_LIBRARIES})
add_dependencies(carla_sensors_integration_library ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME} carla_sensors_integration_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 ## Install Other Resources
install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS ${PROJECT_NAME}_library
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)

# Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY
#  launch
#  config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############


## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_carla_sensors.cpp
  test/test_main.cpp)
if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test carla_sensors_integration_library)
endif()


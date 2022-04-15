
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

cmake_minimum_required(VERSION 3.0.2)
project(carma_wm)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Set catkin dependencies
set ( PKG_CATKIN_DEPS 
  autoware_lanelet2_msgs
  lanelet2_core
  lanelet2_maps
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_extension
  carma_utils
  cav_msgs
  carma_debug_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  autoware_lanelet2_ros_interface
)

## Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${PKG_CATKIN_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

## Catkin export configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PKG_CATKIN_DEPS}
  DEPENDS Boost EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare C++ library
add_library(
  ${PROJECT_NAME}
  src/MapConformer.cpp
  src/CARMAWorldModel.cpp
  src/WMListener.cpp
  src/WMListenerWorker.cpp
  src/Geometry.cpp
  src/WorldModelUtils.cpp
  src/TrafficControl.cpp
  src/IndexedDistanceMap.cpp
  src/collision_detection.cpp
  src/SignalizedIntersectionManager.cpp
)

## Add cmake target dependencies of the library
add_dependencies(
  ${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(
  map_update_logger_node
  src/MapUpdateLoggerNode.cpp
)

target_link_libraries(
  map_update_logger_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(
  map_update_logger_node
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} #map_update_logger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gmock(${PROJECT_NAME}-test
  test/GeometryTest.cpp
  test/TestMain.cpp
  test/CARMAWorldModelTest.cpp
  test/WMListenerWorkerTest.cpp
  test/SignalizedIntersectionManagerTest.cpp
  test/CollisionDetectionTest.cpp
  test/IndexedDistanceMapTest.cpp
  test/MapConformerTest.cpp
  test/TrafficControlTest.cpp
  test/WMTestLibForGuidanceTest.cpp
  test/WorldModelUtilsTest.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
)

catkin_add_gmock(segfault
 test/TestMain.cpp
 test/SegFaultTest.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
)
if(TARGET segfault)
 target_link_libraries(segfault ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

cmake_minimum_required(VERSION 2.8.3)
project(carma_wm)

## Compile as C++14, as it is required by lanelet2
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

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
  roscpp
)

## Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${PKG_CATKIN_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
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
)

## Declare C++ library
add_library(${PROJECT_NAME}
  src/CARMAWorldModel.cpp
  src/WMListener.cpp
  src/WMListenerWorker.cpp
  src/lanelet/CarmaUSTrafficRules.cpp
  src/lanelet/PassingControlLine.cpp
  src/lanelet/DigitalSpeedLimit.cpp
  src/lanelet/RegionAccessRule.cpp
  src/lanelet/DirectionOfTravel.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

catkin_add_gmock(${PROJECT_NAME}-test
  test/TestMain.cpp
  test/CARMAWorldModleTest.cpp  
  test/IndexedDistanceMapTest.cpp
  test/WMListenerWorkerTest.cpp
  test/lanelet/RegionAccessRuleTest.cpp
  test/lanelet/PassingControlLineTest.cpp
  test/lanelet/DirectionOfTravelTest.cpp
  test/lanelet/DigitalSpeedLimitTest.cpp
  test/lanelet/CarmaUSTrafficRulesTest.cpp
  test/lanelet/MapLoadingTest.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

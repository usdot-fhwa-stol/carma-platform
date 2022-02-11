cmake_minimum_required(VERSION 3.0.2)
project(lci_strategic_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(PKG_CATKIN_DEPS
  roscpp
  cav_srvs
  cav_msgs
  carma_utils
  carma_wm
  lanelet2_core
  lanelet2_traffic_rules
)

find_package(catkin REQUIRED COMPONENTS
  ${PKG_CATKIN_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)


catkin_package(
  CATKIN_DEPENDS ${PKG_CATKIN_DEPS}
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

# Build Lib
add_library(${PROJECT_NAME}
  src/lci_strategic_plugin_algo.cpp 
  src/lci_strategic_plugin.cpp
  src/lci_states.cpp
  src/lci_state_transition_table.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Build Node Executable
add_executable( ${PROJECT_NAME}_node
  src/main.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install C++
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
 test/test_main.cpp
 test/test_fixture.h
 test/test_strategic_plugin_helpers.cpp
 test/test_transition_table.cpp
 test/test_strategic_plugin.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})

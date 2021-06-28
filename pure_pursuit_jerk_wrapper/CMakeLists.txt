cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit_jerk_wrapper)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
add_definitions(-DBOOST_LOG_DYN_LINK=1)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-lboost_system") 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set( CATKIN_DEPS 
  cav_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  autoware_msgs
  autoware_config_msgs
  message_filters
  carma_utils
  trajectory_utils
  carma_wm
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


add_library(${PROJECT_NAME} src/pure_pursuit_jerk_wrapper.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${CATKIN_DEPS}
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
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library

add_executable(${PROJECT_NAME}_node src/pure_pursuit_jerk_wrapper_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}_test test/${PROJECT_NAME}_test.cpp)

target_link_libraries( ${PROJECT_NAME}_test ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )


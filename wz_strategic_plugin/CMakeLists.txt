cmake_minimum_required(VERSION 3.0.2)
project(wz_strategic_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  carma_utils
  carma_wm
  cav_msgs
  cav_srvs
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs cav_srvs carma_utils cav_msgs # carma_wm
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
  src/${PROJECT_NAME}.cpp
  src/main.cpp)

#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_library(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wz_strategic_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

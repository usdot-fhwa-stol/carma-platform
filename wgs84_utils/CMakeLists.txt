cmake_minimum_required(VERSION 2.8.3)
project(wgs84_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wgs84_utils_library
  CATKIN_DEPENDS roscpp tf2 tf2_ros tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers */*.hpp */*.h)

add_library(wgs84_utils_library src/wgs84_utils/wgs84_utils.cpp)
target_link_libraries(wgs84_utils_library)
add_dependencies(wgs84_utils_library ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS wgs84_utils_library
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

##########
## Test ##
##########

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# catkin_add_gtest(transform_maintainer_test test/transform_maintainer_test.cpp)

# target_link_libraries( transform_maintainer_test
#         wgs84_utils_library
#         ${Boost_LIBRARIES} 
#         ${catkin_LIBRARIES}
#         )
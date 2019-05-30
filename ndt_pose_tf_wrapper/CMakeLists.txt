cmake_minimum_required(VERSION 2.8.3)
project(ndt_pose_tf_wrapper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros tf2_geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( ${PROJECT_NAME} src/ndt_wrapper_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.hpp"
PATTERN ".svn" EXCLUDE
)

## Install C++
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

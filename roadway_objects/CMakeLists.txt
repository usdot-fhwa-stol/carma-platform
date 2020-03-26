cmake_minimum_required(VERSION 2.8.3)
project(roadway_objects)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(DEPS 
  cav_msgs
  roscpp
  carma_utils
  carma_wm
  lanelet2_core
  lanelet2_routing
  lanelet2_traffic_rules
  tf2
  tf2_geometry_msgs
)
find_package(catkin REQUIRED COMPONENTS
  ${DEPS}
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
  CATKIN_DEPENDS ${DEPS}
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


add_executable(roadway_objects_node src/main.cpp src/RoadwayObjectsNode.cpp)

add_library(roadway_objects_worker src/RoadwayObjectsWorker.cpp)
add_dependencies(roadway_objects_worker ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(roadway_objects_node roadway_objects_worker ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS roadway_objects_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gmock(${PROJECT_NAME}-test test/TestRoadwayObjectsWorker.cpp test/TestMain.cpp)
target_link_libraries(${PROJECT_NAME}-test roadway_objects_worker ${catkin_LIBRARIES})

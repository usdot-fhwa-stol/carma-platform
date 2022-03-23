cmake_minimum_required(VERSION 3.0.2)
project(traffic_incident_parser)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  carma_utils
  cav_msgs
  roscpp
  lanelet2_extension
  carma_wm 
  lanelet2_core
  autoware_lanelet2_ros_interface
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
#  LIBRARIES traffic_incident_parser
  CATKIN_DEPENDS carma_utils cav_msgs roscpp lanelet2_extension carma_wm lanelet2_core autoware_lanelet2_ros_interface
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
  ${Boost_INCLUDE_DIRS}
)

add_executable(traffic_incident_parser_node src/main.cpp src/traffic_incident_parser_node.cpp)

add_library(traffic_incident_parser_worker_lib src/traffic_incident_parser_worker.cpp)
add_dependencies(traffic_incident_parser_worker_lib ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(traffic_incident_parser_worker_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(traffic_incident_parser_node traffic_incident_parser_worker_lib)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS traffic_incident_parser_node traffic_incident_parser_worker_lib
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
catkin_add_gmock(${PROJECT_NAME}-test 
  test/main.cpp 
  test/test_traffic_parser.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
)
target_link_libraries(${PROJECT_NAME}-test traffic_incident_parser_worker_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

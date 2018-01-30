cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

#options
option(BUILD_TEST_INTERFACE_MANAGER "Should the test interface manager be built" OFF)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  cav_msgs
  cav_srvs
  bondcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
	cfg/SensorFusion.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_fusion
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs cav_msgs cav_srvs bondcpp tf2 tf2_ros tf2_geometry_msgs dynamic_reconfigure
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

add_executable(${PROJECT_NAME}_node ${headers} src/main.cpp src/sensor_fusion.cpp src/wgs84_utils.cpp)
add_dependencies( ${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

add_executable(so_to_marker src/so_to_marker.cpp)
add_dependencies(so_to_marker ${catkin_EXPORTED_TARGETS})
target_link_libraries(so_to_marker ${catkin_LIBRARIES})


if(${BUILD_TEST_INTERFACE_MANAGER})
 message("Building test interface manager")
 add_executable(interface_mgr_node src/interface_mgr.cpp)
 add_dependencies(interface_mgr_node ${catkin_EXPORTED_TARGETS})
 target_link_libraries(interface_mgr_node
         ${catkin_LIBRARIES}
         )
endif(${BUILD_TEST_INTERFACE_MANAGER})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}_node
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


install(DIRECTORY
        cfg
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

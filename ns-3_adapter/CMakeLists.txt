cmake_minimum_required(VERSION 2.8.3)
project(ns-3_adapter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ##cav_driver_utils
  cav_msgs
  cav_srvs
  roscpp
  bondcpp
  dynamic_reconfigure
)

add_executable( ${PROJECT_NAME}
  ${headers}
  src/ns-3_client.cpp
  src/ns-3_adapter.cpp
  src/driver_application/driver_application.cpp
  src/driver_wrapper/driver_wrapper.cpp
  src/main.cpp)
add_library(ns-3_adapter_library src/ns-3_adapter.cpp src/ns-3_client.cpp src/main.cpp src/driver_application/driver_application.cpp src/driver_wrapper/driver_wrapper.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(ns-3_adapter_library ${catkin_EXPORTED_TARGETS})

catkin_package(
  CATKIN_DEPENDS roscpp dynamic_reconfigure bondcpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 thirdparty/rapidjson/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers */*.hpp */*.h)


## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node  ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#install(TARGETS ${PROJECT_NAME}_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY 
#  thirdparty
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY
#  etc
#  launch
#  config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
test/test_ns-3_adapter.cpp
test/test_main.cpp)
target_link_libraries(${PROJECT_NAME}-test ns-3_adapter_library ${catkin_LIBRARIES})
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)





cmake_minimum_required(VERSION 2.8.3)
project(ns3_adapter)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

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
  src/ns3_client.cpp
  src/ns3_adapter.cpp
  src/driver_application/driver_application.cpp
  src/driver_wrapper/driver_wrapper.cpp
  src/main.cpp)
add_library(ns3_adapter_library src/ns3_adapter.cpp src/ns3_client.cpp src/main.cpp src/driver_application/driver_application.cpp src/driver_wrapper/driver_wrapper.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(ns3_adapter_library ${catkin_EXPORTED_TARGETS})

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

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install C++
install(TARGETS ${PROJECT_NAME} ns3_adapter_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test 
test/test_ns3_adapter.cpp
test/test_main.cpp)
target_link_libraries(${PROJECT_NAME}-test ns3_adapter_library ${catkin_LIBRARIES})
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)





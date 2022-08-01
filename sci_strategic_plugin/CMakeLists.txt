cmake_minimum_required(VERSION 3.0.2)
project(sci_strategic_plugin)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)
carma_package()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(PKG_CATKIN_DEPS
  roscpp
  cav_srvs
  cav_msgs
  carma_utils
  bsm_helper
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
  src/sci_strategic_plugin.cpp
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

install(DIRECTORY include
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
 test/test_strategic_plugin.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_sci_strategic_plugin test/sci_strategic_plugin.test test/test_sci_strategic_plugin.cpp)
  target_link_libraries(test_sci_strategic_plugin ${catkin_LIBRARIES})
endif()
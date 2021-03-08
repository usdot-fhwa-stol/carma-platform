cmake_minimum_required(VERSION 2.8.3)
project(stanley_controller)

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  amathutils_lib
  autoware_health_checker
  autoware_msgs
  geometry_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  amathutils_lib
)

SET(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(
  STANLEY_CONTROLLER_SRC
    src/mpc_utils.cpp
    src/mpc_trajectory.cpp
)

add_executable(stanley_controller src/stanley_controller_node.cpp src/stanley_controller_core.cpp ${STANLEY_CONTROLLER_SRC})
add_dependencies(stanley_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stanley_controller ${catkin_LIBRARIES})

install(
  TARGETS stanley_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

cmake_minimum_required(VERSION 3.0.2)
project(rosbag_mock_drivers)

## Compile as C++14
add_compile_options(-std=c++14)


####################
## Dependancies   ##
####################
set ( PKG_CATKIN_DEPENDS 
  roscpp
  std_msgs
  carma_utils
  cav_msgs
  cav_srvs
  rosbag
  cav_simulation_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${PKG_CATKIN_DEPENDS}
)

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
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${PKG_CATKIN_DEPENDS}
 DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/MockDriverNode.cpp
  src/MockCameraDriver.cpp
  src/MockCANDriver.cpp
  src/MockCommsDriver.cpp
  src/MockControllerDriver.cpp
  src/MockGNSSDriver.cpp
  src/MockIMUDriver.cpp
  src/MockLidarDriver.cpp
  src/MockRadarDriver.cpp
  src/MockRoadwaySensorDriver.cpp
  src/BagParser.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(mock_driver
  src/main.cpp
)

add_executable(bag_parser
  src/parser_main.cpp
  src/BagParser.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(mock_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(bag_parser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mock_driver
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(bag_parser
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS mock_driver bag_parser
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS mock_driver ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gmock(${PROJECT_NAME}-test
  test/TestMain.cpp
  test/MockCameraDriverTest.cpp
  test/MockCANDriverTest.cpp
  test/MockCommsDriverTest.cpp
  test/MockControllerDriverTest.cpp
  test/MockGNSSDriverTest.cpp
  test/MockIMUDriverTest.cpp
  test/MockLidarDriverTest.cpp
  test/MockRadarDriverTest.cpp
  test/MockRoadwaySensorDriverTest.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)

catkin_add_gmock(bag_parser-test
  test/TestMain.cpp
  test/BagParserTest.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

if(TARGET bag_parser-test)
  target_link_libraries(bag_parser-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
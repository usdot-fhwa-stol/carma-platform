cmake_minimum_required(VERSION 2.8.3)
project(cav_driver_utils)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  bondcpp
  cav_msgs
  cav_srvs
  can_msgs
  roscpp
  std_msgs
)

find_package(Boost REQUIRED system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES driver_application cav_socketcan_interface ros_socketcan_interface
  CATKIN_DEPENDS bondcpp cav_msgs cav_srvs roscpp std_msgs
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
)

## Declare a C++ library
add_library(driver_application
    src/driver_application/driver_application.cpp
    include/driver_application/driver_application.h
)

add_dependencies(driver_application ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
 target_link_libraries(driver_application
   ${catkin_LIBRARIES}
 )


add_library(cav_socketcan_interface
        src/socketcan_interface/socketcan_interface.cpp
        include/cav_driver_utils/can/socketcan_interface/socketcan_interface.h
        )


add_dependencies(cav_socketcan_interface ${catkin_EXPORTED_TARGETS})

target_link_libraries(cav_socketcan_interface
        ${Boost_LIBRARIES})

add_library(ros_socketcan_interface
        src/ros_socketcan_bridge/ros_socketcan_bridge.cpp
        include/cav_driver_utils/can/ros_socketcan_bridge/ros_socketcan_bridge.h
        )

add_dependencies(ros_socketcan_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_socketcan_interface
        ${Boost_LIBRARIES})

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

## Mark executables and/or libraries for installation
 install(TARGETS driver_application cav_socketcan_interface
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/driver_application/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/cav_driver_utils/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cav_driver_utils.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 2.8.3)
project(openjaus)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Threads REQUIRED)

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
 LIBRARIES jaus openjaus
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/jaus
  include/openjaus
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE JAUS_SRC_FILES "${PROJECT_SOURCE_DIR}/src/libjaus" *.c)

## Declare a C++ library
add_library(jaus
    ${JAUS_SRC_FILES}
)
target_include_directories(jaus PUBLIC include/jaus)

file(GLOB_RECURSE OPENJAUS_SRC_FILES "${PROJECT_SOURCE_DIR}/src/libopenjaus" *.cpp *.c)
add_library(openjaus
    ${OPENJAUS_SRC_FILES}
)
target_include_directories(openjaus PUBLIC include/openjaus)
target_link_libraries(openjaus jaus)

option(BUILD_OPENJAUS_NM OFF)
if(BUILD_OPENJAUS_NM)
find_package(Curses REQUIRED)
include_directories(
  ${CURSES_INCLUDE_DIR}
)
add_executable(ojNodeManager src/ojNodeManager/main.cpp)
target_link_libraries(ojNodeManager jaus openjaus ${CURSES_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} )
endif()


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
 install(TARGETS jaus openjaus
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_OpenJaus33.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

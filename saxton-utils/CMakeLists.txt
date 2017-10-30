cmake_minimum_required(VERSION 2.8.3)
project(saxton-utils)

################
## POST BUILD ##
################

message(STATUS "The copy target path is: " ${CATKIN_DEVEL_PREFIX}/lib)

file(COPY saxton-utils-1.0-SNAPSHOT.jar DESTINATION ${CATKIN_DEVEL_PREFIX}/lib)


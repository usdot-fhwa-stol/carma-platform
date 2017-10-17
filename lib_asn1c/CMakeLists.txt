cmake_minimum_required(VERSION 2.8.3)
project(asn1c)

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
	#INCLUDE_DIRS
	#LIBRARIES
	##CATKIN_DEPENDS
	##DEPENDS
)

################
## POST BUILD ##
################
set(LIBRARY_PATH_STRING $ENV{LD_LIBRARY_PATH})
string(REPLACE ":" ";" LIBRARY_PATH_LIST ${LIBRARY_PATH_STRING})
list(GET LIBRARY_PATH_LIST 0 COPY_TARGET)

file(COPY third_party_lib/libasn1c.so DESTINATION ${COPY_TARGET})

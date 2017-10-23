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
message(STATUS "The copy target path is: " ${CATKIN_DEVEL_PREFIX}/lib)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	file(COPY third_party_lib/libasn1c_x64.so DESTINATION ${CATKIN_DEVEL_PREFIX}/lib)
	file(RENAME ${CATKIN_DEVEL_PREFIX}/lib/libasn1c_x64.so ${CATKIN_DEVEL_PREFIX}/lib/libasn1c.so)
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
	file(COPY third_party_lib/libasn1c_x86.so DESTINATION ${CATKIN_DEVEL_PREFIX}/lib)
	file(RENAME ${CATKIN_DEVEL_PREFIX}/lib/libasn1c_x86.so ${CATKIN_DEVEL_PREFIX}/lib/libasn1c.so)
endif()



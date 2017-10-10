cmake_minimum_required(VERSION 2.8.3)
project(asn1c)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also nee

catkin_package(
	INCLUDE_DIRS include
	##LIBRARIES
	##CATKIN_DEPENDS
	##DEPENDS
)

###########
## Build ##
###########

##Specify additional locations of header files
include_directories(
	include
	/usr/lib/jvm/default-java/include
	/usr/lib/jvm/default-java/include/linux
)

##Declare the C library
add_library(
	asn1c
	src/wrapper.c
)

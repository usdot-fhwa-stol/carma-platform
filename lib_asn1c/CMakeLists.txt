cmake_minimum_required(VERSION 2.8.3)
project(asn1c)

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



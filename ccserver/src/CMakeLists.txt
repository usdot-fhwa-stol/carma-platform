cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
find_package(qhttpengine REQUIRED)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

OPTION(NODEBUG  "Deactivate No debugging option" "OFF")

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC -Wall -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++14 -Wall -Wno-unused-variable")

if(${NODEBUG} STREQUAL "OFF")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -pg -g3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -g3")
else (${NODEBUG} STREQUAL "OFF")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -s -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s -O3")
endif(${NODEBUG} STREQUAL "OFF")

find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
include(GNUInstallDirs)
file(GLOB SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/models/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/handlers/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/requests/*.cpp
   # ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)
add_library (${PROJECT_NAME} ${SRCS})

target_link_libraries(${PROJECT_NAME} Qt5Core Qt5Network ssl crypto qhttpengine)

target_include_directories( ${PROJECT_NAME} PUBLIC ${Qt5Core_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/models> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/handlers>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/requests>
    $<INSTALL_INTERFACE:include> PRIVATE ${SRCS} )

#set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)

install(TARGETS ${PROJECT_NAME} EXPORT v2xhubWebAPIConfig ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY handlers/ models/ requests/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

install(EXPORT v2xhubWebAPIConfig DESTINATION share/v2xhubWebAPI/cmake)

export(TARGETS ${PROJECT_NAME} FILE v2xhubWebAPIConfig.cmake)
                                                              

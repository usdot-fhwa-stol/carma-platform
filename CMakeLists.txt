cmake_minimum_required(VERSION 3.10)
# based on https://cliutils.gitlab.io/modern-cmake/

# define project
project(ODRParser
        VERSION 1.0.0
        DESCRIPTION "A very simple OpenDRIVE parser"
        LANGUAGES CXX)

# c++ version
set(CMAKE_CXX_STANDARD 14)

# add ./cmake to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# unset testing for libs
option(BUILD_TESTS "Sets or unsets the option to generate the test target" OFF)
option(BUILD_GTEST "Sets or unsets the option to build gtest manually (do not set if gtest is installed)" OFF)

# add source directory
add_subdirectory(src)

## add tinyxml2
set(BUILD_TESTING off CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS off CACHE BOOL "" FORCE)

# load tinyxml2
add_subdirectory(lib/tinyxml2 EXCLUDE_FROM_ALL)

### GTEST
if ((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME) AND BUILD_TESTS)

    # enable testing and load library
    enable_testing()

    # switch (build or use installed lib)
    if (BUILD_GTEST)

        # add subdirectory
        add_subdirectory(googletest EXCLUDE_FROM_ALL)
        set(GTEST_BOTH_LIBRARIES gtest_main)
        include_directories(./googletest/googletest/include)

    else()

        # find gtest
        find_package(GTest REQUIRED)
        include_directories(${GTEST_INCLUDE_DIRS})

    endif()

    # load macros for test
    include(AddGoogleTest)

    # add test sources
    add_subdirectory(tests)

endif ()

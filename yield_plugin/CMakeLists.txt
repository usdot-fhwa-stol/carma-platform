#
# Copyright (C) 2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#

cmake_minimum_required(VERSION 3.8)
project(yield_plugin)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec yield_plugin_exec)
set(worker_lib yield_plugin_lib)
set(node_lib yield_plugin_node)

## System dependencies are found with CMake's conventions
find_package(Boost)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Build libraries first (BEFORE SIMD configuration)
ament_auto_add_library(${node_lib} SHARED
        src/yield_plugin.cpp
        src/yield_plugin_node.cpp
)

ament_auto_add_executable(${node_exec}
        src/main.cpp
)

# SIMD support configuration (AFTER targets are created)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
    # Check for AVX2 support
    include(CheckCXXSourceCompiles)

    # Save original flags
    set(CMAKE_REQUIRED_FLAGS_BACKUP "${CMAKE_REQUIRED_FLAGS}")
    set(CMAKE_REQUIRED_FLAGS "-mavx2")

    check_cxx_source_compiles("
        #include <immintrin.h>
        int main() {
            __m256 a = _mm256_set1_ps(1.0f);
            return 0;
        }" HAVE_AVX2)

    # Restore original flags
    set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS_BACKUP}")

    if(HAVE_AVX2)
        target_compile_options(${node_lib} PRIVATE -mavx2 -mfma)
        target_compile_definitions(${node_lib} PRIVATE HAVE_AVX2)
        message(STATUS "AVX2 support enabled for ${node_lib}")
    else()
        # Check for SSE4.1 support
        set(CMAKE_REQUIRED_FLAGS "-msse4.1")
        check_cxx_source_compiles("
            #include <smmintrin.h>
            int main() {
                __m128 a = _mm_set1_ps(1.0f);
                return 0;
            }" HAVE_SSE41)

        if(HAVE_SSE41)
            target_compile_options(${node_lib} PRIVATE -msse4.1)
            target_compile_definitions(${node_lib} PRIVATE HAVE_SSE)
            message(STATUS "SSE4.1 support enabled for ${node_lib}")
        else()
            message(STATUS "No SIMD support available")
        endif()
    endif()

    # Also apply SIMD flags to the executable if it contains relevant code
    if(HAVE_AVX2)
        target_compile_options(${node_exec} PRIVATE -mavx2 -mfma)
        target_compile_definitions(${node_exec} PRIVATE HAVE_AVX2)
    elseif(HAVE_SSE41)
        target_compile_options(${node_exec} PRIVATE -msse4.1)
        target_compile_definitions(${node_exec} PRIVATE HAVE_SSE)
    endif()
endif()

# Register component
rclcpp_components_register_nodes(${node_lib} "yield_plugin::YieldPluginNode")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_lib}
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES}
)
target_link_libraries(${node_exec}
        ${node_lib}
)

# Testing
if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_yield_plugin
  test/test_cooperative_yield.cpp
  test/test_yield_plugin.cpp
  test/test_yield.cpp
 )

  ament_target_dependencies(test_yield_plugin ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_yield_plugin ${node_lib})

  # Apply SIMD flags to tests as well if they use SIMD code
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
    if(HAVE_AVX2)
        target_compile_options(test_yield_plugin PRIVATE -mavx2 -mfma)
        target_compile_definitions(test_yield_plugin PRIVATE HAVE_AVX2)
    elseif(HAVE_SSE41)
        target_compile_options(test_yield_plugin PRIVATE -msse4.1)
        target_compile_definitions(test_yield_plugin PRIVATE HAVE_SSE)
    endif()
  endif()

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE config launch
)

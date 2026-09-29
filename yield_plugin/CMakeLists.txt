#
# Copyright (C) 2022-2026 LEIDOS.
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

cmake_minimum_required(VERSION 3.18)
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
## CUDA  ##
###########

enable_language(CUDA)
# nvcc 11.5 is not fully compatible with GCC 11 C++17 headers; C++14 is sufficient
# for the kernel TU which contains no C++17-specific syntax.
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
# sm_75 = Turing (RTX 20xx); sm_80 = A100; sm_86 = Ampere (RTX 30xx)
# sm_89 (Ada/RTX 40xx) is not supported by nvcc 11.7 but JITs from sm_86 PTX at runtime
set(CMAKE_CUDA_ARCHITECTURES "75;80;86")

# Build the CUDA collision kernel as a separate PIC static library so that
# ament_auto_add_library (which produces a shared object) can link it cleanly.
add_library(yield_cuda_lib STATIC src/yield_plugin_cuda.cu)
target_include_directories(yield_cuda_lib PUBLIC include)
set_target_properties(yield_cuda_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/yield_plugin.cpp
        src/yield_plugin_node.cpp
)

ament_auto_add_executable(${node_exec}
        src/main.cpp
)

# Register component
rclcpp_components_register_nodes(${node_lib} "yield_plugin::YieldPluginNode")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_lib}
        yield_cuda_lib
        ${CUDA_LIBRARIES}
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

  # Accuracy + performance benchmark for the CUDA collision rewrite
  ament_add_gtest(test_collision_cuda_benchmark
    test/test_collision_cuda_benchmark.cpp
  )
  ament_target_dependencies(test_collision_cuda_benchmark ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
  target_link_libraries(test_collision_cuda_benchmark ${node_lib})

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE config launch
)

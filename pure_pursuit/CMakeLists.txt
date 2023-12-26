# Copyright 2019 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(pure_pursuit)
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/pure_pursuit/config.hpp
  src/config.cpp
  include/pure_pursuit/pure_pursuit.hpp
  src/pure_pursuit.cpp
)
autoware_set_compile_options(${PROJECT_NAME})
# Setting a system include is needed to avoid having an explicit Eigen dependency.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
if(BUILD_TESTING)
  # run common linters
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(PURE_PURSUIT_GTEST pure_pursuit_gtest)
  ament_add_gtest(${PURE_PURSUIT_GTEST}
    test/gtest_main.cpp
    test/sanity_checks.cpp
    test/test_pure_pursuit.cpp)
  autoware_set_compile_options(${PURE_PURSUIT_GTEST})
  target_include_directories(${PURE_PURSUIT_GTEST} PRIVATE "include")
  target_link_libraries(${PURE_PURSUIT_GTEST} ${PROJECT_NAME} osrf_testing_tools_cpp::memory_tools)
  ament_target_dependencies(${PURE_PURSUIT_GTEST} "osrf_testing_tools_cpp")
endif()

# Ament Exporting
ament_auto_package()

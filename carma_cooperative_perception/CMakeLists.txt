# Copyright 2023 Leidos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(carma_cooperative_perception)

# Added outside of `dependencies.cmake` because ament sets some variables
# (e.g., BUILD_TESTING) that affect the configuration options for the rest of
# the package. Putting the command call here allows us to put all project
# options together in separate CMake module then query those options in
# `dependencies.cmake`.
find_package(ament_cmake_auto REQUIRED)

include(project_options.cmake)
include(dependencies.cmake)

# C17 CMake support added in CMake 3.21
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# This will automatically add include files from include/carma_cooperative_perception
ament_auto_add_library(sdsm_to_detection_list_node SHARED
  src/sdsm_to_detection_list_node
)

target_link_libraries(sdsm_to_detection_list_node
  PROJ::proj
  Microsoft.GSL::GSL
  units::units
)

rclcpp_components_register_nodes(sdsm_to_detection_list_node
  "carma_cooperative_perception::SdsmToDetectionListNode"
)

if(carma_cooperative_perception_BUILD_TESTS)
  enable_testing()

  # We must add tests to top-level CMakeLists.txt because the
  # ament_auto_add_gtest command will look for headers in the include/
  # directory. Moving this command to test/CMakeLists.txt will prevent the
  # header files from getting included.
  ament_auto_add_gtest(cooperative_perception_tests
    test/test_geodetic.cpp
    test/test_j2735_types.cpp
    test/test_j3224_types.cpp
    test/test_month.cpp
    test/test_msg_conversion.cpp
  )
endif()

carma_package()
ament_auto_package()

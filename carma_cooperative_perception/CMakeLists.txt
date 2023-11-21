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

# CARMA builds packages in an environment with both ROS 1 and ROS 2 installed.
# This check gracefully skips the current package if the sourced ROS environment
# is not the specified version. This call must come before any other ROS
# dependencies becasue ROS 1 does not have some of the required packages.
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

# Added outside of `dependencies.cmake` because ament sets some variables
# (e.g., BUILD_TESTING) that affect the configuration options for the rest of
# the package. Putting the command call here allows us to put all project
# options together in separate CMake module then query those options in
# `dependencies.cmake`.
find_package(ament_cmake_auto REQUIRED)

# Needed so CMake can find the vendored PROJ4 module file. Th FindPROJ4.cmake
# module file can be removed if we upgrade to a more recent PROJ version. See
# https://github.com/usdot-fhwa-stol/carma-platform/issues/2139 for updates.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake_options.cmake)
include(dependencies.cmake)

# The generated compilation database is helpful with code completion in IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ${carma_cooperative_perception_EXPORT_COMPILE_COMMANDS})

# This prevents `colcon` from tyring to build the CMake project's binary
# directory. This is useful in case we want to build the project outside of
# `colcon` but still keep it in a ROS repository.
file(TOUCH ${PROJECT_BINARY_DIR}/COLCON_IGNORE)

# Configures CARMA package default settings
carma_package()

# C17 CMake support added in CMake 3.21
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# This will automatically add include files from include/carma_cooperative_perception
ament_auto_add_library(carma_cooperative_perception SHARED
  src/external_object_list_to_detection_list_component.cpp
  src/external_object_list_to_sdsm_component.cpp
  src/geodetic.cpp
  src/j2735_types.cpp
  src/j3224_types.cpp
  src/msg_conversion.cpp
  src/sdsm_to_detection_list_component.cpp
  src/track_list_to_external_object_list_component.cpp
  src/utm_zone.cpp
  src/multiple_object_tracker_component.cpp
  src/host_vehicle_filter_component.cpp
)

target_link_libraries(carma_cooperative_perception
  ${PROJ4_LIBRARIES}  # Note: Newer versions of PROJ use PROJ::proj
  GSL  # Note: Newer versions of GSL use Microsoft.GSL::GSL
  units::units
  cooperative_perception_core::cooperative_perception_coreLibrary
)

rclcpp_components_register_nodes(carma_cooperative_perception
  "carma_cooperative_perception::ExternalObjectListToDetectionListNode"
  "carma_cooperative_perception::ExternalObjectListToSdsmNode"
  "carma_cooperative_perception::SdsmToDetectionListNode"
  "carma_cooperative_perception::TrackListToExternalObjectListNode"
  "carma_cooperative_perception::MultipleObjectTrackerNode"
  "carma_cooperative_perception::HostVehicleFilterNode"
)

ament_auto_add_executable(external_object_list_to_detection_list_node
  src/external_object_list_to_detection_list_node.cpp
)

ament_auto_add_executable(track_list_to_external_object_list_node
  src/track_list_to_external_object_list_node.cpp
)

ament_auto_add_executable(multiple_object_tracker_node
  src/multiple_object_tracker_node.cpp
)

ament_auto_add_executable(external_object_list_to_sdsm_node
  src/external_object_list_to_sdsm_node.cpp
)

ament_auto_add_executable(host_vehicle_filter_node
  src/host_vehicle_filter_node.cpp
)

# boost::posix_time definition for using nanoseconds
add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)

if(carma_cooperative_perception_BUILD_TESTS)
  enable_testing()

  # We must add tests to top-level CMakeLists.txt because the
  # ament_auto_add_gtest command will look for headers in the include/
  # directory. Moving this command to test/CMakeLists.txt will prevent the
  # header files from getting included.
  ament_auto_add_gtest(carma_cooperative_perception_tests
    test/test_external_object_list_to_detection_list_component.cpp
    test/test_geodetic.cpp
    test/test_j2735_types.cpp
    test/test_j3224_types.cpp
    test/test_month.cpp
    test/test_msg_conversion.cpp
  )

  target_link_libraries(carma_cooperative_perception_tests
    carma_cooperative_perception
  )

  add_launch_test(test/track_list_to_external_object_list_launch_test.py)
  add_launch_test(test/multiple_object_tracker_duplicates_launch_test.py)
  add_launch_test(test/multiple_object_tracker_static_obstacle_launch_test.py)
  add_launch_test(test/host_vehicle_filter_launch_test.py)

endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)

include(GNUInstallDirs)

# ament_auto does not provide file pattern matching
install(DIRECTORY ${PROJECT_SOURCE_DIR}/test/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/carma_cooperative_perception/test
  FILES_MATCHING
    PATTERN *.py
    PATTERN *.cpp
    PATTERN __pycache__ EXCLUDE
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/test/data/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/carma_cooperative_perception/test/data
  FILES_MATCHING
    PATTERN *.yaml
)

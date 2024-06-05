# Copyright 2024 Leidos
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

cmake_minimum_required(VERSION 3.5)
project(carma_ros2_utils)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)

include(dependencies.cmake)

carma_package()

# TODO(CAR-6017): Remove when we drop support for Foxy
# rclcpp_lifecycle has breaking API changes starting with Humble
if(("$ENV{ROS_DISTRO}" STREQUAL "foxy") OR ("$ENV{ROS_DISTRO}" STREQUAL "galactic"))
  set(carma_ros2_utils_LIFECYCLE_PUBLISHER_INTERFACE_TYPE
    rclcpp_lifecycle::LifecyclePublisherInterface
  )
else()
  set(carma_ros2_utils_LIFECYCLE_PUBLISHER_INTERFACE_TYPE
    rclcpp_lifecycle::ManagedEntityInterface
  )
endif()

configure_file(
  src/carma_lifecycle_node.hpp.in
  carma_ros2_utils/carma_lifecycle_node.hpp
)

ament_auto_add_library(carma_ros2_utils SHARED
  src/carma_lifecycle_node.cpp
)

target_include_directories(carma_ros2_utils
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
)

include(GNUInstallDirs)

INSTALL(FILES ${PROJECT_BINARY_DIR}/carma_ros2_utils/carma_lifecycle_node.hpp
  DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/carma_ros2_utils
)

ament_auto_add_library(carma_ros2_utils_timers SHARED
  src/timers/testing/TestTimer.cpp
  src/timers/testing/TestTimerFactory.cpp
  src/timers/testing/TestClock.cpp
  src/timers/ROSTimer.cpp
  src/timers/ROSTimerFactory.cpp
)

ament_auto_add_library(lifecycle_component_wrapper SHARED
  src/lifecycle_component_wrapper.cpp
)

ament_auto_add_library(carma_component_manager SHARED
  src/component_manager.cpp
)

ament_auto_add_executable(lifecycle_component_wrapper_st
  src/lifecycle_component_wrapper_st.cpp
)

ament_auto_add_executable(lifecycle_component_wrapper_mt
  src/lifecycle_component_wrapper_mt.cpp
)

ament_auto_add_executable(carma_component_container
  src/component_container.cpp
)

ament_auto_add_executable(carma_component_container_mt
  src/component_container_mt.cpp
)

if(BUILD_TESTING)
  # These CMake commands were added to ament_cmake_auto in ROS 2 Humble. Until
  # CARMA supports ROS 2 Humble, we will use package-local copies.
  include(cmake/ament_auto_find_test_dependencies.cmake)
  include(cmake/ament_auto_add_gtest.cmake)

  ament_auto_add_executable(test_carma_lifecycle_node
    test/test_node.cpp
  )

  ament_auto_add_library(test_minimal_node SHARED
    test/test_minimal_node.cpp
  )

  add_launch_test(
    test/launch_test_lifecycle_wrapper.py
  )

  ament_add_gtest_executable(test_lifecycle_gtest
    test/test_lifecycle_manager.cpp
  )

  ament_target_dependencies(test_lifecycle_gtest
    ros2_lifecycle_manager
  )

  target_link_libraries(test_lifecycle_gtest
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )

  ament_add_test(test_lifecycle
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/launch_lifecycle_test.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    TIMEOUT 30
    ENV
      TEST_EXECUTABLE=$<TARGET_FILE:test_lifecycle_gtest>
  )

  ament_auto_add_gtest(test_carma_lifecycle_gtest
    test/test_carma_lifecycle_exceptions.cpp
    test/test_carma_lifecycle_helpers.cpp
  )

  ament_auto_add_gtest(test_timers
    test/timers/TestingTimers.cpp
  )

  ament_auto_add_gtest(test_containers
    test/containers/containers_test.cpp
  )

  install(
    TARGETS test_carma_lifecycle_node test_minimal_node
    # EXPORT should not be needed for unit tests
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib/carma_ros2_utils
    RUNTIME DESTINATION lib/carma_ros2_utils # In Ament, Executables are installed to lib/${PROJECT_NAME} not bin
    INCLUDES DESTINATION include
  )

  rclcpp_components_register_nodes(test_minimal_node "carma_ros2_utils_testing::MinimalNode")

endif()

ament_python_install_package(carma_ros2_utils)

ament_auto_package()

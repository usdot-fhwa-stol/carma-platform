# Copyright 2020-2023 Leidos
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
project(motion_computation)

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

include(options.cmake)
include(dependencies.cmake)

# The generated compilation database is helpful with code completion in IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ${motion_computation_EXPORT_COMPILE_COMMANDS})

# This prevents `colcon` from tyring to build the CMake project's binary
# directory. This is useful in case we want to build the project outside of
# `colcon` but still keep it in a ROS repository.
file(TOUCH ${PROJECT_BINARY_DIR}/COLCON_IGNORE)

# Configures CARMA package default settings
carma_package()

# C17 CMake support added in CMake 3.21
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# ament_auto will automatically add include files from include/motion_computation
# and link necessary libraries
ament_auto_add_library(motion_computation SHARED
  src/motion_computation_worker.cpp
  src/mobility_path_to_external_object.cpp
  src/psm_to_external_object_convertor.cpp
  src/bsm_to_external_object_convertor.cpp
  src/motion_computation_node.cpp
)

ament_auto_add_executable(motion_computation_node_exec
  src/main.cpp
)

rclcpp_components_register_nodes(motion_computation
  "motion_computation::MotionComputationNode"
)

if(motion_computation_BUILD_TESTS)

  ament_auto_add_gtest(test_motion_computation
    test/MotionComputationTest.cpp
    test/TestMain.cpp
  )

  # boost::posix_time definition for using nanoseconds
  target_compile_definitions(test_motion_computation
    PRIVATE
      -DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG
  )

  add_launch_test(test/motion_computation_external_object_launch_test.py)
  add_launch_test(test/motion_computation_queueing_launch_test.py)

endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)

include(GNUInstallDirs)

# ament_auto does not provide file pattern matching
install(DIRECTORY ${PROJECT_SOURCE_DIR}/test/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/motion_computation/test
  FILES_MATCHING
    PATTERN *.py
    PATTERN *.cpp
    PATTERN __pycache__ EXCLUDE
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/test/data/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/motion_computation/test/data
  FILES_MATCHING
    PATTERN *.yaml
)

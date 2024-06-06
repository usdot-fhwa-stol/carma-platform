# Copyright (C) 2020-2022 LEIDOS.
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

cmake_minimum_required(VERSION 3.5)
project(motion_predict)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# Includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(motion_predict
  src/motion_predict.cpp
  src/predict_ctrv.cpp
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_motion_predict 
    test/main.cpp
    test/test_motion_predict.cpp 
    test/test_predict_ctrv.cpp
  )

  ament_target_dependencies(test_motion_predict ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_motion_predict ${PROJECT_NAME})

endif()

ament_auto_package()
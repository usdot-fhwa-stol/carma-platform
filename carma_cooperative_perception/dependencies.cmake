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

include(cmake/get_cpm.cmake)
set(CPM_USE_LOCAL_PACKAGES ON)

# lint_cmake: -readability/wonkycase
CPMAddPackage(NAME units
  GITHUB_REPOSITORY nholthaus/units
  GIT_TAG v2.3.3
  EXCLUDE_FROM_ALL ON
  SYSTEM ON
  OPTIONS
    "BUILD_TESTS FALSE"
    "BUILD_DOCS FALSE"
)

find_package(multiple_object_tracking REQUIRED)

find_package(PROJ REQUIRED MODULE)

# lint_cmake: -readability/wonkycase
CPMAddPackage(NAME Microsoft.GSL
  GITHUB_REPOSITORY microsoft/GSL
  GIT_TAG v2.1.0  # This is the version shipped with Ubuntu 20.04; newer versions are available
  EXCLUDE_FROM_ALL ON
  SYSTEM ON
  OPTIONS
    "GSL_INSTALL TRUE"
    "GSL_TEST FALSE"
    "CMAKE_CXX_STANDARD 17"
)

# This will pull dependencies from <build_depend>...</build_depend> tags in the
# package.xml file. It saves us from having to manually call find_package(...)
# for each dependency.
ament_auto_find_build_dependencies()

if(carma_cooperative_perception_BUILD_TESTS)
  # These CMake commands were added to ament_cmake_auto in ROS 2 Humble. Until
  # CARMA supports ROS 2 Humble, we will use package-local copies.
  include(cmake/ament_auto_find_test_dependencies.cmake)
  include(cmake/ament_auto_add_gtest.cmake)

  ament_auto_find_test_dependencies()

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify  # Using clang-format instead
    # This test has been temporarily disabled to support Continuous Improvement (CI) processes.
    # Related GitHub Issue: <https://github.com/usdot-fhwa-stol/carma-platform/issues/2335>
    ament_cmake_cppcheck
    ament_cmake_cppcheck
    ament_cmake_cpplint
    ament_cmake_flake8
  )

  set(ament_cmake_clang_format_CONFIG_FILE ${PROJECT_SOURCE_DIR}/.clang-format)

  ament_lint_auto_find_test_dependencies()
endif()

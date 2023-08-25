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

CPMAddPackage(NAME units
  GITHUB_REPOSITORY nholthaus/units
  GIT_TAG v2.3.3
  OPTIONS
    "BUILD_TESTS FALSE"
    "BUILD_DOCS FALSE"
)

CPMAddPackage(NAME PROJ
  GITHUB_REPOSITORY OSGeo/PROJ
  # Version 9.2.1 introduces `unintall` target, which collides with Eigen's
  # `uninstall` target.
  GIT_TAG 9.1.1
  OPTIONS
    "BUILD_APPS FALSE"
    "BUILD_TESTING FALSE"
    "ENABLE_CURL FALSE"
    "ENABLE_TIFF FALSE"
)

CPMAddPackage(NAME Microsoft.GSL
  GITHUB_REPOSITORY microsoft/GSL
  GIT_TAG v4.0.0
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
endif()

# From carma_cmake_common (ament_cmake_auto finds the package)
carma_check_ros_version(2)

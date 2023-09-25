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

# This will pull dependencies from <build_depend>...</build_depend> tags in the
# package.xml file. It saves us from having to manually call find_package(...)
# for each dependency.
ament_auto_find_build_dependencies()

if(motion_computation_BUILD_TESTS)
  # These CMake commands were added to ament_cmake_auto in ROS 2 Humble. Until
  # CARMA supports ROS 2 Humble, we will use package-local copies.
  include(cmake/ament_auto_find_test_dependencies.cmake)
  include(cmake/ament_auto_add_gtest.cmake)

  ament_auto_find_test_dependencies()

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify  # Using clang-format instead
  )

  ament_lint_auto_find_test_dependencies()
endif()

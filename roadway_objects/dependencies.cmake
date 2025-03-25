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

# Turned off as we don't use this package actively anymore. Enable and fix when we use it again.
# https://usdot-carma.atlassian.net/browse/CAR-6091

# if(roadway_objects_BUILD_TESTS)
#   ament_auto_find_test_dependencies()

#   list(APPEND AMENT_LINT_AUTO_EXCLUDE
#     ament_cmake_uncrustify  # Using clang-format instead
#   )

#   set(ament_cmake_clang_format_CONFIG_FILE ${PROJECT_SOURCE_DIR}/.clang-format)

#   ament_lint_auto_find_test_dependencies()
# endif()

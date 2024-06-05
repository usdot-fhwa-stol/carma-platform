# Copyright 2021 Whitley Software Services, LLC
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

#
# Invoke find_package() for all test dependencies.
#
# All found package names are appended to the
# ``${PROJECT_NAME}_FOUND_TEST_DEPENDS`` variables.
#
# @public
#
macro(ament_auto_find_test_dependencies)
  set(_ARGN "${ARGN}")
  if(_ARGN)
    message(FATAL_ERROR "ament_auto_find_test_dependencies() called with "
      "unused arguments: ${_ARGN}")
  endif()

  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  # try to find_package() all test dependencies
  foreach(_dep ${${PROJECT_NAME}_TEST_DEPENDS})
    find_package(${_dep} QUIET)
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_TEST_DEPENDS ${_dep})
    endif()
  endforeach()
endmacro()

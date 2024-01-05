// Copyright 2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef PURE_PURSUIT__VISIBILITY_CONTROL_HPP_
#define PURE_PURSUIT__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(PURE_PURSUIT_BUILDING_DLL) || defined(PURE_PURSUIT_EXPORTS)
    #define PURE_PURSUIT_PUBLIC __declspec(dllexport)
    #define PURE_PURSUIT_LOCAL
  #else  // defined(PURE_PURSUIT_BUILDING_DLL) || defined(PURE_PURSUIT_EXPORTS)
    #define PURE_PURSUIT_PUBLIC __declspec(dllimport)
    #define PURE_PURSUIT_LOCAL
  #endif  // defined(PURE_PURSUIT_BUILDING_DLL) || defined(PURE_PURSUIT_EXPORTS)
#elif defined(__linux__)
  #define PURE_PURSUIT_PUBLIC __attribute__((visibility("default")))
  #define PURE_PURSUIT_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define PURE_PURSUIT_PUBLIC __attribute__((visibility("default")))
  #define PURE_PURSUIT_LOCAL __attribute__((visibility("hidden")))
#else  // defined(LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(WINDOWS)

#endif  // PURE_PURSUIT__VISIBILITY_CONTROL_HPP_

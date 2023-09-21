// Copyright 2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "carma_cooperative_perception/sdsm_to_detection_list_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(carma_cooperative_perception::SdsmToDetectionListNode)  // NOLINT

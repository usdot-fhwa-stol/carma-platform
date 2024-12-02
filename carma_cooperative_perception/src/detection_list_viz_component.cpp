// Copyright 2024 Leidos
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

#include "carma_cooperative_perception/detection_list_viz_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace carma_cooperative_perception
{

DetectionListVizNode::DetectionListVizNode(const rclcpp::NodeOptions & options)
: Node("detection_list_viz_node", options)
{
  detection_list_sub_ =
    create_subscription<carma_cooperative_perception_interfaces::msg::DetectionList>(
      "input/detections_lists", 1,
      [this](carma_cooperative_perception_interfaces::msg::DetectionList::ConstSharedPtr msg_ptr) {
        visualization_msgs::msg::MarkerArray markers;

        for (const auto & detection : msg_ptr->detections) {
          visualization_msgs::msg::Marker marker;

          marker.header = detection.header;
          marker.ns = detection.id;
          marker.type = marker.CUBE;
          marker.action = marker.MODIFY;  // equivalent to ADD if marker does not exist
          marker.pose = detection.pose.pose;
          marker.scale.x = 2.0;
          marker.scale.y = 2.0;
          marker.scale.z = 2.0;

          markers.markers.push_back(marker);
        }

        marker_pub_->publish(markers);
      });

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("output/markers", 1);
}

}  // namespace carma_cooperative_perception

// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(carma_cooperative_perception::DetectionListVizNode)  // NOLINT

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

#ifndef CARMA_COOPERATIVE_PERCEPTION__DETECTION_LIST_VIZ_COMPONENT_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__DETECTION_LIST_VIZ_COMPONENT_HPP_

#include <carma_cooperative_perception_interfaces/msg/detection_list.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace carma_cooperative_perception
{
class DetectionListVizNode : public rclcpp::Node
{
public:
  explicit DetectionListVizNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<carma_cooperative_perception_interfaces::msg::DetectionList>::SharedPtr
    detection_list_sub_{nullptr};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_{nullptr};
};
}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__DETECTION_LIST_VIZ_COMPONENT_HPP_

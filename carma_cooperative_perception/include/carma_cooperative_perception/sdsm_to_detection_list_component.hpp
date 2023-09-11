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

#ifndef CARMA_COOPERATIVE_PERCEPTION__SDSM_TO_DETECTION_LIST_COMPONENT_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__SDSM_TO_DETECTION_LIST_COMPONENT_HPP_

#include <carma_cooperative_perception_interfaces/msg/detection_list.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_v2x_msgs/msg/sensor_data_sharing_message.hpp>
#include <rclcpp/rclcpp.hpp>

#include "carma_cooperative_perception/msg_conversion.hpp"

namespace carma_cooperative_perception
{
class SdsmToDetectionListNode : public carma_ros2_utils::CarmaLifecycleNode
{
  using input_msg_type = carma_v2x_msgs::msg::SensorDataSharingMessage;
  using input_msg_shared_pointer = typename input_msg_type::SharedPtr;
  using output_msg_type = carma_cooperative_perception_interfaces::msg::DetectionList;

public:
  explicit SdsmToDetectionListNode(const rclcpp::NodeOptions & options)
  : CarmaLifecycleNode{options},
    publisher_{create_publisher<output_msg_type>("output/detections", 1)},
    subscription_{create_subscription<input_msg_type>(
      "input/sdsm", 1, [this](input_msg_shared_pointer msg_ptr) { sdsm_msg_callback(*msg_ptr); })}
  {
  }

  auto sdsm_msg_callback(const input_msg_type & msg) const noexcept -> void
  {
    publisher_->publish(to_detection_list_msg(msg));
  }

private:
  rclcpp::Publisher<output_msg_type>::SharedPtr publisher_;
  rclcpp::Subscription<input_msg_type>::SharedPtr subscription_;
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__SDSM_TO_DETECTION_LIST_COMPONENT_HPP_

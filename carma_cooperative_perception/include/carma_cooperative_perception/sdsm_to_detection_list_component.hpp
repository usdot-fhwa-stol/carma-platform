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

#include <string>

#include <carma_cooperative_perception_interfaces/msg/detection_list.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_v2x_msgs/msg/sensor_data_sharing_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>

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
      "input/sdsm", 1, [this](input_msg_shared_pointer msg_ptr) { sdsm_msg_callback(*msg_ptr); })},
    georeference_subscription_{create_subscription<std_msgs::msg::String>(
      "input/georeference", 1,
      [this](std_msgs::msg::String::SharedPtr msg_ptr) { georeference_ = msg_ptr->data; })},
    cdasim_clock_sub_{create_subscription<rosgraph_msgs::msg::Clock>(
      "input/cdasim_clock", 1,
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg_ptr) { cdasim_time_ = msg_ptr->clock; })}
  {
  }

  auto sdsm_msg_callback(const input_msg_type & msg) const -> void
  {
    try {
      RCLCPP_ERROR_STREAM(get_logger(), "Received a sdsm msg");
      auto detection_list_msg{to_detection_list_msg(msg, georeference_)};
      RCLCPP_ERROR_STREAM(get_logger(), "Completed to detection list msg");
      if (cdasim_time_) {
        // When in simulation, ROS time is CARLA time, but SDSMs use CDASim time
        const auto time_delta{now() - cdasim_time_.value()};

        for (auto & detection : detection_list_msg.detections) {
          detection.header.stamp = rclcpp::Time(detection.header.stamp) + time_delta;
        }
      }
      RCLCPP_ERROR_STREAM(get_logger(), "Before publishing sdsm msg");
      publisher_->publish(detection_list_msg);
      RCLCPP_ERROR_STREAM(get_logger(), "After publishing sdsm msg");
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to convert SDSM to detection list: " << e.what());
    }
  }

private:
  rclcpp::Publisher<output_msg_type>::SharedPtr publisher_;
  rclcpp::Subscription<input_msg_type>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr georeference_subscription_;

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr cdasim_clock_sub_;
  std::optional<rclcpp::Time> cdasim_time_{std::nullopt};

  std::string georeference_{""};
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__SDSM_TO_DETECTION_LIST_COMPONENT_HPP_

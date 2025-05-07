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
  auto handle_on_configure(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn override
  {
    // Declare parameters
    declare_parameter("overwrite_covariance", config_.overwrite_covariance);
    declare_parameter("pose_covariance_x", config_.pose_covariance_x);
    declare_parameter("pose_covariance_y", config_.pose_covariance_y);
    declare_parameter("pose_covariance_z", config_.pose_covariance_z);
    declare_parameter("pose_covariance_yaw", config_.pose_covariance_yaw);
    declare_parameter("twist_covariance_x", config_.twist_covariance_x);
    declare_parameter("twist_covariance_z", config_.twist_covariance_z);
    declare_parameter("twist_covariance_yaw", config_.twist_covariance_yaw);

    // Get parameters
    config_.overwrite_covariance = get_parameter("overwrite_covariance").as_bool();
    config_.pose_covariance_x = get_parameter("pose_covariance_x").as_double();
    config_.pose_covariance_y = get_parameter("pose_covariance_y").as_double();
    config_.pose_covariance_z = get_parameter("pose_covariance_z").as_double();
    config_.pose_covariance_yaw = get_parameter("pose_covariance_yaw").as_double();
    config_.twist_covariance_x = get_parameter("twist_covariance_x").as_double();
    config_.twist_covariance_z = get_parameter("twist_covariance_z").as_double();
    config_.twist_covariance_yaw = get_parameter("twist_covariance_yaw").as_double();

    // Set up parameter validation callback
    on_set_parameters_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return update_parameters(parameters);
      });

    // Use the stream operator for debug output
    RCLCPP_INFO_STREAM(get_logger(), "SdsmToDetectionListNode Configuration: " << config_);

    return carma_ros2_utils::CallbackReturn::SUCCESS;
  }

  explicit SdsmToDetectionListNode(const rclcpp::NodeOptions & options)
  : CarmaLifecycleNode{options},
    publisher_{create_publisher<output_msg_type>("output/detections", 1)},
    subscription_{create_subscription<input_msg_type>(
      "input/sdsm", 100, [this](input_msg_shared_pointer msg_ptr) { sdsm_msg_callback(*msg_ptr); })},
    georeference_subscription_{create_subscription<std_msgs::msg::String>(
      "input/georeference", 1,
      [this](std_msgs::msg::String::SharedPtr msg_ptr) { georeference_ = msg_ptr->data; })},
    cdasim_clock_sub_{create_subscription<rosgraph_msgs::msg::Clock>(
      "input/cdasim_clock", 1,
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg_ptr) { cdasim_time_ = msg_ptr->clock; })}
  {
    // Initialize the valid parameter names to check when setting the at runtime
    valid_parameter_names_ = {
      "overwrite_covariance",
      "pose_covariance_x",
      "pose_covariance_y",
      "pose_covariance_z",
      "pose_covariance_yaw",
      "twist_covariance_x",
      "twist_covariance_z",
      "twist_covariance_yaw"
    };
  }

  auto sdsm_msg_callback(const input_msg_type & msg) const -> void
  {
    try {
      std::optional<SdsmToDetectionListConfig> covariance_to_overwrite = std::nullopt;
      if (config_.overwrite_covariance) {
        covariance_to_overwrite = config_;
      }
      auto detection_list_msg{
        to_detection_list_msg(msg, georeference_, cdasim_time_ != std::nullopt,
          covariance_to_overwrite)
      };

      // hardcode for now as we are replaying the SDSM
      // for (auto & detection : detection_list_msg.detections) {
      //   detection.header.stamp = now();
      // }
      if (cdasim_time_) {
        // When in simulation, ROS time is CARLA time, but SDSMs use CDASim time
        const auto time_delta{now() - cdasim_time_.value()};

        for (auto & detection : detection_list_msg.detections) {
          detection.header.stamp = rclcpp::Time(detection.header.stamp) + time_delta;
          RCLCPP_ERROR_STREAM(
            get_logger(), "SDSM to detection list conversion: "
            << rclcpp::Time(detection.header.stamp).seconds() << " s");
        }
      }
      RCLCPP_ERROR_STREAM(
        get_logger(), "SDSM to detection list conversion: "
        << detection_list_msg.detections.size() << " detections");
      publisher_->publish(detection_list_msg);
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

  // Configuration object
  SdsmToDetectionListConfig config_{};

  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_{nullptr};

  // Parameter validation function
  rcl_interfaces::msg::SetParametersResult update_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto & parameter : parameters) {
      // Check if parameter name is valid
      if (valid_parameter_names_.find(parameter.get_name()) == valid_parameter_names_.end()) {
        result.successful = false;
        result.reason = "Unexpected parameter name '" + parameter.get_name() + "'";
        RCLCPP_ERROR_STREAM(get_logger(), "Cannot change parameter: " << result.reason);
        break;
      }

      // Validate covariance values (must be non-negative)
      if (parameter.get_name().find("covariance") != std::string::npos &&
          parameter.get_name() != "overwrite_covariance") {
        if (const auto value{parameter.as_double()}; value < 0.0) {
          result.successful = false;
          result.reason = "Covariance parameter must be non-negative";
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "Cannot change parameter '" << parameter.get_name() << "': " << result.reason);
          break;
        } else {
          // Update the appropriate parameter in the config
          update_config_parameter(parameter);
        }
      } else if (parameter.get_name() == "overwrite_covariance") {
        config_.overwrite_covariance = parameter.as_bool();
      }
    }

    // If parameters were successfully updated, log the new configuration
    if (result.successful) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Updated configuration: " << config_);
    }

    return result;
  }

  // Helper function to update a specific parameter in the config
  void update_config_parameter(const rclcpp::Parameter & parameter) {
    const std::string & name = parameter.get_name();
    const double value = parameter.as_double();

    if (name == "pose_covariance_x") {
      config_.pose_covariance_x = value;
    } else if (name == "pose_covariance_y") {
      config_.pose_covariance_y = value;
    } else if (name == "pose_covariance_z") {
      config_.pose_covariance_z = value;
    } else if (name == "pose_covariance_yaw") {
      config_.pose_covariance_yaw = value;
    } else if (name == "twist_covariance_x") {
      config_.twist_covariance_x = value;
    } else if (name == "twist_covariance_z") {
      config_.twist_covariance_z = value;
    } else if (name == "twist_covariance_yaw") {
      config_.twist_covariance_yaw = value;
    }
  }

  // Set of valid parameter names for validation
  std::unordered_set<std::string> valid_parameter_names_;
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__SDSM_TO_DETECTION_LIST_COMPONENT_HPP_

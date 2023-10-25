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

#include "carma_cooperative_perception/host_vehicle_filter_component.hpp"

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace carma_cooperative_perception
{

static auto pose_mahalanobis_distance(
  const carma_cooperative_perception_interfaces::Detection & a,
  const carma_cooperative_perception_interfaces::Detection & b) noexcept -> double
{
  return 0;
}

HostVehicleFilterNode::HostVehicleFilterNode(const rclcpp::NodeOptions & options)
: CarmaLifecycleNode{options}
{
  lifecycle_publishers_.push_back(track_list_pub_);
}

auto handle_on_configure(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: configuring");

  detection_list_sub_ = create_subscription<
    carma_cooperative_perception_interfaces::msg::DetectionList>(
    "input/detection_list", 1,
    [this](const carma_cooperative_perception_interfaces::msg::DetectionList::SharedPtr msg_ptr) {
      if (const auto current_state{this->get_current_state().label()}; current_state == "active") {
        // ...
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->detection_list_sub_->get_topic_name(), current_state.c_str());
      }
    });

  host_vehicle_pose_sub_ =
    create_subscription<>("input/host_vehicle_pose", 1, [this](const auto & msg_ptr) {
      if (const auto current_state{this->get_current_state().label()}; current_state == "active") {
        // ...
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->detection_list_sub_->get_topic_name(), current_state.c_str());
      }
    });

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully configured");

  return carma_ros2_utils::CallbackReturn::Success;
}

auto handle_on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: actiavting");
  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully activated");

  return carma_ros2_utils::CallbackReturn::Success;
}

auto handle_on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: deactivating");
  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully deactivated");

  return carma_ros2_utils::CallbackReturn::Success;
}

auto handle_on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: cleaning up");

  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  detection_list_sub_.reset();
  host_vehicle_pose_sub_.reset();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully cleaned up");

  return carma_ros2_utils::CallbackReturn::Success;
}

auto handle_on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: shutting down");

  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  detection_list_sub_.reset();
  host_vehicle_pose_sub_.reset();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully shut down");

  return carma_ros2_utils::CallbackReturn::Success;
}

}  // namespace carma_cooperative_perception

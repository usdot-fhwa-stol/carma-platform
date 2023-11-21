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

#include <carma_cooperative_perception_interfaces/msg/detection.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <vector>

namespace carma_cooperative_perception
{
auto HostVehicleFilterNode::handle_on_configure(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: configuring");

  detection_list_sub_ = create_subscription<
    carma_cooperative_perception_interfaces::msg::DetectionList>(
    "input/detection_list", 1,
    [this](const carma_cooperative_perception_interfaces::msg::DetectionList::SharedPtr msg_ptr) {
      if (const auto current_state{this->get_current_state().label()}; current_state == "active") {
        attempt_filter_and_republish(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->detection_list_sub_->get_topic_name(), current_state.c_str());
      }
    });

  host_vehicle_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/host_vehicle_pose", 1, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg_ptr) {
      if (const auto current_state{this->get_current_state().label()}; current_state == "active") {
        update_host_vehicle_pose(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->detection_list_sub_->get_topic_name(), current_state.c_str());
      }
    });

  detection_list_pub_ =
    create_publisher<carma_cooperative_perception_interfaces::msg::DetectionList>(
      "output/detection_list", 1);

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully configured");

  declare_parameter("distance_threshold_meters", 0.0);

  on_set_parameters_callback_ =
    add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";

      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "distance_threshold_meters") {
          if (this->get_current_state().label() == "active") {
            result.successful = false;
            result.reason = "parameter is read-only while node is in 'Active' state";

            RCLCPP_ERROR(
              get_logger(), "Cannot change parameter 'distance_threshold_meters': " + result.reason);

            break;
          }

          if (const auto value{parameter.as_double()}; value < 0) {
            result.successful = false;
            result.reason = "parameter must be nonnegative";

            RCLCPP_ERROR(
              get_logger(), "Cannot change parameter 'distance_threshold_meters': " + result.reason);

            break;
          } else {
            this->squared_distance_threshold_meters_ = std::pow(value, 2);
          }
        } else {
          result.successful = false;
          result.reason = "Unexpected parameter name '" + parameter.get_name() + '\'';
          break;
        }
      }

      return result;
    });

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto HostVehicleFilterNode::handle_on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: activating");
  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully activated");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto HostVehicleFilterNode::handle_on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: deactivating");
  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully deactivated");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto HostVehicleFilterNode::handle_on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: cleaning up");

  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  detection_list_sub_.reset();
  host_vehicle_pose_sub_.reset();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully cleaned up");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto HostVehicleFilterNode::handle_on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: shutting down");

  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  detection_list_sub_.reset();
  host_vehicle_pose_sub_.reset();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully shut down");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto HostVehicleFilterNode::update_host_vehicle_pose(
  const geometry_msgs::msg::PoseStamped & msg) noexcept -> void
{
  host_vehicle_pose_ = msg;
}

auto HostVehicleFilterNode::attempt_filter_and_republish(
  carma_cooperative_perception_interfaces::msg::DetectionList msg) noexcept -> void
{
  if (!host_vehicle_pose_.has_value()) {
    RCLCPP_WARN(get_logger(), "Could not filter detection list: host vehicle pose unknown");
    return;
  }

  const auto is_within_distance = [this](const auto & detection) {
    return euclidean_distance_squared(host_vehicle_pose_.value().pose, detection.pose.pose) <=
           this->squared_distance_threshold_meters_;
  };

  const auto new_end{
    std::remove_if(std::begin(msg.detections), std::end(msg.detections), is_within_distance)};

  msg.detections.erase(new_end, std::end(msg.detections));

  this->detection_list_pub_->publish(msg);
}

auto euclidean_distance_squared(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) noexcept -> double
{
  return std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2) +
         std::pow(a.position.z - b.position.z, 2) + std::pow(a.orientation.x - b.orientation.x, 2) +
         std::pow(a.orientation.y - b.orientation.y, 2) +
         std::pow(a.orientation.z - b.orientation.z, 2) +
         std::pow(a.orientation.w - b.orientation.w, 2);
}

}  // namespace carma_cooperative_perception


// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(                               // NOLINT
  carma_cooperative_perception::HostVehicleFilterNode)  // NOLINT
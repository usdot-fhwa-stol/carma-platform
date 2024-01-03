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

#ifndef CARMA_COOPERATIVE_PERCEPTION__HOST_VEHICLE_FILTER_COMPONENT_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__HOST_VEHICLE_FILTER_COMPONENT_HPP_

#include <carma_cooperative_perception_interfaces/msg/detection_list.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace carma_cooperative_perception
{

class HostVehicleFilterNode : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  using carma_ros2_utils::CarmaLifecycleNode::CarmaLifecycleNode;

  auto handle_on_configure(const rclcpp_lifecycle::State & /* previous_state */)
    -> carma_ros2_utils::CallbackReturn override;

  auto handle_on_activate(const rclcpp_lifecycle::State & /* previous_state */)
    -> carma_ros2_utils::CallbackReturn override;

  auto handle_on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
    -> carma_ros2_utils::CallbackReturn override;

  auto handle_on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
    -> carma_ros2_utils::CallbackReturn override;

  auto handle_on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
    -> carma_ros2_utils::CallbackReturn override;

  auto update_host_vehicle_pose(const geometry_msgs::msg::PoseStamped & msg) noexcept -> void;

  auto attempt_filter_and_republish(
    carma_cooperative_perception_interfaces::msg::DetectionList msg) noexcept -> void;

private:
  rclcpp::Subscription<carma_cooperative_perception_interfaces::msg::DetectionList>::SharedPtr
    detection_list_sub_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr host_vehicle_pose_sub_{nullptr};

  rclcpp_lifecycle::LifecyclePublisher<
    carma_cooperative_perception_interfaces::msg::DetectionList>::SharedPtr detection_list_pub_{
    nullptr};

  std::optional<geometry_msgs::msg::PoseStamped> host_vehicle_pose_{std::nullopt};

  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_{nullptr};

  double squared_distance_threshold_meters_{0.0};
};

auto euclidean_distance_squared(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b) noexcept -> double;

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__HOST_VEHICLE_FILTER_COMPONENT_HPP_

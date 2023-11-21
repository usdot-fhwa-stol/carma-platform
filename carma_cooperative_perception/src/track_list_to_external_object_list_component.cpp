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

#include "carma_cooperative_perception/track_list_to_external_object_list_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <utility>
#include <vector>

#include "carma_cooperative_perception/external_object_list_to_detection_list_component.hpp"
#include "carma_cooperative_perception/geodetic.hpp"
#include "carma_cooperative_perception/units_extensions.hpp"

namespace carma_cooperative_perception
{
TrackListToExternalObjectListNode::TrackListToExternalObjectListNode(
  const rclcpp::NodeOptions & options)
: CarmaLifecycleNode{options}
{
  lifecycle_publishers_.push_back(publisher_);
  param_callback_handles_.push_back(on_set_parameters_callback_);
}

auto TrackListToExternalObjectListNode::handle_on_configure(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  publisher_ = create_publisher<carma_perception_msgs::msg::ExternalObjectList>(
    "output/external_object_list", 1);

  track_list_subscription_ = create_subscription<
    carma_cooperative_perception_interfaces::msg::TrackList>(
    "input/track_list", 1,
    [this](const carma_cooperative_perception_interfaces::msg::TrackList::SharedPtr msg_ptr) {
      if (const auto current_state{this->get_current_state().label()}; current_state == "active") {
        publish_as_external_object_list(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->track_list_subscription_->get_topic_name(), current_state.c_str());
      }
    });

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto TrackListToExternalObjectListNode::handle_on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  track_list_subscription_.reset();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto TrackListToExternalObjectListNode::handle_on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  track_list_subscription_.reset();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto TrackListToExternalObjectListNode::publish_as_external_object_list(
  const carma_cooperative_perception_interfaces::msg::TrackList & msg) const noexcept -> void
{
  auto external_object_list{to_external_object_list_msg(msg)};
  external_object_list.header.stamp = now();

  publisher_->publish(external_object_list);
}

}  // namespace carma_cooperative_perception

// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(carma_cooperative_perception::TrackListToExternalObjectListNode)  // NOLINT
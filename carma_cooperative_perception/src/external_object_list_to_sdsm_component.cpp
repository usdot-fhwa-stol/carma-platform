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

#include "carma_cooperative_perception/external_object_list_to_sdsm_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "carma_cooperative_perception/msg_conversion.hpp"
#include <memory>

namespace carma_cooperative_perception
{
ExternalObjectListToSdsmNode::ExternalObjectListToSdsmNode(const rclcpp::NodeOptions & options)
: CarmaLifecycleNode{options}
{
  lifecycle_publishers_.push_back(sdsm_publisher_);
  param_callback_handles_.push_back(on_set_parameters_callback_);
}

auto ExternalObjectListToSdsmNode::handle_on_configure(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Life cycle state transition: configuring");

  sdsm_publisher_ = create_publisher<sdsm_msg_type>("output/sdsms", 1);

  external_objects_subscription_ = create_subscription<external_objects_msg_type>(
    "input/external_objects", 1, [this](const external_objects_msg_type::SharedPtr msg_ptr) {
      const auto current_state{this->get_current_state().label()};

      if (current_state == "active") {
        publish_as_sdsm(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->georeference_subscription_->get_topic_name(), current_state.c_str());
      }
    });

  georeference_subscription_ = create_subscription<georeference_msg_type>(
    "input/georeference", 1, [this](const georeference_msg_type::SharedPtr msg_ptr) {
      const auto current_state{this->get_current_state().label()};

      if (current_state == "active") {
        update_georeference(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not "
          "activated. "
          "Current node state: '%s'",
          this->georeference_subscription_->get_topic_name(), current_state.c_str());
      }
    });

  current_pose_subscription_ = create_subscription<pose_msg_type>(
    "input/pose_stamped", 1, [this](const pose_msg_type::SharedPtr msg_ptr) {
      const auto current_state{this->get_current_state().label()};

      if (current_state == "active") {
        update_current_pose(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to recieve message on topic '%s', but the containing node is not activated."
          "current node state: '%s'",
          this->current_pose_subscription_->get_topic_name(), current_state.c_str());
      }
    });

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully configured");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto ExternalObjectListToSdsmNode::handle_on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Life cycle state transition: cleaning up");

  sdsm_publisher_.reset();
  external_objects_subscription_.reset();
  georeference_subscription_.reset();

  current_pose_subscription_.reset();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto ExternalObjectListToSdsmNode::handle_on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Life cycle state transition: shutting down");

  sdsm_publisher_.reset();
  external_objects_subscription_.reset();
  georeference_subscription_.reset();

  current_pose_subscription_.reset();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto ExternalObjectListToSdsmNode::publish_as_sdsm(const external_objects_msg_type & msg) const
  -> void
{
  try {
    const auto sdsm_output{to_sdsm_msg(msg, current_pose_, map_projector_)};

    sdsm_publisher_->publish(sdsm_output);
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not convert external object list to SDSM: %s", e.what());
  }
}

auto ExternalObjectListToSdsmNode::update_georeference(const georeference_msg_type & msg) noexcept
  -> void
{
  map_georeference_ = msg.data;
  map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg.data.c_str());
}

auto ExternalObjectListToSdsmNode::update_current_pose(const pose_msg_type & msg) noexcept
  -> void
{
  current_pose_ = msg;
}

}  // namespace carma_cooperative_perception

// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(                               // NOLINT
  carma_cooperative_perception::ExternalObjectListToSdsmNode)  // NOLINT

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

#include "carma_cooperative_perception/multiple_object_tracker_component.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <units.h>
#include <algorithm>
#include <chrono>
#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/fusing.hpp>
#include <cooperative_perception/scoring.hpp>
#include <cooperative_perception/temporal_alignment.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace carma_cooperative_perception
{

namespace mot = cooperative_perception;

auto make_ctrv_detection(
  const carma_cooperative_perception_interfaces::msg::Detection & msg) noexcept -> Detection
{
  const auto timestamp{
    units::time::second_t{static_cast<double>(msg.header.stamp.sec)} +
    units::time::nanosecond_t{static_cast<double>(msg.header.stamp.nanosec)}};

  tf2::Quaternion orientation;
  orientation.setX(msg.pose.pose.orientation.x);
  orientation.setY(msg.pose.pose.orientation.y);
  orientation.setZ(msg.pose.pose.orientation.z);
  orientation.setW(msg.pose.pose.orientation.w);

  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};

  tf2::Matrix3x3 matrix{orientation};
  matrix.getRPY(roll, pitch, yaw);

  const mot::CtrvState state{
    units::length::meter_t{msg.pose.pose.position.x},
    units::length::meter_t{msg.pose.pose.position.y},
    units::velocity::meters_per_second_t{msg.twist.twist.linear.x},
    mot::Angle{units::angle::radian_t{yaw}},
    units::angular_velocity::radians_per_second_t{msg.twist.twist.angular.z}};

  return mot::CtrvDetection{timestamp, state, mot::CtrvStateCovariance{}, mot::Uuid{msg.id}};
}

auto make_ctra_detection(
  const carma_cooperative_perception_interfaces::msg::Detection & msg) noexcept -> Detection
{
  const auto timestamp{
    units::time::second_t{static_cast<double>(msg.header.stamp.sec)} +
    units::time::nanosecond_t{static_cast<double>(msg.header.stamp.nanosec)}};

  tf2::Quaternion orientation;
  orientation.setX(msg.pose.pose.orientation.x);
  orientation.setY(msg.pose.pose.orientation.y);
  orientation.setZ(msg.pose.pose.orientation.z);
  orientation.setW(msg.pose.pose.orientation.w);

  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};

  tf2::Matrix3x3 matrix{orientation};
  matrix.getRPY(roll, pitch, yaw);

  const mot::CtraState state{
    units::length::meter_t{msg.pose.pose.position.x},
    units::length::meter_t{msg.pose.pose.position.y},
    units::velocity::meters_per_second_t{msg.twist.twist.linear.x},
    mot::Angle{units::angle::radian_t{yaw}},
    units::angular_velocity::radians_per_second_t{msg.twist.twist.angular.z},
    units::acceleration::meters_per_second_squared_t{msg.accel.accel.linear.x}};

  return mot::CtraDetection{timestamp, state, mot::CtraStateCovariance{}, mot::Uuid{msg.id}};
}

auto make_detection(const carma_cooperative_perception_interfaces::msg::Detection & msg)
  -> Detection
{
  switch (msg.motion_model) {
    case msg.MOTION_MODEL_CTRV:
      return make_ctrv_detection(msg);

    case msg.MOTION_MODEL_CTRA:
      return make_ctra_detection(msg);

    case msg.MOTION_MODEL_CV:
      break;
  }

  throw std::runtime_error("unkown motion model type '" + std::to_string(msg.motion_model) + "'");
}

MultipleObjectTrackerNode::MultipleObjectTrackerNode(const rclcpp::NodeOptions & options)
: CarmaLifecycleNode{options}
{
  // CarmaLifecycleNode base class will automatically handle lifecycle state changes for
  // lifecycle publishers and timers.
  lifecycle_publishers_.push_back(track_list_pub_);
  timers_.push_back(pipeline_execution_timer_);
}

auto MultipleObjectTrackerNode::handle_on_configure(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: configuring");

  track_list_pub_ = create_publisher<carma_cooperative_perception_interfaces::msg::TrackList>(
    "output/track_list", 1);

  detection_list_sub_ = create_subscription<
    carma_cooperative_perception_interfaces::msg::DetectionList>(
    "input/detection_list", 1,
    [this](const carma_cooperative_perception_interfaces::msg::DetectionList::SharedPtr msg_ptr) {
      if (const auto current_state{this->get_current_state().label()}; current_state == "active") {
        for (const auto detection : msg_ptr->detections) {
          detections_.push_back(make_detection(detection));
        }
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->detection_list_sub_->get_topic_name(), current_state.c_str());
      }
    });

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully configured");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::handle_on_activate(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: actiavting");

  using std::chrono_literals::operator""ms;
  if (pipeline_execution_timer_ == nullptr) {
    pipeline_execution_timer_ = create_wall_timer(500ms, [this] { execute_pipeline(); });
  } else {
    pipeline_execution_timer_->reset();
  }

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully activated");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::handle_on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: deactivating");

  pipeline_execution_timer_->cancel();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully deactivated");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::handle_on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: cleaning up");

  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  detection_list_sub_.reset();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully cleaned up");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::handle_on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO(get_logger(), "Lifecycle transition: shutting down");

  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  detection_list_sub_.reset();

  RCLCPP_INFO(get_logger(), "Lifecycle transition: successfully shut down");

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto MultipleObjectTrackerNode::execute_pipeline() const noexcept -> void
auto MultipleObjectTrackerNode::execute_pipeline() noexcept -> void
{
  if (detections_.empty()) {
    RCLCPP_DEBUG(get_logger(), "Not executing pipeline: internal detection list is empty");
    RCLCPP_DEBUG(get_logger(), "List of detections is empty. Not executing pipeline.");
    return;
  }
}

}  // namespace carma_cooperative_perception

// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(carma_cooperative_perception::MultipleObjectTrackerNode)  // NOLINT

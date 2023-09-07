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

#include "carma_cooperative_perception/external_object_list_to_detection_list_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <utility>
#include <vector>

#include "carma_cooperative_perception/geodetic.hpp"
#include "carma_cooperative_perception/units_extensions.hpp"

namespace carma_cooperative_perception
{
auto transform_from_map_to_utm(
  carma_cooperative_perception_interfaces::msg::DetectionList detection_list,
  const std::string & map_origin) -> carma_cooperative_perception_interfaces::msg::DetectionList
{
  gsl::owner<PJ_CONTEXT *> context = proj_context_create();
  proj_log_level(context, PJ_LOG_NONE);

  if (context == nullptr) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not create PROJ context: " + error_string + '.');
  }

  gsl::owner<PJ *> map_transformation = proj_create(context, map_origin.c_str());

  if (map_transformation == nullptr) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not create PROJ transform: " + error_string + '.');
  }

  std::vector<carma_cooperative_perception_interfaces::msg::Detection> new_detections;
  for (auto detection : detection_list.detections) {
    // Coordinate order is easting (meters), northing (meters)
    const auto position_planar{
      proj_coord(detection.pose.pose.position.x, detection.pose.pose.position.y, 0, 0)};
    const auto proj_inverse{proj_trans(map_transformation, PJ_DIRECTION::PJ_INV, position_planar)};
    const Wgs84Coordinate position_wgs84{
      units::angle::radian_t{proj_inverse.lp.phi}, units::angle::radian_t{proj_inverse.lp.lam},
      units::length::meter_t{detection.pose.pose.position.z}};

    const auto utm_zone{calculate_utm_zone(position_wgs84)};
    const auto position_utm{project_to_utm(position_wgs84)};

    detection.header.frame_id = to_string(utm_zone);
    detection.pose.pose.position.x = remove_units(position_utm.easting);
    detection.pose.pose.position.y = remove_units(position_utm.northing);

    new_detections.push_back(std::move(detection));
  }

  std::swap(detection_list.detections, new_detections);

  proj_destroy(map_transformation);
  proj_context_destroy(context);

  return detection_list;
}

ExternalObjectListToDetectionListNode::ExternalObjectListToDetectionListNode(
  const rclcpp::NodeOptions & options)
: CarmaLifecycleNode{options},
  publisher_{nullptr},
  external_objects_subscription_{create_subscription<input_msg_type>(
    "input/external_objects", 1,
    [this](input_msg_shared_pointer msg_ptr) {
      const auto current_state{this->get_current_state().label()};

      if (current_state == "active") {
        publish_as_detection_list(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->external_objects_subscription_->get_topic_name(), current_state.c_str());
      }
    })},
  georeference_subscription_{create_subscription<std_msgs::msg::String>(
    "input/georeference", 1, [this](std_msgs::msg::String::SharedPtr msg_ptr) {
      const auto current_state{this->get_current_state().label()};

      if (current_state == "active") {
        update_proj_string(*msg_ptr);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->georeference_subscription_->get_topic_name(), current_state.c_str());
      }
    })}
{
}

auto ExternalObjectListToDetectionListNode::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  publisher_ = create_publisher<output_msg_type>("output/detections", 1);

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto ExternalObjectListToDetectionListNode::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  publisher_->on_activate();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto ExternalObjectListToDetectionListNode::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  publisher_->on_deactivate();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto ExternalObjectListToDetectionListNode::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  publisher_.reset();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto ExternalObjectListToDetectionListNode::on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  publisher_.reset();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

}  // namespace carma_cooperative_perception

// This is not our macro, so we should not worry about linting it.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0.
RCLCPP_COMPONENTS_REGISTER_NODE(                                        // NOLINT
  carma_cooperative_perception::ExternalObjectListToDetectionListNode)  // NOLINT

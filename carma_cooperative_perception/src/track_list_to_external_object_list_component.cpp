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

auto transform_from_map_origin_to_local( // improve name scheme
  carma_cooperative_perception_interfaces::msg::TrackList track_list,
  const std::string &map_origin) -> carma_cooperative_perception_interfaces::msg::DetectionList
{

    // TODO

}


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
  publisher_ = create_publisher<output_msg_type("output/external_objects_list", 1);

  track_list_subscription = create_subscription<input_msg_type>(
    "input/track_list", 1, [this](input_msg_shared_pointer msg_ptr){
      const auto current_State{this->get_current_state().label()};

      if(current_state == "active"){
        publish_as_external_object_list(*msg_ptr);
      } else{
        RCLCPP_WARN(
          this->get_logger(),
          "Trying to receive message on the topic '%s', but the containing node is not activated. "
          "Current node state: '%s'",
          this->track_subscription_->get_topic_name(), current_state.c_str()); 
      }
    });

    // TODO

}

auto TrackListToExternalObjectListNode::handle_on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  track_subscription_.reset();
  georeference_subscription_.reset();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto TrackListToExternalObjectListNode::handle_on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */) -> carma_ros2_utils::CallbackReturn
{
  // CarmaLifecycleNode does not handle subscriber pointer reseting for us
  track_subscription_.reset();
  georeference_subscription_.reset();

  return carma_ros2_utils::CallbackReturn::SUCCESS;
}

auto TrackListToExternalObjectListNode::publish_as_external_object_list(
  const input_msg_type & msg) const -> void
{
  try {
    const auto detection_list{transform_from_map_to_utm(
      to_detection_list_msg(msg, motion_model_mapping_), map_georeference_)};

    publisher_->publish(detection_list);
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not convert external object list to detection list: %s", e.what());
  }
}

auto TrackListToExternalObjectListNode::update_georeference(
  const std_msgs::msg::String & msg) noexcept -> void
{
  map_georeference_ = msg.data;
}


} // namespace carma_cooperative_perception
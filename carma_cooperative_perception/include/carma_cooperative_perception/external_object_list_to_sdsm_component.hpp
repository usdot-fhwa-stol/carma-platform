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

#ifndef CARMA_COOPERATIVE_PERCEPTION__EXTERNAL_OBJECT_LIST_TO_SDSM_COMPONENT_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__EXTERNAL_OBJECT_LIST_TO_SDSM_COMPONENT_HPP_

#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_v2x_msgs/msg/sensor_data_sharing_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_extension/projection/local_frame_projector.h>

#include <memory>
#include <string>

namespace carma_cooperative_perception
{
class ExternalObjectListToSdsmNode : public carma_ros2_utils::CarmaLifecycleNode
{
  using sdsm_msg_type = carma_v2x_msgs::msg::SensorDataSharingMessage;
  using external_objects_msg_type = carma_perception_msgs::msg::ExternalObjectList;
  using georeference_msg_type = std_msgs::msg::String;

  using pose_msg_type = geometry_msgs::msg::PoseStamped;

public:
  explicit ExternalObjectListToSdsmNode(const rclcpp::NodeOptions & options);

  auto handle_on_configure(const rclcpp_lifecycle::State & /* previous_state */)
    -> carma_ros2_utils::CallbackReturn override;

  auto handle_on_cleanup(const rclcpp_lifecycle::State & /* previous_state */)
    -> carma_ros2_utils::CallbackReturn override;

  auto handle_on_shutdown(const rclcpp_lifecycle::State & /* previous_state */)
    -> carma_ros2_utils::CallbackReturn override;

  auto publish_as_sdsm(const external_objects_msg_type & msg) const -> void;

  auto update_georeference(const georeference_msg_type & proj_string) noexcept -> void;

  auto update_current_pose(const pose_msg_type & pose) noexcept -> void;

private:
  rclcpp_lifecycle::LifecyclePublisher<sdsm_msg_type>::SharedPtr sdsm_publisher_{nullptr};
  rclcpp::Subscription<external_objects_msg_type>::SharedPtr external_objects_subscription_{
    nullptr};
  rclcpp::Subscription<georeference_msg_type>::SharedPtr georeference_subscription_{nullptr};
  rclcpp::Subscription<pose_msg_type>::SharedPtr current_pose_subscription_{nullptr};

  std::string map_georeference_{""};
  geometry_msgs::msg::PoseStamped current_pose_;

  std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_{nullptr};
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_{nullptr};
};
}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__EXTERNAL_OBJECT_LIST_TO_SDSM_COMPONENT_HPP_

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

#ifndef CARMA_COOPERATIVE_PERCEPTION__MULTIPLE_OBJECT_TRACKER_COMPONENT_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__MULTIPLE_OBJECT_TRACKER_COMPONENT_HPP_

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <carma_cooperative_perception_interfaces/msg/detection_list.hpp>
#include <carma_cooperative_perception_interfaces/msg/track_list.hpp>

#include <cooperative_perception/ctra_model.hpp>
#include <cooperative_perception/ctrv_model.hpp>
#include <cooperative_perception/track_management.hpp>
#include <unordered_map>
#include <variant>
#include <vector>

namespace carma_cooperative_perception
{

using Detection =
  std::variant<cooperative_perception::CtrvDetection, cooperative_perception::CtraDetection>;
using Track = std::variant<cooperative_perception::CtrvTrack, cooperative_perception::CtraTrack>;

auto make_detection(const carma_cooperative_perception_interfaces::msg::Detection & msg)
  -> Detection;

class MultipleObjectTrackerNode : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  explicit MultipleObjectTrackerNode(const rclcpp::NodeOptions & options);

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

  auto store_new_detections(
    const carma_cooperative_perception_interfaces::msg::DetectionList & msg) noexcept -> void;

  auto execute_pipeline() -> void;

private:
  rclcpp::Subscription<carma_cooperative_perception_interfaces::msg::DetectionList>::SharedPtr
    detection_list_sub_{nullptr};

  rclcpp_lifecycle::LifecyclePublisher<
    carma_cooperative_perception_interfaces::msg::TrackList>::SharedPtr track_list_pub_{nullptr};

  rclcpp::TimerBase::SharedPtr pipeline_execution_timer_{nullptr};

  std::vector<Detection> detections_;
  std::unordered_map<cooperative_perception::Uuid, std::size_t> uuid_index_map_;
  cooperative_perception::FixedThresholdTrackManager<Track> track_manager_{
    cooperative_perception::PromotionThreshold{3U}, cooperative_perception::RemovalThreshold{0U}};
  units::time::nanosecond_t execution_period_{1 / units::frequency::hertz_t{2.0}};
  OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_{nullptr};
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__MULTIPLE_OBJECT_TRACKER_COMPONENT_HPP_

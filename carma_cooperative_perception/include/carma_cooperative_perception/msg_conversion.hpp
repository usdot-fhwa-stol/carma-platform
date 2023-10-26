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

#ifndef CARMA_COOPERATIVE_PERCEPTION__MSG_CONVERSION_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__MSG_CONVERSION_HPP_

#include <units.h>

#include <carma_cooperative_perception_interfaces/msg/detection_list.hpp>
#include <carma_cooperative_perception_interfaces/msg/track.hpp>
#include <carma_cooperative_perception_interfaces/msg/track_list.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <carma_v2x_msgs/msg/sensor_data_sharing_message.hpp>

#include "carma_cooperative_perception/geodetic.hpp"
#include "carma_cooperative_perception/j2735_types.hpp"
#include "carma_cooperative_perception/j3224_types.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_extension/projection/local_frame_projector.h>

#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <string>

namespace carma_cooperative_perception
{
auto to_time_msg(const DDateTime & d_date_time) noexcept -> builtin_interfaces::msg::Time;

auto calc_detection_time_stamp(DDateTime d_date_time, const MeasurementTimeOffset & offset) noexcept
  -> DDateTime;

auto to_ddate_time_msg(const builtin_interfaces::msg::Time & builtin_time) noexcept
  -> j2735_v2x_msgs::msg::DDateTime;

auto calc_sdsm_time_offset(
  const builtin_interfaces::msg::Time & external_object_list_time,
  const builtin_interfaces::msg::Time & external_object_time) noexcept
  -> carma_v2x_msgs::msg::MeasurementTimeOffset;

auto to_position_msg(const UtmCoordinate & position_utm) noexcept -> geometry_msgs::msg::Point;

auto heading_to_enu_yaw(const units::angle::degree_t & heading) noexcept -> units::angle::degree_t;

auto calc_relative_position(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const carma_v2x_msgs::msg::PositionOffsetXYZ & detected_object_data) noexcept
  -> carma_v2x_msgs::msg::PositionOffsetXYZ;

auto transform_pose_from_map_to_wgs84(
  const geometry_msgs::msg::PoseStamped & source_pose,
  const std::shared_ptr<lanelet::projection::LocalFrameProjector> & map_projection) noexcept
  -> carma_v2x_msgs::msg::Position3D;

auto to_detection_list_msg(const carma_v2x_msgs::msg::SensorDataSharingMessage & sdsm) noexcept
  -> carma_cooperative_perception_interfaces::msg::DetectionList;

struct MotionModelMapping
{
  std::uint8_t small_vehicle_model;
  std::uint8_t large_vehicle_model;
  std::uint8_t motorcycle_model;
  std::uint8_t pedestrian_model;
  std::uint8_t unknown_model;
};

auto to_detection_msg(
  const carma_perception_msgs::msg::ExternalObject & object,
  const MotionModelMapping & motion_model_mapping) noexcept
  -> carma_cooperative_perception_interfaces::msg::Detection;

auto to_detection_list_msg(
  const carma_perception_msgs::msg::ExternalObjectList & object_list,
  const MotionModelMapping & motion_model_mapping) noexcept
  -> carma_cooperative_perception_interfaces::msg::DetectionList;

auto to_external_object_msg(
  const carma_cooperative_perception_interfaces::msg::Track & track) noexcept
  -> carma_perception_msgs::msg::ExternalObject;

auto to_external_object_list_msg(
  const carma_cooperative_perception_interfaces::msg::TrackList & track_list) noexcept
  -> carma_perception_msgs::msg::ExternalObjectList;

auto to_sdsm_msg(
  const carma_perception_msgs::msg::ExternalObjectList & external_object_list,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::shared_ptr<lanelet::projection::LocalFrameProjector> & map_projection) noexcept
  -> carma_v2x_msgs::msg::SensorDataSharingMessage;

auto to_detected_object_data_msg(
  const carma_perception_msgs::msg::ExternalObject & external_object,
  const std::shared_ptr<lanelet::projection::LocalFrameProjector> & map_projection) noexcept
  -> carma_v2x_msgs::msg::DetectedObjectData;

auto enu_orientation_to_true_heading(
  double yaw, const lanelet::BasicPoint3d & obj_pose,
  const std::shared_ptr<lanelet::projection::LocalFrameProjector> & map_projection) noexcept
  -> units::angle::degree_t;

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__MSG_CONVERSION_HPP_

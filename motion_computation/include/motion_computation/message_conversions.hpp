// Copyright 2020-2023 Leidos
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

#ifndef MOTION_COMPUTATION__MESSAGE_CONVERSIONS_HPP_
#define MOTION_COMPUTATION__MESSAGE_CONVERSIONS_HPP_

#include <lanelet2_extension/projection/local_frame_projector.h>
#include <tf2/LinearMath/Quaternion.h>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <carma_v2x_msgs/msg/mobility_path.hpp>
#include <carma_v2x_msgs/msg/psm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace motion_computation
{

namespace conversion
{

void convert(
  const carma_v2x_msgs::msg::PSM & in_msg, carma_perception_msgs::msg::ExternalObject & out_msg,
  const std::string & map_frame_id, double pred_period, double pred_step_size,
  const lanelet::projection::LocalFrameProjector & map_projector,
  const tf2::Quaternion & ned_in_map_rotation,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock);

void convert(
  const carma_v2x_msgs::msg::BSM & in_msg, carma_perception_msgs::msg::ExternalObject & out_msg,
  const std::string & map_frame_id, double pred_period, double pred_step_size,
  const lanelet::projection::LocalFrameProjector & map_projector,
  tf2::Quaternion ned_in_map_rotation);

void convert(
  const carma_v2x_msgs::msg::MobilityPath & in_msg,
  carma_perception_msgs::msg::ExternalObject & out_msg,
  const lanelet::projection::LocalFrameProjector & map_projector);
}  // namespace conversion
}  // namespace motion_computation

#endif  // MOTION_COMPUTATION__MESSAGE_CONVERSIONS_HPP_

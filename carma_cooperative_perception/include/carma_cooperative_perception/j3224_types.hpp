#ifndef CARMA_COOPERATIVE_PERCEPTION_J3224_TYPES_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_J3224_TYPES_HPP_

/*
 * Copyright 2023 Leidos
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <units.h>

#include <j3224_v2x_msgs/msg/measurement_time_offset.hpp>
#include <j3224_v2x_msgs/msg/position_offset_xyz.hpp>
#include <optional>

namespace carma_cooperative_perception
{

struct PositionOffsetXYZ
{
  units::length::decimeter_t offset_x;
  units::length::decimeter_t offset_y;
  std::optional<units::length::decimeter_t> offset_z;

  static auto from_msg(const j3224_v2x_msgs::msg::PositionOffsetXYZ & msg)
  {
    PositionOffsetXYZ offset{
      .offset_x{units::length::decimeter_t{static_cast<double>(msg.offset_x.object_distance)}},
      .offset_y{units::length::decimeter_t{static_cast<double>(msg.offset_y.object_distance)}},
      .offset_z{std::nullopt}};

    if (msg.presence_vector & msg.HAS_OFFSET_Z) {
      offset.offset_z =
        units::length::decimeter_t{static_cast<double>(msg.offset_z.object_distance)};
    }

    return offset;
  }
};

struct MeasurementTimeOffset
{
  units::time::millisecond_t measurement_time_offset;

  static auto from_msg(const j3224_v2x_msgs::msg::MeasurementTimeOffset & msg)
  {
    return MeasurementTimeOffset{
      units::time::millisecond_t{static_cast<double>(msg.measurement_time_offset)}};
  }
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION_J3224_TYPES_HPP_

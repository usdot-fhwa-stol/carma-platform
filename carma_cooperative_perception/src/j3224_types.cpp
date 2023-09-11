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

#include "carma_cooperative_perception/j3224_types.hpp"

namespace carma_cooperative_perception
{
auto PositionOffsetXYZ::from_msg(const j3224_v2x_msgs::msg::PositionOffsetXYZ & msg) noexcept
  -> PositionOffsetXYZ
{
  PositionOffsetXYZ offset{
    units::length::decimeter_t{static_cast<double>(msg.offset_x.object_distance)},
    units::length::decimeter_t{static_cast<double>(msg.offset_y.object_distance)}, std::nullopt};

  if (msg.presence_vector & msg.HAS_OFFSET_Z) {
    offset.offset_z = units::length::decimeter_t{static_cast<double>(msg.offset_z.object_distance)};
  }

  return offset;
}

auto PositionOffsetXYZ::from_msg(const carma_v2x_msgs::msg::PositionOffsetXYZ & msg) noexcept
  -> PositionOffsetXYZ
{
  PositionOffsetXYZ offset{
    units::length::meter_t{static_cast<double>(msg.offset_x.object_distance)},
    units::length::meter_t{static_cast<double>(msg.offset_y.object_distance)}, std::nullopt};

  if (msg.presence_vector & msg.HAS_OFFSET_Z) {
    offset.offset_z = units::length::meter_t{static_cast<double>(msg.offset_z.object_distance)};
  }

  return offset;
}

auto MeasurementTimeOffset::from_msg(
  const j3224_v2x_msgs::msg::MeasurementTimeOffset & msg) noexcept -> MeasurementTimeOffset
{
  return {units::time::millisecond_t{static_cast<double>(msg.measurement_time_offset)}};
}

auto MeasurementTimeOffset::from_msg(
  const carma_v2x_msgs::msg::MeasurementTimeOffset & msg) noexcept -> MeasurementTimeOffset
{
  return {units::time::second_t{static_cast<double>(msg.measurement_time_offset)}};
}

}  // namespace carma_cooperative_perception

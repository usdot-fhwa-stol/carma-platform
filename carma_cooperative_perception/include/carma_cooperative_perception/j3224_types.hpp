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

#ifndef CARMA_COOPERATIVE_PERCEPTION__J3224_TYPES_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__J3224_TYPES_HPP_

#include <units.h>

#include <carma_v2x_msgs/msg/measurement_time_offset.hpp>
#include <carma_v2x_msgs/msg/position_offset_xyz.hpp>
#include <j3224_v2x_msgs/msg/measurement_time_offset.hpp>
#include <j3224_v2x_msgs/msg/position_offset_xyz.hpp>
#include <optional>

namespace carma_cooperative_perception
{
struct PositionOffsetXYZ
{
  units::length::decimeter_t offset_x{0.0};
  units::length::decimeter_t offset_y{0.0};
  std::optional<units::length::decimeter_t> offset_z;

  [[nodiscard]] static auto from_msg(const j3224_v2x_msgs::msg::PositionOffsetXYZ & msg) noexcept
    -> PositionOffsetXYZ;

  [[nodiscard]] static auto from_msg(const carma_v2x_msgs::msg::PositionOffsetXYZ & msg) noexcept
    -> PositionOffsetXYZ;
};

struct MeasurementTimeOffset
{
  units::time::millisecond_t measurement_time_offset;

  [[nodiscard]] static auto from_msg(
    const j3224_v2x_msgs::msg::MeasurementTimeOffset & msg) noexcept -> MeasurementTimeOffset;

  [[nodiscard]] static auto from_msg(
    const carma_v2x_msgs::msg::MeasurementTimeOffset & msg) noexcept -> MeasurementTimeOffset;
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__J3224_TYPES_HPP_

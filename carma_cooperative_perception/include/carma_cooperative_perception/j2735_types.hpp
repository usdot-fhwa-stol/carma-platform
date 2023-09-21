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

#ifndef CARMA_COOPERATIVE_PERCEPTION__J2735_TYPES_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__J2735_TYPES_HPP_

#include <units.h>

#include <carma_v2x_msgs/msg/acceleration_set4_way.hpp>
#include <carma_v2x_msgs/msg/heading.hpp>
#include <carma_v2x_msgs/msg/position3_d.hpp>
#include <carma_v2x_msgs/msg/speed.hpp>
#include <j2735_v2x_msgs/msg/acceleration_set4_way.hpp>
#include <j2735_v2x_msgs/msg/d_date_time.hpp>
#include <j2735_v2x_msgs/msg/heading.hpp>
#include <j2735_v2x_msgs/msg/position3_d.hpp>
#include <j2735_v2x_msgs/msg/speed.hpp>
#include <optional>

#include "carma_cooperative_perception/month.hpp"
#include "carma_cooperative_perception/units_extensions.hpp"

namespace carma_cooperative_perception
{
struct DDateTime
{
  std::optional<units::time::year_t> year;
  std::optional<Month> month;
  std::optional<units::time::day_t> day;
  std::optional<units::time::hour_t> hour;
  std::optional<units::time::minute_t> minute;
  std::optional<units::time::second_t> second;
  std::optional<units::time::minute_t> time_zone_offset;

  [[nodiscard]] static auto from_msg(const j2735_v2x_msgs::msg::DDateTime & msg) noexcept
    -> DDateTime;
};

struct AccelerationSet4Way
{
  units::acceleration::centi_meters_per_second_squared_t longitudinal;
  units::acceleration::centi_meters_per_second_squared_t lateral;
  units::acceleration::two_centi_standard_gravities_t vert;
  units::angular_velocity::centi_degrees_per_second_t yaw_rate;

  [[nodiscard]] static auto from_msg(const j2735_v2x_msgs::msg::AccelerationSet4Way & msg) noexcept
    -> AccelerationSet4Way;

  [[nodiscard]] static auto from_msg(const carma_v2x_msgs::msg::AccelerationSet4Way & msg) noexcept
    -> AccelerationSet4Way;
};

struct Position3D
{
  units::angle::deci_micro_degrees_t latitude{0.0};
  units::angle::deci_micro_degrees_t longitude{0.0};
  std::optional<units::length::deca_centimeters_t> elevation;

  [[nodiscard]] static auto from_msg(const j2735_v2x_msgs::msg::Position3D & msg) noexcept
    -> Position3D;

  [[nodiscard]] static auto from_msg(const carma_v2x_msgs::msg::Position3D & msg) noexcept
    -> Position3D;
};

struct Heading
{
  units::angle::eighth_deci_degrees_t heading;

  [[nodiscard]] static auto from_msg(const j2735_v2x_msgs::msg::Heading & heading) noexcept
    -> Heading;

  [[nodiscard]] static auto from_msg(const carma_v2x_msgs::msg::Heading & heading) noexcept
    -> Heading;
};

struct Speed
{
  units::velocity::two_centi_meters_per_second_t speed;

  [[nodiscard]] static auto from_msg(const j2735_v2x_msgs::msg::Speed & speed) noexcept -> Speed;

  [[nodiscard]] static auto from_msg(const carma_v2x_msgs::msg::Speed & speed) noexcept -> Speed;
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__J2735_TYPES_HPP_

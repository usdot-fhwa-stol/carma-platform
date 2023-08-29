#ifndef CARMA_COOPERATIVE_PERCEPTION_J2735_TYPES_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_J2735_TYPES_HPP_

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

  static auto from_msg(const j2735_v2x_msgs::msg::DDateTime & msg) noexcept
  {
    DDateTime d_date_time;

    if (msg.year.year != msg.year.UNAVAILABLE) {
      d_date_time.year = units::time::year_t{static_cast<double>(msg.year.year)};
    }

    if (msg.month.month != msg.month.UNAVAILABLE) {
      d_date_time.month = Month{msg.month.month};
    }

    if (msg.day.day != msg.day.UNAVAILABLE) {
      d_date_time.day = units::time::day_t{static_cast<double>(msg.day.day)};
    }

    if (msg.hour.hour != msg.hour.UNAVAILABLE) {
      d_date_time.hour = units::time::hour_t{static_cast<double>(msg.hour.hour)};
    }

    if (msg.minute.minute != msg.minute.UNAVAILABLE) {
      d_date_time.minute = units::time::minute_t{static_cast<double>(msg.minute.minute)};
    }

    if (msg.second.millisecond != msg.second.UNAVAILABLE) {
      d_date_time.second = units::time::millisecond_t{static_cast<double>(msg.second.millisecond)};
    }

    if (msg.offset.offset_minute != msg.offset.UNAVAILABLE) {
      d_date_time.time_zone_offset =
        units::time::minute_t{static_cast<double>(msg.offset.offset_minute)};
    }

    return d_date_time;
  }
};

struct AccelerationSet4Way
{
  units::acceleration::centi_meters_per_second_squared_t longitudinal;
  units::acceleration::centi_meters_per_second_squared_t lateral;
  units::acceleration::two_centi_standard_gravities_t vert;
  units::angular_velocity::centi_degrees_per_second_t yaw_rate;

  static auto from_msg(const j2735_v2x_msgs::msg::AccelerationSet4Way & msg)
  {
    return AccelerationSet4Way{
      .longitudinal{units::acceleration::centi_meters_per_second_squared_t{
        static_cast<double>(msg.longitudinal)}},
      .lateral{
        units::acceleration::centi_meters_per_second_squared_t{static_cast<double>(msg.lateral)}},
      .vert{units::acceleration::two_centi_standard_gravities_t{static_cast<double>(msg.vert)}},
      .yaw_rate{
        units::angular_velocity::centi_degrees_per_second_t{static_cast<double>(msg.yaw_rate)}}};
  }
};

struct Position3D
{
  units::angle::deci_micro_degrees_t latitude;
  units::angle::deci_micro_degrees_t longitude;
  std::optional<units::length::deca_centimeters_t> elevation;

  static auto from_msg(const j2735_v2x_msgs::msg::Position3D & msg)
  {
    Position3D position{
      .latitude{units::angle::deci_micro_degrees_t{static_cast<double>(msg.latitude)}},
      .longitude{units::angle::deci_micro_degrees_t{static_cast<double>(msg.longitude)}},
      .elevation{std::nullopt}};

    if (msg.elevation_exists) {
      position.elevation = units::length::deca_centimeters_t{static_cast<double>(msg.elevation)};
    }

    return position;
  }
};

struct Heading
{
  units::angle::eighth_deci_degrees_t heading;

  static auto from_msg(const j2735_v2x_msgs::msg::Heading & heading)
  {
    return Heading{units::angle::eighth_deci_degrees_t{static_cast<double>(heading.heading)}};
  }
};

struct Speed
{
  units::velocity::two_centi_meters_per_second_t speed;

  static auto from_msg(const j2735_v2x_msgs::msg::Speed & speed)
  {
    return Speed{units::velocity::two_centi_meters_per_second_t{static_cast<double>(speed.speed)}};
  }
};

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION_J2735_TYPES_HPP_

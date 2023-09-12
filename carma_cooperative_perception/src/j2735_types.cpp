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

#include "carma_cooperative_perception/j2735_types.hpp"

namespace carma_cooperative_perception
{
auto DDateTime::from_msg(const j2735_v2x_msgs::msg::DDateTime & msg) noexcept -> DDateTime
{
  DDateTime d_date_time;

  if (msg.presence_vector & msg.YEAR) {
    d_date_time.year = units::time::year_t{static_cast<double>(msg.year.year)};
  }

  if (msg.presence_vector & msg.MONTH) {
    d_date_time.month = Month{msg.month.month};
  }

  if (msg.presence_vector & msg.DAY) {
    d_date_time.day = units::time::day_t{static_cast<double>(msg.day.day)};
  }

  if (msg.presence_vector & msg.HOUR) {
    d_date_time.hour = units::time::hour_t{static_cast<double>(msg.hour.hour)};
  }

  if (msg.presence_vector & msg.MINUTE) {
    d_date_time.minute = units::time::minute_t{static_cast<double>(msg.minute.minute)};
  }

  if (msg.presence_vector & msg.SECOND) {
    d_date_time.second = units::time::millisecond_t{static_cast<double>(msg.second.millisecond)};
  }

  if (msg.presence_vector & msg.OFFSET) {
    d_date_time.time_zone_offset =
      units::time::minute_t{static_cast<double>(msg.offset.offset_minute)};
  }

  return d_date_time;
}

auto AccelerationSet4Way::from_msg(const j2735_v2x_msgs::msg::AccelerationSet4Way & msg) noexcept
  -> AccelerationSet4Way
{
  return {
    units::acceleration::centi_meters_per_second_squared_t{static_cast<double>(msg.longitudinal)},
    units::acceleration::centi_meters_per_second_squared_t{static_cast<double>(msg.lateral)},
    units::acceleration::two_centi_standard_gravities_t{static_cast<double>(msg.vert)},
    units::angular_velocity::centi_degrees_per_second_t{static_cast<double>(msg.yaw_rate)}};
}

auto AccelerationSet4Way::from_msg(const carma_v2x_msgs::msg::AccelerationSet4Way & msg) noexcept
  -> AccelerationSet4Way
{
  return {
    units::acceleration::meters_per_second_squared_t{static_cast<double>(msg.longitudinal)},
    units::acceleration::meters_per_second_squared_t{static_cast<double>(msg.lateral)},
    units::acceleration::meters_per_second_squared_t{static_cast<double>(msg.vert)},
    units::angular_velocity::degrees_per_second_t{static_cast<double>(msg.yaw_rate)}};
}

auto Position3D::from_msg(const j2735_v2x_msgs::msg::Position3D & msg) noexcept -> Position3D
{
  Position3D position{
    units::angle::deci_micro_degrees_t{static_cast<double>(msg.latitude)},
    units::angle::deci_micro_degrees_t{static_cast<double>(msg.longitude)}, std::nullopt};

  if (msg.elevation_exists) {
    position.elevation = units::length::deca_centimeters_t{static_cast<double>(msg.elevation)};
  }

  return position;
}

auto Position3D::from_msg(const carma_v2x_msgs::msg::Position3D & msg) noexcept -> Position3D
{
  Position3D position{
    units::angle::degree_t{static_cast<double>(msg.latitude)},
    units::angle::degree_t{static_cast<double>(msg.longitude)}, std::nullopt};

  if (msg.elevation_exists) {
    position.elevation = units::length::meter_t{static_cast<double>(msg.elevation)};
  }

  return position;
}

auto Heading::from_msg(const j2735_v2x_msgs::msg::Heading & heading) noexcept -> Heading
{
  return {units::angle::eighth_deci_degrees_t{static_cast<double>(heading.heading)}};
}

auto Heading::from_msg(const carma_v2x_msgs::msg::Heading & heading) noexcept -> Heading
{
  return {units::angle::degree_t{static_cast<double>(heading.heading)}};
}

auto Speed::from_msg(const j2735_v2x_msgs::msg::Speed & speed) noexcept -> Speed
{
  return {units::velocity::two_centi_meters_per_second_t{static_cast<double>(speed.speed)}};
}

auto Speed::from_msg(const carma_v2x_msgs::msg::Speed & speed) noexcept -> Speed
{
  return {units::velocity::meters_per_second_t{static_cast<double>(speed.speed)}};
}

}  // namespace carma_cooperative_perception

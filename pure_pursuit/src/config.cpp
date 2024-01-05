// Copyright 2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#include <limits>
#include <stdexcept>
#include "pure_pursuit/config.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace pure_pursuit
{
////////////////////////////////////////////////////////////////////////////////
Config::Config(
  const float32_t minimum_lookahead_distance,
  const float32_t maximum_lookahead_distance,
  const float32_t speed_to_lookahead_ratio,
  const bool8_t is_interpolate_lookahead_point,
  const bool8_t is_delay_compensation,
  const float32_t emergency_stop_distance,
  const float32_t speed_thres_traveling_direction,
  const float32_t distance_front_rear_wheel)
: m_minimum_lookahead_distance(minimum_lookahead_distance),
  m_maximum_lookahead_distance(maximum_lookahead_distance),
  m_speed_to_lookahead_ratio(speed_to_lookahead_ratio),
  m_is_interpolate_lookahead_point(is_interpolate_lookahead_point),
  m_is_delay_compensation(is_delay_compensation),
  m_emergency_stop_distance(emergency_stop_distance),
  m_speed_thres_traveling_direction(speed_thres_traveling_direction),
  m_distance_front_rear_wheel(distance_front_rear_wheel)
{
  if (m_minimum_lookahead_distance <= 0.0F) {
    throw std::domain_error("pure_pursuit::Config minimum lookahead distance is lower than 0");
  }
  if (m_maximum_lookahead_distance <= 0.0F) {
    throw std::domain_error("pure_pursuit::Config maximum lookahead distance is lower than 0");
  }
  if (m_speed_to_lookahead_ratio <= 0.0F) {
    throw std::domain_error("pure_pursuit::Config: speed to lookahead ratio is lower than 0");
  }
  if (emergency_stop_distance <= 0.0F) {
    throw std::domain_error("pure_pursuit::Config: emergency_stop_distance is lower than 0");
  }
  if (m_speed_thres_traveling_direction < 0.0F) {
    throw std::domain_error(
            "pure_pursuit::Config: speed_thres_traveling_direction is lower than 0");
  }
  if (m_distance_front_rear_wheel < 0.0F) {
    throw std::domain_error(
            "pure_pursuit::Config: distance_front_rear_wheel is lower than 0");
  }
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_minimum_lookahead_distance() const noexcept
{
  return m_minimum_lookahead_distance;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_maximum_lookahead_distance() const noexcept
{
  return m_maximum_lookahead_distance;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_speed_to_lookahead_ratio() const noexcept
{
  return m_speed_to_lookahead_ratio;
}
////////////////////////////////////////////////////////////////////////////////
bool8_t Config::get_is_interpolate_lookahead_point() const noexcept
{
  return m_is_interpolate_lookahead_point;
}
////////////////////////////////////////////////////////////////////////////////
bool8_t Config::get_is_delay_compensation() const noexcept
{
  return m_is_delay_compensation;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_emergency_stop_distance() const noexcept
{
  return m_emergency_stop_distance;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_speed_thres_traveling_direction() const noexcept
{
  return m_speed_thres_traveling_direction;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::get_distance_front_rear_wheel() const noexcept
{
  return m_distance_front_rear_wheel;
}
}  // namespace pure_pursuit
}  // namespace control
}  // namespace motion
}  // namespace autoware

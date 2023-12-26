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
#ifndef PURE_PURSUIT__CONFIG_HPP_
#define PURE_PURSUIT__CONFIG_HPP_

#include <pure_pursuit/visibility_control.hpp>
#include <common/types.hpp>
#include <iostream>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace control
{
namespace pure_pursuit
{

/// \brief A configuration class for the PurePursuit class's Integrator.
struct PURE_PURSUIT_PUBLIC IntegratorConfig
{
  double dt = 0.1;
  double integrator_max_pp = 0.0;
  double integrator_min_pp = 0.0;
  double Ki_pp = 0.0;
  double integral = 0.0;
  bool is_integrator_enabled = false;
  
  friend std::ostream& operator<<(std::ostream& output, const IntegratorConfig& c)
  {
    output << "IntegratorConfig { " << std::endl
           << "dt: " << c.dt << std::endl
           << "integrator_max_pp: " << c.integrator_max_pp << std::endl
           << "integrator_min_pp: " << c.integrator_min_pp << std::endl
           << "Ki_pp: " << c.Ki_pp << std::endl
           << "is_integrator_enabled: " << c.is_integrator_enabled << std::endl
           << "integral: " << c.integral << std::endl
           << "}" << std::endl;
    return output;
  }
};

/// \brief A configuration class for the PurePursuit class.
class PURE_PURSUIT_PUBLIC Config
{
public:
  /// \brief Constructor
  /// \param[in] minimum_lookahead_distance The minimum lookahead distance (meter)
  ///            for the pure pursuit
  /// \param[in] maximum_lookahead_distance The maximum lookahead distance (meter)
  ///            for the pure pursuit
  /// \param[in] speed_to_lookahead_ratio The conversion ratio from the speed to
  ///            the lookahead distance. The ratio is equal to the duration (s)
  /// \param[in] is_interpolate_lookahead_point The boolean whether using the interpolation
  ///            for determining the target position
  /// \param[in] is_delay_compensation The boolean whethre using the delay compensation
  ///            for estimating the current vehicle position at the current timestamp
  /// \param[in] emergency_stop_distance The emergency stop distance for the emergency stop
  /// \param[in] speed_thres_traveling_direction The speed threshold for
  ///            determining the traveling direction
  /// \param[in] distance_front_rear_wheel The distance between front and rear wheels
  Config(
    const float32_t minimum_lookahead_distance,
    const float32_t maximum_lookahead_distance,
    const float32_t speed_to_lookahead_ratio,
    const bool8_t is_interpolate_lookahead_point,
    const bool8_t is_delay_compensation,
    const float32_t emergency_stop_distance,
    const float32_t speed_thres_traveling_direction,
    const float32_t distance_front_rear_wheel);
  /// \brief Gets the minimum lookahead distance for the pure pursuit
  /// \return Fixed value
  float32_t get_minimum_lookahead_distance() const noexcept;
  /// \brief Gets the maximum lookahead distance for the pure pursuit
  /// \return Fixed value
  float32_t get_maximum_lookahead_distance() const noexcept;
  /// \brief Gets the maximum lookahead distance for the pure pursuit
  /// \return Fixed value
  float32_t get_speed_to_lookahead_ratio() const noexcept;
  /// \brief Gets the boolean whether using the interpolation to get the target point
  /// \return Fixed value
  bool8_t get_is_interpolate_lookahead_point() const noexcept;
  /// \brief Gets the boolean whether using the delay compensation to estimate the current point
  /// \return Fixed value
  bool8_t get_is_delay_compensation() const noexcept;
  /// \brief Gets the emergency target distance for the emergency stop
  /// \return Fixed value
  float32_t get_emergency_stop_distance() const noexcept;
  /// \brief Gets the speed threshold for determining the traveling direction
  /// \return Fixed value
  float32_t get_speed_thres_traveling_direction() const noexcept;
  /// \brief Gets the distance between front and rear wheels
  /// \return Fixed value
  float32_t get_distance_front_rear_wheel() const noexcept;

private:
  float32_t m_minimum_lookahead_distance;
  float32_t m_maximum_lookahead_distance;
  float32_t m_speed_to_lookahead_ratio;
  bool8_t m_is_interpolate_lookahead_point;
  bool8_t m_is_delay_compensation;
  float32_t m_emergency_stop_distance;
  float32_t m_speed_thres_traveling_direction;
  float32_t m_distance_front_rear_wheel;
};  // class Config
}  // namespace pure_pursuit
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT__CONFIG_HPP_

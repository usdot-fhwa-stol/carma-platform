#pragma once
/*
 * Copyright (C) 2018-2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <vector>
#include "conversions/conversions.hpp"

namespace trajectory_utils
{
/**
 * \brief Identifies the index of the provided downtrack and speed lists where the specified time_span will be exceeded.
 *
 * \param downtracks The list of downtrack points
 * \param speeds A list of the speed at each downtrack point. Must be the same size as the downtracks vector
 * \param time_span The time span which will be exceeded by the returned index.
 *
 * \return The index in the downtracks list where the time_span is exceeded. A time_span of 0 indicates that none of the
 * provided points fit within the timespan (ie. time_span = 0). An index of downtracks.size() indicates that all the
 * points fit within the time_span
 */
size_t time_boundary_index(const std::vector<double>& downtracks, const std::vector<double>& speeds, double time_span);

/**
 * \brief Computes the maximum constant speed at which a point mass can travel around a circle of the provided curvature
 * without exceeded the provided lateral acceleration limit This is computed using the standard centripital acceleration
 * equation a = (v^2) / R
 *
 * \param curvature The curvature being traveled where curvature is defined as 1/radius. Curvature has units of 1/m
 * \param lateral_accel_limit The maximum acceleration limit in m/s^2
 *
 * \return The maximum speed
 */
double constrain_speed_for_curvature(double curvature, double lateral_accel_limit);

/**
 * \brief Applies the constrain_speed_for_curvature method to each element of the provided curvatures.
 *
 * \param curvatures Vector of curvatures
 * \param lateral_accel_limit The maximum acceleration limit in m/s^2
 *
 * \return Vector of maximum speed values that match up with the input curvatures list
 *
 */
std::vector<double> constrained_speeds_for_curvatures(std::vector<double> curvatures, double lateral_accel_limit);

/**
 * \brief Computes a list of acceleration limited speeds at the provided downtrack points
 *        based on the provided planned speeds at those points and the provided accel and decel limits
 *
 * \param downtracks A list of downtrack points where the speed is defined. Units m
 * \param speeds A list of speeds that match to each downtrack point. Must have the same size as the downtracks vector.
 * Units m/s \param accel_limit The maximum acceleration limit. Units m/s^2. Must be positive \param decel_limit The
 * maximum deceleration limit. Units m/s^2. Must be positive
 *
 * \return A list of acceleration limited speeds at each downtrack point
 *
 */
std::vector<double> apply_accel_limits_by_distance(std::vector<double> downtracks, std::vector<double> speeds,
                                                   double accel_limit, double decel_limit);

/**
 * \brief Computes a list of acceleration limited speeds at the provided time points
 *        based on the provided planned speeds at those points and the provided accel and decel limits
 *
 * \param times A list of time points where the speed is defined. Units s
 * \param speeds A list of speeds that match to each downtrack point. Must have the same size as the times vector. Units
 * m/s \param accel_limit The maximum acceleration limit. Units m/s^2. Must be positive \param decel_limit The maximum
 * deceleration limit. Units m/s^2. Must be positive
 *
 * \return A list of acceleration limited speeds at each time point
 *
 */
std::vector<double> apply_accel_limits_by_time(std::vector<double> times, std::vector<double> speeds,
                                               double accel_limit, double decel_limit);

/**
 * \brief Shifts the values of the input vector back by the specified look ahead count. Trailing points are assigned the
 * value at index values.size() -1.
 * For example, {0, 1, 2, 3, 4, 5} with lookahead_count = 2 will become -> {2, 3, 4, 5, 5, 5}
 * 
 * \param values The vector of values to shift
 * \param lookahead_count The value ahead to grab for each point
 *
 * \return A shifted vector of values
 *
 */
template <class T>
std::vector<T> shift_by_lookahead(std::vector<T> values, unsigned int lookahead_count)
{
  std::vector<T> output;
  output.reserve(values.size());

  for (int i = 0; i < values.size(); i++)
  {
    T lookahead_value = 0;
    if (i + lookahead_count < values.size() - 1)
    {
      lookahead_value = values[i + lookahead_count];
    }
    else
    {
      lookahead_value = values[values.size() - 1];
    }

    output.push_back(lookahead_value);
  }

  return output;
}

}  // namespace trajectory_utils
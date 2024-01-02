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
#include <trajectory_utils/trajectory_utils.hpp>
#include <exception>
#include <stdexcept>
#include <math.h>
#include <algorithm>

namespace trajectory_utils
{
size_t time_boundary_index(const std::vector<double>& downtracks, const std::vector<double>& speeds, double time_span)
{
  if (downtracks.size() != speeds.size())
  {
    throw std::invalid_argument("Input vectors must be of equal size");
  }

  if (downtracks.size() == 0 || time_span <= 0.0)
  {
    return 0;
  }

  std::vector<double> times;
  conversions::speed_to_time(downtracks, speeds, &times);

  double current_time = 0;
  for (size_t i = 0; i < times.size(); i++)
  {
    if (times[i] > time_span)
    {
      return i;
    }
  }

  return times.size();
}

double constrain_speed_for_curvature(double curvature, double lateral_accel_limit)
{
  // Check at compile time for infinity availability
  static_assert(std::numeric_limits<double>::has_infinity, "This code requires compilation using a system that "
                                                           "supports IEEE 754 for access to positive infinity values");

  // Solve a = v^2/r (k = 1/r) for v
  // a = v^2 * k
  // a / k = v^2
  // v = sqrt(a / k)

  if (fabs(curvature) < 0.00000001)
  {  // Check for curvature of 0.
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(fabs(lateral_accel_limit / curvature));
}

std::vector<double> constrained_speeds_for_curvatures(std::vector<double> curvatures, double lateral_accel_limit)
{
  std::vector<double> out;
  for (double k : curvatures)
  {
    out.push_back(trajectory_utils::constrain_speed_for_curvature(k, lateral_accel_limit));
  }

  return out;
}

std::vector<double> apply_accel_limits_by_distance(std::vector<double> downtracks, std::vector<double> speeds,
                                                   double accel_limit, double decel_limit)
{
  if (downtracks.size() != speeds.size())
  {
    throw std::invalid_argument("Downtracks and speeds do not have the same size");
  }

  if (accel_limit <= 0 || decel_limit <= 0)
  {
    throw std::invalid_argument("Accel and Decel limits should be positive");
  }

  std::vector<double> output;
  output.reserve(downtracks.size());

  if (downtracks.size() == 0)
  {
    return output;
  }

  output.push_back(speeds[0]);  // First point will be unchanged

  for (size_t i = 1; i < downtracks.size(); i++)
  {
    double delta_d = downtracks[i] - downtracks[i - 1];
    double prev_speed = output.back();
    double cur_speed = speeds[i];
    double new_speed = cur_speed;
    if (cur_speed > prev_speed)
    {  // Acceleration case
      new_speed = std::min(cur_speed, sqrt(prev_speed * prev_speed + 2 * accel_limit * delta_d));
    }
    else if (cur_speed < prev_speed)
    {  // Deceleration case
      new_speed = std::max(cur_speed, sqrt(prev_speed * prev_speed - 2 * decel_limit * delta_d));
    }

    new_speed = std::max(0.0, new_speed);

    output.push_back(new_speed);
  }

  return output;
}

std::vector<double> apply_accel_limits_by_time(std::vector<double> times, std::vector<double> speeds,
                                               double accel_limit, double decel_limit)
{
  if (times.size() != speeds.size())
  {
    throw std::invalid_argument("Times and speeds do not have the same size");
  }

  if (accel_limit <= 0 || decel_limit <= 0)
  {
    throw std::invalid_argument("Accel and Decel limits should be positive");
  }

  std::vector<double> output;
  output.reserve(times.size());

  if (times.size() == 0)
  {
    return output;
  }

  output.push_back(speeds[0]);  // First point will be unchanged

  for (size_t i = 1; i < times.size(); i++)
  {
    double delta_t = times[i] - times[i - 1];
    double prev_speed = output.back();
    double cur_speed = speeds[i];
    double new_speed = cur_speed;
    if (cur_speed > prev_speed)
    {  // Acceleration case
      new_speed = std::min(cur_speed, prev_speed + accel_limit * delta_t);
    }
    else if (cur_speed < prev_speed)
    {  // Deceleration case
      new_speed = std::max(cur_speed, prev_speed - decel_limit * delta_t);
    }

    new_speed = std::max(0.0, new_speed);

    output.push_back(new_speed);
  }

  return output;
}
};  // namespace trajectory_utils
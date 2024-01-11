#pragma once
/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>

namespace trajectory_utils
{
namespace conversions
{

/**
 * \brief Converts a list of speeds to a list of times for the corresponding downtrack points
 *
 * \param downtracks The downtrack points where each speed and time will be
 * \param speeds The speed at each downtrack point. Must have the same length as the downtracks list
 * \param times Output parameter which points to the vector which will store the times at each point.
 *              The first time will always be 0 as the times are relative
 */
void speed_to_time(const std::vector<double>& downtrack, const std::vector<double>& speeds, std::vector<double>* times);

/**
 * \brief Converts a list of times to a list of speeds for the corresponding downtrack points
 *
 * \param downtracks The downtrack points where each speed and time will be
 * \param times The time at each downtrack point. Must have the same length as the downtracks list
 * \param initial_speed
 * \param speeds Output parameter which points to the vector which will store the speeds at each point.
 *              The first speed will always be initial_speed
 * \throw std::runtime_error if less than -0.01 speed detected, which could indicate invalid trajectory input
 */
void time_to_speed(const std::vector<double>& downtrack, const std::vector<double>& times, double initial_speed,
                   std::vector<double>* speeds);

/**
 * \brief Converts a list of times to a list of speeds for the corresponding downtrack points using constant jerk equations
 *
 * \param downtracks The downtrack points where each speed and time will be
 * \param times The time at each downtrack point. Must have the same length as the downtracks list
 * \param initial_speed
 * \param speeds Output parameter which points to the vector which will store the speeds at each point.
 *              The first speed will always be initial_speed
 * \param decel_jerk The decelerating constant jerk used in calculation
 */
void time_to_speed_constjerk(const std::vector<double>& downtracks, const std::vector<double>& times, double initial_speed,
                   std::vector<double>* speeds , double decel_jerk);

/**
 * \brief Converts the trajectory points of a TrajectoryPlan message into equal sized vectors of downtrack distance and time
 *
 * \param traj_points The trajectory points to convert
 * \param downtrack Output parameter which points to the vector which will store the resulting downtracks.
 * \param times Output parameter which points to the vector which will store the resulting downtracks
 *
 */
void trajectory_to_downtrack_time(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& traj_points,
                                  std::vector<double>* downtrack, std::vector<double>* times);
}  // namespace conversions
}  // namespace trajectory_utils
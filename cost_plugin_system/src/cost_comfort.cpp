/*
 * Copyright (C) 2020 LEIDOS.
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

#include <cmath>
#include "cost_utils.hpp"
#include "cost_comfort.hpp"

namespace cost_plugin_system
{

CostofComfort::CostofComfort(double max_deceleration)
{
    max_deceleration_ = max_deceleration;
}

double CostofComfort::compute_cost(cav_msgs::ManeuverPlan plan) const
{
    double cost = 0.0;
    int maneuver_size = sizeof(plan.maneuvers);
    for (auto it = plan.maneuvers.begin(); it != plan.maneuvers.end(); it++)
    {
        double average_acceleration = abs((cost_utils::get_maneuver_start_speed(*it) - cost_utils::get_maneuver_end_speed(*it)) /
                                          (cost_utils::get_maneuver_end_time(*it) - cost_utils::get_maneuver_start_time(*it)));
        cost += average_acceleration;

        // If there is a lane change, add 1.0 as the cost
        if (cost_utils::get_maneuver_starting_lane_id.compare(cost_utils::get_maneuver_ending_lane_id) != 0)
        {
            cost += 1.0;
        }
    }
    return normalize_cost(cost, maneuver_size);
}

double normalize_cost(double cost, double size) const
{
    return cost / ((abs(max_deceleration_) + 1.0) * size);
}
} // namespace cost_plugin_system
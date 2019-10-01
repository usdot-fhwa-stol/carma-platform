/*
 * Copyright (C) 2019 LEIDOS.
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

#include "beam_search_strategy.hpp"

namespace arbitrator
{
    std::vector<cav_msgs::ManeuverPlan, double> BeamSearchStrategy::prioritize_plans(std::map<cav_msgs::ManeuverPlan, double> plans) const
    {
        std::sort(plans.begin(), 
            plans.end(), 
            [this] (cav_msgs::ManeuverPlan a, cav_msgs::ManeuverPlan b) 
            {
                return cost_function_.compute_cost_per_unit_distance(a) < cost_function_.compute_cost_per_unit_distance(b); 
            }
        );

        plans.resize(beam_width_);
        
        return plans;
    }
}
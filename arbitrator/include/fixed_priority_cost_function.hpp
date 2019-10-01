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

#ifndef __FIXED_PRIORITY_COST_FUNCTION_HPP__
#define __FIXED_PRIORITY_COST_FUNCTION_HPP__

#include "cost_function.hpp"
#include <map>
#include <string>

namespace arbitrator
{
    class FixedPriorityCostFunction : CostFunction
    {
        public:
            FixedPriorityCostFunction(std::map<std::string, double> plugin_priorities): 
                plugin_priorities_(plugin_priorities) {};
            double compute_total_cost(cav_msgs::ManeuverPlan plan) const;
            double compute_cost_per_unit_distance(cav_msgs::ManeuverPlan plan) const;
        private:
            std::map<std::string, double> plugin_priorities_;
    };
};

#endif //__FIXED_PRIORITY_COST_FUNCTION_HPP__

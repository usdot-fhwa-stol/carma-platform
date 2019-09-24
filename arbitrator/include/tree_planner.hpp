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

#ifndef __TREE_PLANNER_HPP__
#define __TREE_PLANNER_HPP__

#include <cav_msgs/ManeuverPlan.h>
#include "planning_strategy.hpp"
#include "cost_function.hpp"
#include "neighbor_generator.hpp"
#include "search_strategy.hpp"

namespace arbitrator
{
    class TreePlanner : PlanningStrategy
    {
        public:
            TreePlanner(const CostFunction &cf, const NeighborGenerator &ng, const SearchStrategy &ss):
                cost_function_(cf),
                neighbor_generator_(ng),
                search_strategy_(ss) {};
            cav_msgs::ManeuverPlan generate_plan();
        protected:
            const CostFunction &cost_function_;
            const NeighborGenerator &neighbor_generator_;
            const SearchStrategy &search_strategy_;

    };
};

#endif //__TREE_PLANNER_HPP__
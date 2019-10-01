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

#include "tree_planner.hpp"
#include "arbitrator_utils.hpp"
#include <vector>

namespace arbitrator
{
    cav_msgs::ManeuverPlan TreePlanner::generate_plan()
    {
        cav_msgs::ManeuverPlan root;
        std::vector<cav_msgs::ManeuverPlan> open_list;
        open_list.push_back(root);

        cav_msgs::ManeuverPlan longest_plan = root; // Track longest plan in case target length is never reached
        ros::Duration longest_plan_duration = ros::Duration(0);

        while (!open_list.empty())
        {
            // Pop the first element off the open list
            cav_msgs::ManeuverPlan cur_plan = open_list[0];
            open_list.erase(open_list.begin());


            // Evaluate terminal condition
            ros::Duration plan_duration = get_plan_end_time(cur_plan) - get_plan_start_time(cur_plan);
            if (plan_duration >= target_plan_duration) 
            {
                return cur_plan;
            } else if (plan_duration > longest_plan_duration) {
                longest_plan_duration = plan_duration;
                longest_plan = cur_plan;
            }

            // Expand it, and reprioritize
            std::vector<cav_msgs::ManeuverPlan> children = neighbor_generator_.generate_neighbors(cur_plan);
            std::vector<cav_msgs::ManeuverPlan> open_list = search_strategy_.prioritize_plans(children);
        }

        // If no perfect match is found, return the longest plan that fit the criteria
        return longest_plan;
    }
}
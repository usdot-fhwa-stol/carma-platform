/*
 * Copyright (C) 2022 LEIDOS.
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
#include <map>
#include <limits>

namespace arbitrator
{
    carma_planning_msgs::msg::ManeuverPlan TreePlanner::generate_plan(const VehicleState& start_state)
    {
        carma_planning_msgs::msg::ManeuverPlan root;
        std::vector<std::pair<carma_planning_msgs::msg::ManeuverPlan, double>> open_list_to_evaluate;
        std::vector<std::pair<carma_planning_msgs::msg::ManeuverPlan, double>> final_open_list;

        const double INF = std::numeric_limits<double>::infinity();
        open_list_to_evaluate.push_back(std::make_pair(root, INF));

        carma_planning_msgs::msg::ManeuverPlan longest_plan = root; // Track longest plan in case target length is never reached
        rclcpp::Duration longest_plan_duration = rclcpp::Duration::from_nanoseconds(0);


        while (!open_list_to_evaluate.empty())
        {
            std::vector<std::pair<carma_planning_msgs::msg::ManeuverPlan, double>> temp_open_list;

            for (auto it = open_list_to_evaluate.begin(); it != open_list_to_evaluate.end(); it++)
            {
                // Pop the first element off the open list
                auto& cur_plan = it->first;

                if (cur_plan.maneuver_plan_id == "")
                {
                    // Generate a new unique id for the maneuver plan
                    cur_plan.maneuver_plan_id =
                        boost::uuids::to_string(boost::uuids::random_generator()());
                }

                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"),
                    "Start printing newly requested maneuver plan for debugging:");

                for (auto mvr : cur_plan.maneuvers)
                {
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"),
                        "Maneuver: "<< maneuver_type_to_string(mvr.type));
                }

                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"),
                    "Ended printing newly requested maneuver plan for debugging");

                auto plan_duration = rclcpp::Duration(0, 0); // zero duration

                // If we're not at the root, plan_duration is nonzero (our plan should have maneuvers)
                if (!cur_plan.maneuvers.empty())
                {
                    // get plan duration
                    plan_duration = arbitrator_utils::get_plan_end_time(cur_plan) - arbitrator_utils::get_plan_start_time(cur_plan);
                }

                // save longest if none of the plans have enough target duration
                if (plan_duration > longest_plan_duration)
                {
                    longest_plan_duration = plan_duration;
                    longest_plan = cur_plan;
                }

                // Evaluate plan_duration is sufficient do not expand more
                if (plan_duration >= target_plan_duration_)
                {
                    final_open_list.push_back((*it));
                    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"), "Has enough duration, skipping that which has following mvrs..:");
                    for (auto mvr : it->first.maneuvers)
                    {
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"),
                            "Printing successful mvr: " << maneuver_type_to_string(mvr.type));
                    }
                    continue;
                }

                // Expand it, and reprioritize
                std::vector<carma_planning_msgs::msg::ManeuverPlan> children = neighbor_generator_->generate_neighbors(cur_plan, start_state);
                RCLCPP_DEBUG_STREAM(
                    rclcpp::get_logger("arbitrator"),
                    "Arbitrator received total of " << children.size()
                    << " response from strategic_plugins for plan_id: "
                    << std::string(cur_plan.maneuver_plan_id)
                );
                // Compute cost for each child and store in open list
                for (auto child = children.begin(); child != children.end(); child++)
                {
                    if (child->maneuvers.empty())
                    {
                        RCLCPP_DEBUG_STREAM(
                            rclcpp::get_logger("arbitrator"),
                            "Arbitrator didn't get successful maneuver plan for plan_id: "
                            << std::string(cur_plan.maneuver_plan_id)
                            <<", from one of the strategic plugin, which so far had maneuvers: "
                        );
                        for (auto mvr : cur_plan.maneuvers)
                        {
                            RCLCPP_DEBUG_STREAM(
                                rclcpp::get_logger("arbitrator"),
                                "Maneuver type: " << maneuver_type_to_string(mvr.type);
                            );
                        }
                        continue;
                    }

                    temp_open_list.push_back(std::make_pair(*child, cost_function_->compute_cost_per_unit_distance(*child)));
                }
            }

            open_list_to_evaluate = temp_open_list;
        }

        if (final_open_list.empty())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("arbitrator"), "None of the strategic plugins generated any valid plans! Please check if any is turned and returning valid maneuvers...");
            throw std::runtime_error("None of the strategic plugins generated any valid plans! Please check if any is turned and returning valid maneuvers...");
        }

        final_open_list = search_strategy_->prioritize_plans(final_open_list);

        // now every plan has enough duration if possible and prioritized
        for (auto pair : final_open_list)
        {
            // Pop the first element off the open list
            carma_planning_msgs::msg::ManeuverPlan cur_plan = pair.first;
            rclcpp::Duration plan_duration(0,0); // zero duration

            // get plan duration
            plan_duration = arbitrator_utils::get_plan_end_time(cur_plan) - arbitrator_utils::get_plan_start_time(cur_plan);

            // Evaluate plan_duration is sufficient do not expand more
            if (plan_duration >= target_plan_duration_)
            {
                return cur_plan;
            }
        }

        // If no perfect match is found, return the longest plan that fit the criteria
        return longest_plan;
    }
}

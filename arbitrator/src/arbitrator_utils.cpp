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

#include "arbitrator_utils.hpp"
#include <carma_planning_msgs/msg/maneuver.hpp>
#include <exception>


namespace arbitrator_utils
{
    rclcpp::Time get_plan_end_time(const carma_planning_msgs::msg::ManeuverPlan &plan) 
    {
        if (plan.maneuvers.empty())
        {
            throw std::invalid_argument("arbitrator::get_plan_end_time called on empty maneuver plan");
        }

        carma_planning_msgs::msg::Maneuver m = plan.maneuvers.back();

        return get_maneuver_end_time(m);
    }

    double get_plan_end_distance(const carma_planning_msgs::msg::ManeuverPlan &plan)
    {
        if (plan.maneuvers.empty())
        {
            throw std::invalid_argument("arbitrator::get_plan_end_dist called on empty maneuver plan");
        }

        carma_planning_msgs::msg::Maneuver m = plan.maneuvers.back();
        return get_maneuver_end_distance(m);
    }

    rclcpp::Time get_plan_start_time(const carma_planning_msgs::msg::ManeuverPlan &plan) 
    {
        if (plan.maneuvers.empty())
        {
            throw std::invalid_argument("arbitrator::get_plan_start_time called on empty maneuver plan");
        }

        carma_planning_msgs::msg::Maneuver m = plan.maneuvers.front();
        return get_maneuver_start_time(m);
    }

    double get_plan_start_distance(const carma_planning_msgs::msg::ManeuverPlan &plan)
    {
        if (plan.maneuvers.empty())
        {
            throw std::invalid_argument("arbitrator::get_plan_start_dist called on empty maneuver plan");
        }

        carma_planning_msgs::msg::Maneuver m = plan.maneuvers.front();
        return get_maneuver_start_distance(m);
    }

    rclcpp::Time get_maneuver_end_time(const carma_planning_msgs::msg::Maneuver &mvr) 
    {
        return GET_MANEUVER_PROPERTY(mvr, end_time);
    }

    rclcpp::Time get_maneuver_start_time(const carma_planning_msgs::msg::Maneuver &mvr) 
    {
        return GET_MANEUVER_PROPERTY(mvr, start_time);
    }

    double get_maneuver_end_distance(const carma_planning_msgs::msg::Maneuver &mvr) 
    {
        return GET_MANEUVER_PROPERTY(mvr, end_dist);
    }

    double get_maneuver_start_distance(const carma_planning_msgs::msg::Maneuver &mvr) 
    {
        return GET_MANEUVER_PROPERTY(mvr, start_dist);
    }
} // namespace arbitrator_utils

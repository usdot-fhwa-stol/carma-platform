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

#include "arbitrator_utils.hpp"
#include <cav_msgs/Maneuver.h>
#include <exception>

namespace arbitrator
{
    ros::Time get_plan_end_time(const cav_msgs::ManeuverPlan plan) 
    {
        if (plan.maneuvers.empty())
        {
            throw new std::invalid_argument("arbitrator::get_plan_end_time called on empty maneuver plan");
        }

        cav_msgs::Maneuver m = *plan.maneuvers.end();
        switch (m.type)
        {
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                return m.intersection_transit_left_turn_maneuver.end_time;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                return m.intersection_transit_right_turn_maneuver.end_time;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                return m.intersection_transit_straight_maneuver.end_time;
                break;
            case cav_msgs::Maneuver::LANE_CHANGE:
                return m.lane_change_maneuver.end_time;
                break;
            case cav_msgs::Maneuver::LANE_FOLLOWING:
                return m.lane_following_maneuver.end_time;
                break;
            default:
                throw new std::invalid_argument("arbitrator::get_plan_end_time called on maneuver list containing invalid maneuver type id");
        }
    }

    double get_plan_end_distance(const cav_msgs::ManeuverPlan plan)
    {
        if (plan.maneuvers.empty())
        {
            throw new std::invalid_argument("arbitrator::get_plan_end_dist called on empty maneuver plan");
        }

        cav_msgs::Maneuver m = *plan.maneuvers.end();
        switch (m.type)
        {
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                return m.intersection_transit_left_turn_maneuver.end_dist;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                return m.intersection_transit_right_turn_maneuver.end_dist;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                return m.intersection_transit_straight_maneuver.end_dist;
                break;
            case cav_msgs::Maneuver::LANE_CHANGE:
                return m.lane_change_maneuver.end_dist;
                break;
            case cav_msgs::Maneuver::LANE_FOLLOWING:
                return m.lane_following_maneuver.end_dist;
                break;
            default:
                throw new std::invalid_argument("arbitrator::get_plan_end_dist called on maneuver list containing invalid maneuver type id");
        }
    }

    ros::Time get_plan_start_time(const cav_msgs::ManeuverPlan plan) 
    {
        if (plan.maneuvers.empty())
        {
            throw new std::invalid_argument("arbitrator::get_plan_start_time called on empty maneuver plan");
        }

        cav_msgs::Maneuver m = *plan.maneuvers.begin();
        switch (m.type)
        {
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                return m.intersection_transit_left_turn_maneuver.start_time;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                return m.intersection_transit_right_turn_maneuver.start_time;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                return m.intersection_transit_straight_maneuver.start_time;
                break;
            case cav_msgs::Maneuver::LANE_CHANGE:
                return m.lane_change_maneuver.start_time;
                break;
            case cav_msgs::Maneuver::LANE_FOLLOWING:
                return m.lane_following_maneuver.start_time;
                break;
            default:
                throw new std::invalid_argument("arbitrator::get_plan_start_time called on maneuver list containing invalid maneuver type id");
        }
    }

    double get_plan_start_distance(const cav_msgs::ManeuverPlan plan)
    {
        if (plan.maneuvers.empty())
        {
            throw new std::invalid_argument("arbitrator::get_plan_start_dist called on empty maneuver plan");
        }

        cav_msgs::Maneuver m = *plan.maneuvers.begin();
        switch (m.type)
        {
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
                return m.intersection_transit_left_turn_maneuver.start_dist;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
                return m.intersection_transit_right_turn_maneuver.start_dist;
                break;
            case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
                return m.intersection_transit_straight_maneuver.start_dist;
                break;
            case cav_msgs::Maneuver::LANE_CHANGE:
                return m.lane_change_maneuver.start_dist;
                break;
            case cav_msgs::Maneuver::LANE_FOLLOWING:
                return m.lane_following_maneuver.start_dist;
                break;
            default:
                throw new std::invalid_argument("arbitrator::get_plan_start_dist called on maneuver list containing invalid maneuver type id");
        }
    }
} // namespace arbitrator

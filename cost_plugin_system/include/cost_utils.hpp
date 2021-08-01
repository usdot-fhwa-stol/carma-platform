/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include <string>
#include <ros/ros.h>
#include <cav_msgs/ManeuverPlan.h>

/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver typees
 * 
 * TODO: Implement a better system for handling Maneuver objects such that this
 *       macro isn't needed.
 * 
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types. Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evalutes to the desired field
 */
#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                    ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property :\
                        ((mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                        ((mvr).type == cav_msgs::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property :\
                            throw std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id"))))))))

namespace cost_utils
{
    /**
     * \brief Get the start time of the first maneuver in the plan
     * \param plan The plan to examine
     * \return The ros::Time at which it starts
     * \throws An invalid argument exception if the plan is empty
     */
    ros::Time get_plan_start_time(const cav_msgs::ManeuverPlan&);

    /**
     * \brief Get the start distance of the first maneuver in the plan
     * \param plan The plan to examine
     * \return The double-valued linear downtrack distance in meters to the start of the plan
     * \throws An invalid argument exception if the plan is empty
     */
    double get_plan_start_distance(const cav_msgs::ManeuverPlan&);

    /**
     * \brief Get the end time of the first maneuver in the plan
     * \param plan The plan to examine
     * \return The ros::Time at which it ends
     * \throws An invalid argument exception if the plan is empty
     */
    ros::Time get_plan_end_time(const cav_msgs::ManeuverPlan&);

    /**
     * \brief Get the end distance of the first maneuver in the plan
     * \param plan The plan to examine
     * \return The double-valued linear downtrack distance in meters to the end of the plan
     * \throws An invalid argument exception if the plan is empty
     */
    double get_plan_end_distance(const cav_msgs::ManeuverPlan&);

    /**
     * \brief Get the start time of the specified maneuver
     * \param mvr The maneuver to examine
     * \return The ros::Time at which it starts
     * \throws An invalid argument exception if the maneuver is poorly constructed
     */
    ros::Time get_maneuver_start_time(const cav_msgs::Maneuver&);

    /**
     * \brief Get the start distance the specified maneuver
     * \param mvr The maneuver to examine
     * \return The double-valued linear downtrack distance in meters to the start of the maneuver
     * \throws An invalid argument exception if the maneuver is poorly constructed
     */
    double get_maneuver_start_distance(const cav_msgs::Maneuver&);
    
    /**
     * \brief Get the end time of the specified maneuver
     * \param mvr The maneuver to examine
     * \return The ros::Time at which it ends
     * \throws An invalid argument exception if the maneuver is poorly constructed
     */
    ros::Time get_maneuver_end_time(const cav_msgs::Maneuver&);

    /**
     * \brief Get the end distance of the specified maneuver
     * \param mvr The maneuver to examine
     * \return The double-valued linear downtrack distance in meters to the end of the maneuver
     * \throws An invalid argument exception if the maneuver is poorly constructed
     */
    double get_maneuver_end_distance(const cav_msgs::Maneuver&);

    /**
     * \brief Get the start speed of the specified maneuver
     * \param mvr The maneuver to examine
     * \return The double-valued speed in meter/second to the start of the maneuver
     * \throws An invalid argument exception if the maneuver is poorly constructed
     */
    double get_maneuver_start_speed(const cav_msgs::Maneuver&);

    /**
     * \brief Get the end speed of the specified maneuver
     * \param mvr The maneuver to examine
     * \return The double-valued speed in meter/second to the end of the maneuver
     * \throws An invalid argument exception if the maneuver is poorly constructed
     */
    double get_maneuver_end_speed(const cav_msgs::Maneuver&);

    /**
     * \brief Get the starting lane of the specified maneuver
     * \param mvr The maneuver to examine
     * \return The string-valued starting lane id of the maneuver
     * \throws An invalid argument exception if the maneuver is poorly constructed
     */
    std::string get_maneuver_starting_lane_id(const cav_msgs::Maneuver&);

    /**
     * \brief Get the ending lane of the specified maneuver
     * \param mvr The maneuver to examine
     * \return The string-valued ending lane id of the maneuver
     * \throws An invalid argument exception if the maneuver is poorly constructed
     */
    std::string get_maneuver_ending_lane_id(const cav_msgs::Maneuver&);
} // namespace arbitrator

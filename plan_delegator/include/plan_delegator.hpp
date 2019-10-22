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

#ifndef __PLAN_DELEGATOR_HPP__
#define __PLAN_DELEGATOR_HPP__

/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver typees
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
                            throw new std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id")))))))

namespace plan_delegator
{
    /**
     * Mock plan delegator shell for initial testing.
     */
    class PlanDelegator
    {
        public:
            /**
             * \brief Initialize the plan delegator and run it
             * \return Standard unix program exit code
             */
            int run();
    };
}

#endif //__PLAN_DELEGATOR_HPP__
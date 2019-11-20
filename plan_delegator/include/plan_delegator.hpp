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

#ifndef PLAN_DELEGATOR_INCLUDE_PLAN_DELEGATOR_HPP_
#define PLAN_DELEGATOR_INCLUDE_PLAN_DELEGATOR_HPP_

#include <unordered_map>
#include <ros/ros.h>
#include <cav_msgs/ManeuverPlan.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_utils/CARMAUtils.h>

// TODO Replace this Macro if possible
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
    class PlanDelegator
    {
        public:

            /**
             * \brief Initialize the plan delegator
             */
            void init();

            /**
             * \brief Run the main thread of plan delegator
             */
            void run();

        private:

            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;

            ros::Publisher traj_pub_;

            std::string planning_topic_prefix_;
            std::string planning_topic_suffix_;

            std::unordered_map<std::string, ros::ServiceClient> trajectory_planners_;

            void ManeuverPlanCallback(const cav_msgs::ManeuverPlanConstPtr& plan);

            ros::ServiceClient& GetPlannerClientByName(const std::string& planner_name);
            
    };
}
#endif // PLAN_DELEGATOR_INCLUDE_PLAN_DELEGATOR_HPP_
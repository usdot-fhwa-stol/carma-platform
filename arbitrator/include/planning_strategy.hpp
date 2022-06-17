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

#ifndef __ARBITRATOR_INCLUDE_PLANNING_STRATEGY_HPP__
#define __ARBITRATOR_INCLUDE_PLANNING_STRATEGY_HPP__

#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include "vehicle_state.hpp"

namespace arbitrator
{

    /**
     * \brief Generic interface representing a strategy for arriving at a maneuver
     * plan
     */
    class PlanningStrategy
    {
        public:
            /**
             * \brief Generate a plausible maneuver plan
             * 
             * \param start_state The starting state of the vehicle to plan for
             * 
             * \return A maneuver plan from the vehicle's current state
             */
            virtual carma_planning_msgs::msg::ManeuverPlan generate_plan(const VehicleState& start_state) = 0;

            /**
             * \brief Virtual destructor provided for memory safety
             */
            virtual ~PlanningStrategy(){};
    };
};

#endif //__ARBITRATOR_INCLUDE_PLANNING_STRATEGY_HPP__
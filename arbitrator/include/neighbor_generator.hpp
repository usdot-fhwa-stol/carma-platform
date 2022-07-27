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

#ifndef __ARBITRATOR_INCLUDE_NEIGHBOR_GENERATOR_HPP__
#define __ARBITRATOR_INCLUDE_NEIGHBOR_GENERATOR_HPP__

#include <vector>
#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include "vehicle_state.hpp"

namespace arbitrator
{
    /**
     * Generic interface representing computation of children or neighbor nodes 
     * in a planning search graph
     */
    class NeighborGenerator
    {
        public:
            /**
             * \brief Generate the list of neighbors/children that a given node in the search graph
             *      expands to
             * \param plan The maneuver plan to expand upon
             * \param initial_state The initial state of the vehicle at the start of plan. This will be provided to planners for specific use when plan is empty
             *
             * \return A vector containing the new plans generated from it, if any
             */
            virtual std::vector<carma_planning_msgs::msg::ManeuverPlan> generate_neighbors(carma_planning_msgs::msg::ManeuverPlan plan, const VehicleState& initial_state) const = 0;

            /**
             * \brief Virtual destructor provided for memory safety
             */
            virtual ~NeighborGenerator(){};
    };
}

#endif //__ARBITRATOR_INCLUDE_NEIGHBOR_GENERATOR_HPP__
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

#ifndef __ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_TPP__ARBITRATOR_INCLUDE_
#define __ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_TPP__ARBITRATOR_INCLUDE_

#include "capabilities_interface.hpp"
#include "plugin_neighbor_generator.hpp"
#include <carma_planning_msgs/srv/plan_maneuvers.hpp>
#include <map>

namespace arbitrator
{   
    using PlanMvrRes = carma_planning_msgs::srv::PlanManeuvers::Response;
    using PlanMvrReq = carma_planning_msgs::srv::PlanManeuvers::Request;

    template <class T>
    std::vector<carma_planning_msgs::msg::ManeuverPlan> PluginNeighborGenerator<T>::generate_neighbors(carma_planning_msgs::msg::ManeuverPlan plan, const VehicleState& initial_state) const
    {
        auto msg = std::make_shared<PlanMvrReq>();
        // Set prior plan
        msg->prior_plan = plan;

        // Set vehicle state at prior plan start
        msg->header.frame_id = "map";
        msg->header.stamp = initial_state.stamp;
        msg->veh_x = initial_state.x;
        msg->veh_y = initial_state.y;
        msg->veh_downtrack = initial_state.downtrack;
        msg->veh_logitudinal_velocity = initial_state.velocity;
        msg->veh_lane_id = std::to_string(initial_state.lane_id);
        
        std::map<std::string, std::shared_ptr<PlanMvrRes>> res = ci_->template multiplex_service_call_for_capability<PlanMvrReq, PlanMvrRes>(CapabilitiesInterface::STRATEGIC_PLAN_CAPABILITY, msg);
        
        // Convert map to vector of map values
        std::vector<carma_planning_msgs::msg::ManeuverPlan> out;
        for (auto it = res.begin(); it != res.end(); it++)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"), "Pushing response of child: " << it->first << ", which had mvr size: " << it->second->new_plan.maneuvers.size());
            out.push_back(it->second->new_plan);
        }
        return out;
    }
}

#endif //__ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_TPP__ARBITRATOR_INCLUDE_

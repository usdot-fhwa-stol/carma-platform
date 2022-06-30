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

#include "cost_system_cost_function.hpp"
#include "arbitrator_utils.hpp"
#include <carma_planning_msgs/srv/compute_plan_cost.hpp>
#include <carma_planning_msgs/msg/maneuver_parameters.hpp>
#include <limits>

namespace arbitrator
{
    void CostSystemCostFunction::init(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh)
    {
        cost_system_sc_ = nh->create_client<carma_planning_msgs::srv::ComputePlanCost>("compute_plan_cost");
        initialized_ = true;
    }

    double CostSystemCostFunction::compute_total_cost(const carma_planning_msgs::msg::ManeuverPlan& plan)
    {
        if (!initialized_) {
            throw std::logic_error("Attempt to use CostSystemCostFunction before initialization.");
        }

        double total_cost = std::numeric_limits<double>::infinity();

        auto service_message = std::make_shared<carma_planning_msgs::srv::ComputePlanCost::Request>();
        service_message->maneuver_plan = plan;
        
        auto resp = cost_system_sc_->async_send_request(service_message);

        auto future_status = resp.wait_for(std::chrono::milliseconds(500));

        if (future_status == std::future_status::ready){
            total_cost = resp.get()->plan_cost;
        } else {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"), "Unable to get cost for plan from CostPluginSystem due to service call failure.");
        }

        return total_cost;
    }

    double CostSystemCostFunction::compute_cost_per_unit_distance(const carma_planning_msgs::msg::ManeuverPlan& plan)
    {
        double plan_dist = arbitrator_utils::get_plan_end_distance(plan) - arbitrator_utils::get_plan_start_distance(plan);
        return compute_total_cost(plan) / plan_dist;
    }
}


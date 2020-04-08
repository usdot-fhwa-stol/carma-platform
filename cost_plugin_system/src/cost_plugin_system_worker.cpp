/*
 * Copyright (C) 2018-2020 LEIDOS.
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
#include "cost_safety.hpp"
#include "cost_fuel.hpp"
#include "cost_legality.hpp"
#include "cost_comfort.hpp"
#include "cost_efficiency.hpp"
#include "cost_feasibility.hpp"
#include "cost_plugin_system_worker.hpp"

namespace cost_plugin_system
{

CostPluginSystemWorker::CostPluginSystemWorker()
{
    nh_ = ros::CARMANodeHandle{};
    pnh_ = ros::CARMANodeHandle{"~"};
}

int CostPluginSystemWorker::run()
{
    ROS_INFO("Initalizing cost_plugin_system node...");
    // Init our ROS objects
    compute_plan_cost_service_server_ = nh_.advertiseService("compute_plan_cost", &CostPluginSystemWorker::get_score, this);
    ROS_INFO("Ready to compute the total cost");
    ros::spin();

    return 0;
}

bool get_score(cav_srvs::ComputePlanCostRequest& req, cav_srvs::ComputePlanCostREsponse& res)
{
    cav_msgs::ManeuverPlan plan = req.maneuverPlan;

    res.plan_cost = compute_final_score(plan);

    return true;
}

double compute_final_score(cav_msgs::ManeuverPlan plan)
{
    double max_accelaration = pnh_.param<double>("max_accelaration", 5.0);
    double max_decelaration = pnh_.param<double>("max_decelaration", 8.0);

    double speed_limit = pnh_.param<double>("speed_limit", 27.0);
    double speed_buffer = pnh_.param<double>("speed_buffer", 25.0);

    double weight_of_comfort = pnh_.param<double>("weight_of_comfort", 1.0);
    double weight_of_efficiency = pnh_.param<double>("weight_of_efficiency", 1.0);
    double weight_of_feasibility = pnh_.param<double>("weight_of_feasibility", 1.0);
    double weight_of_fuel = pnh_.param<double>("weight_of_fuel", 1.0);
    double weight_of_safety = pnh_.param<double>("weight_of_safety", 1.0);

    cost_plugin_system::CostofComfort coc();
    cost_plugin_system::CostofEfficiency coe(speed_limit, speed_buffer);
    cost_plugin_system::CostofFeasibility cofe(max_accelaration, max_decelaration);
    cost_plugin_system::CostofFuel cof();
    cost_plugin_system::CostofLegality col();
    cost_plugin_system::CostofSafety cos(speed_limit);

    double total_cost = 0.0;
    if (col.compute_cost(plan) == 0)
    {
        double cost_of_comfort = coc.compute_cost(plan);
        double cost_of_efficiency = coe.compute_cost(plan);
        double cost_of_feasibility = cofe.compute_cost(plan);
        double cost_of_fuel = cof.compute_cost(plan);
        double cost_of_safety = cos.compute_cost(plan);

        total_cost = weight_of_comfort * cost_of_comfort + weight_of_efficiency * cost_of_efficiency +
                     weight_of_feasibility * cost_of_feasibility + weight_of_fuel * cost_of_fuel +
                     weight_of_safety * cost_of_safety;
    }
    else
    {
        total_cost = -999.0;
    }
    return total_cost;
}
} // namespace cost_plugin_system
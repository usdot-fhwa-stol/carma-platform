/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <map>
#include <string>
#include <carma_wm_ros2/WorldModel.h>
#include <carma_wm_ros2/WMListener.h>
#include "arbitrator.hpp"
#include "arbitrator_state_machine.hpp"
#include "cost_system_cost_function.hpp"
#include "fixed_priority_cost_function.hpp"
#include "plugin_neighbor_generator.hpp"
#include "beam_search_strategy.hpp"
#include "tree_planner.hpp"

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv, "arbitrator");
    rclcpp::CARMANodeHandle nh = rclcpp::CARMANodeHandle();
    rclcpp::CARMANodeHandle pnh = rclcpp::CARMANodeHandle("~");

    // Handle dependency injection
    arbitrator::CapabilitiesInterface ci{&nh};
    arbitrator::ArbitratorStateMachine sm;

    bool use_fixed_costs = false; 
    pnh.getParam("use_fixed_costs", use_fixed_costs);

    arbitrator::CostFunction *cf = nullptr;
    arbitrator::CostSystemCostFunction cscf = arbitrator::CostSystemCostFunction{};
    std::map<std::string, double> plugin_priorities;
    pnh.getParam("plugin_priorities", plugin_priorities);
    arbitrator::FixedPriorityCostFunction fpcf{plugin_priorities};
    if (use_fixed_costs) {
        cf = &fpcf;
    } else {
        cscf.init(nh);
        cf = &cscf;
    }

    int beam_width;
    pnh.param("beam_width", beam_width, 3);
    arbitrator::BeamSearchStrategy bss{beam_width};

    arbitrator::PluginNeighborGenerator<arbitrator::CapabilitiesInterface> png{ci};

    double target_plan;
    pnh.param("target_plan_duration", target_plan, 15.0);
    arbitrator::TreePlanner tp{*cf, png, bss, rclcpp::Duration(target_plan)};

    double min_plan_duration;
    pnh.param("min_plan_duration", min_plan_duration, 6.0);

    double planning_frequency;
    pnh.param("planning_frequency", planning_frequency, 1.0);

    carma_wm::WMListener wml;
    auto wm = wml.getWorldModel();

    arbitrator::Arbitrator arbitrator{
        &nh, 
        &pnh, 
        &sm, 
        &ci, 
        tp, 
        rclcpp::Duration(min_plan_duration),
        rclcpp::Rate(planning_frequency),
        wm };

    
    rclcpp::Subscriber twist_sub = nh.subscribe("current_velocity", 1, &arbitrator::Arbitrator::twist_cb, &arbitrator);

    arbitrator.initializeBumperTransformLookup();

    rclcpp::Timer bumper_pose_timer = nh.createTimer(
            rclcpp::Duration(rclcpp::Rate(10.0)),
            [&arbitrator](const auto&) {arbitrator.bumper_pose_cb();});

    arbitrator.run();

    return 0;
}

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
#ifndef __ARBITRATOR_INCLUDE_ARBITRATOR_NODE_HPP__
#define __ARBITRATOR_INCLUDE_ARBITRATOR_NODE_HPP__

#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <map>
#include <string>
#include <carma_wm_ros2/WorldModel.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <rapidjson/document.h>

#include "arbitrator.hpp"
#include "arbitrator_config.hpp"
#include "arbitrator_state_machine.hpp"
#include "cost_system_cost_function.hpp"
#include "fixed_priority_cost_function.hpp"
#include "plugin_neighbor_generator.hpp"
#include "beam_search_strategy.hpp"
#include "tree_planner.hpp"


namespace arbitrator 
{
    /**
     * An arbitrator node instance with currently used configuration/planning paradigms
     * 
     * Governs the interactions of plugins during the maneuver planning phase of
     * the CARMA planning process by internally using a worker class with generic planning interface.
     * 
     */
    class ArbitratorNode : public carma_ros2_utils::CarmaLifecycleNode
    {
        public:
            /**
             * @brief Constructor
             */
            explicit ArbitratorNode(const rclcpp::NodeOptions& options);
            
            ////
            // Overrides
            ////
            carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
            carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
        
        private:
            // helper function to parse plugin_priorities param from yaml as a json
            std::map<std::string, double> plugin_priorities_map_from_json(const std::string& json_string);

            Config config_;
             // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wm_listener_;
            carma_wm::WorldModelConstPtr wm_;
            std::shared_ptr<Arbitrator> arbitrator_;
            rclcpp::TimerBase::SharedPtr bumper_pose_timer_;
            rclcpp::TimerBase::SharedPtr arbitrator_run_;

    };

}   // namespace abitrator

#endif
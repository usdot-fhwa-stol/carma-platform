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

#include "arbitrator_node.hpp"
#include "yaml-cpp/yaml.h"

namespace arbitrator
{
    ArbitratorNode::ArbitratorNode(const rclcpp::NodeOptions& options) : carma_ros2_utils::CarmaLifecycleNode(options) 
    {
        // Create initial config
        config_ = Config();

        config_.min_plan_duration = declare_parameter<double>("min_plan_duration", config_.min_plan_duration);
        config_.target_plan_duration = declare_parameter<double>("target_plan_duration", config_.target_plan_duration);
        config_.planning_frequency = declare_parameter<double>("planning_frequency", config_.planning_frequency);
        config_.beam_width = declare_parameter<int>("beam_width", config_.beam_width);
        config_.use_fixed_costs = declare_parameter<bool>("use_fixed_costs", config_.use_fixed_costs);
        
        declare_parameter("plugin_priorities");
    }
    
    carma_ros2_utils::CallbackReturn ArbitratorNode::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        // Handle dependency injection
        arbitrator::CapabilitiesInterface ci(shared_from_this());
        arbitrator::ArbitratorStateMachine sm;
        
        // Reset config
        config_ = Config();
  
        get_parameter<double>("min_plan_duration", config_.min_plan_duration);
        get_parameter<double>("target_plan_duration", config_.target_plan_duration);
        get_parameter<double>("planning_frequency", config_.planning_frequency);
        get_parameter<int>("beam_width", config_.beam_width);
        get_parameter<bool>("use_fixed_costs", config_.use_fixed_costs);
        rclcpp::Parameter plugin_priorities_param = get_parameter("plugin_priorities");

        YAML::Node yaml_node = YAML::Load("plugin_priorities"); // TODO cricle back on map loading

        std::map<std::string, double> plugin_priorities;
        if (yaml_node.IsDefined())
        {
            plugin_priorities = yaml_node.as<std::map<std::string, double>>();
            RCLCPP_ERROR_STREAM(get_logger(), "HEY it worked!");
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Parameter mapping did not work");
        }

        
        arbitrator::CostFunction *cf = nullptr;
        arbitrator::CostSystemCostFunction cscf = arbitrator::CostSystemCostFunction();

        arbitrator::FixedPriorityCostFunction fpcf(plugin_priorities);
        if (config_.use_fixed_costs) {
            cf = &fpcf;
        } else {
            cscf.init(shared_from_this());
            cf = &cscf;
        }

        arbitrator::BeamSearchStrategy bss(config_.beam_width);

        arbitrator::PluginNeighborGenerator<arbitrator::CapabilitiesInterface> png{ci};
        arbitrator::TreePlanner tp(*cf, png, bss, rclcpp::Duration(config_.target_plan_duration* 1e9));

     
        auto wm_listener_ = std::make_shared<carma_wm::WMListener>(
            this->get_node_base_interface(), this->get_node_logging_interface(),
        this->get_node_topics_interface(), this->get_node_parameters_interface()
        );

        auto wm = wm_listener_->getWorldModel();

        arbitrator_ = std::make_shared<Arbitrator>(
            shared_from_this(),
            &sm, 
            &ci, 
            tp, 
            rclcpp::Duration(config_.min_plan_duration* 1e9),
            1/config_.planning_frequency,
            wm );

        carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 1, std::bind(&Arbitrator::twist_cb, arbitrator_.get(), std::placeholders::_1));

        arbitrator_->initializeBumperTransformLookup();

        return CallbackReturn::SUCCESS;
    }
    
    carma_ros2_utils::CallbackReturn ArbitratorNode::handle_on_activate(const rclcpp_lifecycle::State &)
    {
        bumper_pose_timer_ = create_timer(get_clock(),
                                std::chrono::milliseconds(100),
                                [this]() {this->arbitrator_->bumper_pose_cb();});
        
        std::thread run_arbitrator_worker(&Arbitrator::run, arbitrator_); // TODO does this actually work??

        return CallbackReturn::SUCCESS;
    }
};

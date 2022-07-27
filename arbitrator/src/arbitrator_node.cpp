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
        config_.plugin_priorities = plugin_priorities_map_from_json(declare_parameter<std::string>("plugin_priorities", ""));
    }
    
    carma_ros2_utils::CallbackReturn ArbitratorNode::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        // Handle dependency injection
        auto ci = std::make_shared<arbitrator::CapabilitiesInterface>(shared_from_this());
        arbitrator::ArbitratorStateMachine sm;
        
        // Reset config
        config_ = Config();
        
        get_parameter<double>("min_plan_duration", config_.min_plan_duration);
        get_parameter<double>("target_plan_duration", config_.target_plan_duration);
        get_parameter<double>("planning_frequency", config_.planning_frequency);
        get_parameter<int>("beam_width", config_.beam_width);
        get_parameter<bool>("use_fixed_costs", config_.use_fixed_costs);
        std::string json_string;
        get_parameter<std::string>("plugin_priorities", json_string);

        config_.plugin_priorities = plugin_priorities_map_from_json(json_string);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("arbitrator"), "Arbitrator Loaded Params: " << config_);

        std::shared_ptr<arbitrator::CostFunction> cf;
        arbitrator::CostSystemCostFunction cscf = arbitrator::CostSystemCostFunction();

        arbitrator::FixedPriorityCostFunction fpcf(config_.plugin_priorities);
        if (config_.use_fixed_costs) {
            cf = std::make_shared<arbitrator::FixedPriorityCostFunction>(fpcf);
        } else {
            cscf.init(shared_from_this());
            cf = std::make_shared<arbitrator::CostSystemCostFunction>(cscf);
        }

        auto bss = std::make_shared<arbitrator::BeamSearchStrategy>(config_.beam_width);

        auto png = std::make_shared<arbitrator::PluginNeighborGenerator<arbitrator::CapabilitiesInterface>>(ci);
        arbitrator::TreePlanner tp(cf, png, bss, rclcpp::Duration(config_.target_plan_duration* 1e9));
    
        wm_listener_ = std::make_shared<carma_wm::WMListener>(
            this->get_node_base_interface(), this->get_node_logging_interface(),
        this->get_node_topics_interface(), this->get_node_parameters_interface()
        );

        wm_ = wm_listener_->getWorldModel();

        arbitrator_ = std::make_shared<Arbitrator>(
            shared_from_this(),
            std::make_shared<ArbitratorStateMachine>(sm), 
            ci, 
            std::make_shared<TreePlanner>(tp), 
            rclcpp::Duration(config_.min_plan_duration* 1e9),
            1/config_.planning_frequency,
            wm_ );
        
        
        carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 1, std::bind(&Arbitrator::twist_cb, arbitrator_.get(), std::placeholders::_1));

        arbitrator_->initializeBumperTransformLookup();

        return CallbackReturn::SUCCESS;
    }
    
    carma_ros2_utils::CallbackReturn ArbitratorNode::handle_on_activate(const rclcpp_lifecycle::State &)
    {
        bumper_pose_timer_ = create_timer(get_clock(),
                                std::chrono::milliseconds(100),
                                [this]() {this->arbitrator_->bumper_pose_cb();});
        
        arbitrator_run_ = create_timer(get_clock(),
                                std::chrono::duration<double>(1/config_.planning_frequency),
                                [this]() {this->arbitrator_->run();});
        RCLCPP_INFO_STREAM(rclcpp::get_logger("arbitrator"), "Arbitrator started, beginning arbitrator state machine.");
        return CallbackReturn::SUCCESS;
    }

    std::map<std::string, double> ArbitratorNode::plugin_priorities_map_from_json(const std::string& json_string)
    {
        std::map<std::string, double> map;
    
        rapidjson::Document d;
        if(d.Parse(json_string.c_str()).HasParseError())
        {
            RCLCPP_WARN(rclcpp::get_logger("arbitrator"), "Failed to parse plugin_priorities map. Invalid json structure");
            return map;
        }
        if (!d.HasMember("plugin_priorities")) {
            RCLCPP_WARN(rclcpp::get_logger("arbitrator"), "No plugin_priorities found in arbitrator config");
            return map;
        }
        rapidjson::Value& map_value = d["plugin_priorities"];

        for (rapidjson::Value::ConstMemberIterator it = map_value.MemberBegin(); 
                                    it != map_value.MemberEnd(); it++) 
        {
            map[it->name.GetString()] = it->value.GetDouble();   
        }
        return map;

    }

};


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(arbitrator::ArbitratorNode)
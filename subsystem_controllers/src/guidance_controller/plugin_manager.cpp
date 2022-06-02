/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include "plugin_manager.h"

namespace subsystem_controllers
{

    PluginManager::PluginManager(const std::vector<std::string>& required_plugins,
                          const std::vector<std::string>& auto_activated_plugins, 
                          ros2_lifecycle_manager::Ros2LifecycleManager plugin_lifecycle_mgr) // TODO should we pass pointer instead?
        : required_plugins_(required_plugins.begin(), required_plugins_.end()),
            auto_activated_plugins_(auto_activated_plugins.begin(), auto_activated_plugins.end()),
            plugin_lifecycle_mgr_(plugin_lifecycle_mgr_)
    {}


    // Expected behavior

    void PluginManager::add_plugin(const Entry& plugin)
    {
        plugin_lifecycle_mgr_->add_managed_node(plugin.name); 

        em_.update_entry(plugin);

        Entry deactivated_entry = plugin;


        // TODO we don't actually have a current state, do we need a callback???
        // If this node is not in the active or inactive states then move the plugin to unconfigured
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE
            && get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
        {

            deactivated_entry.active_ = false;

            // Move plugin to unconfigured
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, plugin.name);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) 
            {                

                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name) != required_plugins_.end())
                {
                    throw std::runtime_error("Newly discovered required plugin " + plugin.name + " could not be brought to unconfigured state.")
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclccp::get_logger("subsystem_controllers", "Failed to cleanup newly discovered non-required plugin: " 
                    << plugin.name << " Marking as deactivated and unavailable!")); 
            
                deactivated_entry.available_ = false;

            }

            em_.update_entry(deactivated_entry);
            return;
        }
        
        
        // If the node is active or inactive then, 
        // when adding a plugin, it should be brought to the inactive state
        // We do not need transition it beyond inactive in this function as that will be managed by the plugin activation process via UI or parameters
        auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, plugin.name);

        if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
        {
            // If this plugin was required then trigger exception
            if (required_plugins_.find(plugin.name) != required_plugins_.end())
            {
                throw std::runtime_error("Newly discovered required plugin " + plugin.name + " could not be brought to inactive state.")
            }

            // If this plugin was not required log an error and mark it is unavailable and deactivated               
            RCLCPP_ERROR_STREAM(rclccp::get_logger("subsystem_controllers", "Failed to configure newly discovered non-required plugin: " 
                << plugin.name << " Marking as deactivated and unavailable!")); 

            deactivated_entry.available_ = false;
        }

        deactivated_entry.active = false;
        em_.update_entry(deactivated_entry);

    }

    bool configure()
    {
        std::vector<std::string> plugins_to_transition;
        plugins_to_transition.reserve(required_plugins_.size() + auto_activated_plugins_.size());
        plugins_to_transition.insert(plugins_to_transition.begin(), required_plugins_.begin(), required_plugins_.end());
        plugins_to_transition.insert(plugins_to_transition.begin(), auto_activated_plugins_.begin(), auto_activated_plugins_.end());


        auto failed_plugins = plugin_lifecycle_mgr_->configure(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms), false, plugins_to_transition);

        for (const auto& p : failed_plugins) 
        {
            if (required_plugins_.find(p) != required_plugins_.end())
            {
                throw std::runtime_error("Failed to trigger configure transition for required plugin: " + p);
            }

            if (auto_activated_plugins_.find(p) != auto_activated_plugins_.end()
                && )
            {

            }
        }


    }
    bool activate()
    {

    }
    bool deactivate(std::vector<std::string> plugins)
    {

    }
    bool cleanup(std::vector<std::string> plugins)
    {

    }
    bool shutdown(std::vector<std::string> plugins)
    {

    }

    void PluginManager::get_registered_plugins(carma_planning_msgs::srv::PluginListRequest&, carma_planning_msgs::srv::PluginListResponse& res)
    {
        std::vector<Entry> plugins = em_.get_entries();
        // convert to plugin list
        for(const auto& plugin :  plugin_map_)
        {
            carma_planning_msgs::msg::Plugin plugin;
            plugin.activated = i->active_;
            plugin.available = i->available_;
            plugin.name = i->name_;
            plugin.type = i->type_;
            res.plugins.push_back(plugin);
        }
    }

    void PluginManager::get_active_plugins(carma_planning_msgs::srv::PluginListRequest&, carma_planning_msgs::srv::PluginListResponse& res)
    {
        std::vector<Entry> plugins = em_.get_entries();
        // convert to plugin list
        for(auto i = plugins.begin(); i < plugins.end(); ++i)
        {
            if(i->active_)
            {
                carma_planning_msgs::msg::Plugin plugin;
                plugin.activated = i->active_;
                plugin.available = i->available_;
                plugin.name = i->name_;
                plugin.type = i->type_;
                res.plugins.push_back(plugin);
            }
        }
    }

    void PluginManager::activate_plugin(cav_srvs::PluginActivationRequest& req, cav_srvs::PluginActivationResponse& res)
    {
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(req.plugin_name);
        
        if(!requested_plugin) // If not a plugin then obviously not activated. Though really it would be better to have an indication of name failure in the message
        {
            res.newstate = false;
            return;
        }

        auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, req.plugin_name);

        bool activated = (result_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

        Entry updated_entry(requested_plugin->available_, activated, requested_plugin->name_, requested_plugin->type_, requested_plugin->capability_);
        em_.update_entry(updated_entry);

        res.newstate = activated;
    }

    void PluginManager::update_plugin_status(const carma_planning_msgs::msg::PluginConstPtr& msg)
    {
        ROS_DEBUG_STREAM("received status from: " << msg->name);
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(msg->name);

        if (!requested_plugin) // This is a new plugin so we need to add it
        {
            Entry plugin(msg->available_, msg->activated_, msg->name_, msg->type_, msg->capability_);
            add_plugin(plugin);
            return;
        }

        Entry plugin(msg->available, msg->activated_, msg->name, msg->type, msg->capability);

        em_.update_entry(plugin);
    }

    bool PluginManager::get_tactical_plugins_by_capability(carma_planning_msgs::srv::GetPluginApiRequest& req, carma_planning_msgs::srv::GetPluginApiResponse& res)
    {
        for(const auto& plugin : em_.get_entries())
        {
            if(plugin.type_ == carma_planning_msgs::msg::Plugin::TACTICAL &&
               (req.capability.size() == 0 || (plugin.capability_.compare(0, req.capability.size(), req.capability) == 0 && plugin.active_ && plugin.available_)))
            {
                res.plan_service.push_back(service_prefix_ + plugin.name_ + tactical_service_suffix_);
            }
        }
        return true;
    }

    bool PluginManager::get_strategic_plugins_by_capability(carma_planning_msgs::srv::GetPluginApiRequest& req, carma_planning_msgs::srv::GetPluginApiResponse& res)
    {
        for(const auto& plugin : em_.get_entries())
        {
            if(plugin.type_ == carma_planning_msgs::msg::Plugin::STRATEGIC && 
                (req.capability.size() == 0 || (plugin.capability_.compare(0, req.capability.size(), req.capability) == 0 && plugin.active_ && plugin.available_)))
            {
                ROS_DEBUG_STREAM("discovered strategic plugin: " << plugin.name_);
                res.plan_service.push_back(service_prefix_ + plugin.name_ + strategic_service_suffix_);
            }
            else
            {
                ROS_DEBUG_STREAM("not valid strategic plugin: " << plugin.name_);
            } 
        }
        return true;
    }

}

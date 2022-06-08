/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <boost/algorithm/string.hpp>
#include "plugin_manager.h"

namespace subsystem_controllers
{

    PluginManager::PluginManager(const std::vector<std::string>& required_plugins,
                          const std::vector<std::string>& auto_activated_plugins, 
                          std::shared_ptr<ros2_lifecycle_manager::LifecycleManagerInterface> plugin_lifecycle_mgr,
                          GetParentNodeStateFunc get_parent_state_func) // TODO should we pass pointer instead?
        : required_plugins_(required_plugins.begin(), required_plugins_.end()),
            auto_activated_plugins_(auto_activated_plugins.begin(), auto_activated_plugins.end()),
            plugin_lifecycle_mgr_(plugin_lifecycle_mgr_), get_parent_state_func_(get_parent_state_func)
    {
        // For all required and auto activated plugins add unknown entries but with 
        // user_requested_activation set to true.
        // This will be used later to determine how to transition the plugin specified by that entry
        for (const auto& p : required_plugins_) {
            Entry e(false, false, p.name, carma_planning_msgs::msg::Plugin::UNKNOWN, "", true);
            em_.update_entry(e);
            plugin_lifecycle_mgr_->add_managed_node(p.name);
        }

        for (const auto& p : auto_activated_plugins_) {
            Entry e(false, false, p.name, carma_planning_msgs::msg::Plugin::UNKNOWN, "", true);
            em_.update_entry(e);
            plugin_lifecycle_mgr_->add_managed_node(p.name);
        }

    }

    void PluginManager::add_plugin(const Entry& plugin)
    {
        plugin_lifecycle_mgr_->add_managed_node(plugin.name); 

        em_.update_entry(plugin);

        Entry deactivated_entry = plugin;

        auto parent_node_state = get_parent_state_func();

        // If this node is not in the active or inactive states then move the plugin to unconfigured
        if (parent_node_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE
            && parent_node_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
        {

            deactivated_entry.active_ = false;

            // Move plugin to match this nodes state
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(parent_node_state, plugin.name);

            if(result_state != parent_node_state) 
            {                

                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name) != required_plugins_.end())
                {
                    throw std::runtime_error("Newly discovered required plugin " + plugin.name + " could not be brought to controller node state.");
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclccp::get_logger("subsystem_controllers", "Failed to transition newly discovered non-required plugin: " 
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
                throw std::runtime_error("Newly discovered required plugin " + plugin.name + " could not be brought to inactive state.");
            }

            // If this plugin was not required log an error and mark it is unavailable and deactivated               
            RCLCPP_ERROR_STREAM(rclccp::get_logger("subsystem_controllers", "Failed to configure newly discovered non-required plugin: " 
                << plugin.name << " Marking as deactivated and unavailable!")); 

            deactivated_entry.available_ = false;
        }

        deactivated_entry.active = false;
        em_.update_entry(deactivated_entry);

    }

    void configure()
    {
        
        // Bring all known plugins to the inactive state
        for (auto plugin : em_.get_entries())
        {
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, plugin.name_);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
            {
                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Required plugin " + plugin.name_ + " could not be configured.");
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclccp::get_logger("subsystem_controllers", "Failed to configure newly discovered non-required plugin: " 
                    << plugin.name << " Marking as deactivated and unavailable!")); 

                Entry deactivated_entry = plugin;
                deactivated_entry.active = false;
                deactivated_entry.available = false;
                deactivated_entry.user_requested_activation_ = false; // TODO is there an edge case where the user hits this first????
                em_.update_entry(deactivated_entry);
            }

        }

    }
    
    void activate()
    {
        // Bring all required or auto activated plugins to the active state
        for (auto plugin : em_.get_entries())
        {
            // If this is not a plugin slated for activation then continue and leave up to user to activate manually later
            if (!plugin.user_requested_activation_)
                continue;
            
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, plugin.name_);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) 
            {
                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Required plugin " + plugin.name_ + " could not be activated.");
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclccp::get_logger("subsystem_controllers", "Failed to activate newly discovered non-required plugin: " 
                    << plugin.name << " Marking as deactivated and unavailable!")); 

                Entry deactivated_entry = plugin;
                deactivated_entry.active = false;
                deactivated_entry.available = false;
                deactivated_entry.user_requested_activation_ = false;
                em_.update_entry(deactivated_entry);
            }

            // If this was an auto activated plugin and not required then we only activate is once
            if (auto_activated_plugins_.find(plugin.name_) != auto_activated_plugins_.end())
            {
                plugin.user_requested_activation_ = false;
            }

            plugin.active = true; // Mark plugin as active
            
            em_.update_entry(deactivated_entry);

        }
    }
    
    void deactivate()
    {
        // Bring all required or auto activated plugins to the active state
        for (auto plugin : em_.get_entries())
        {
            
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, plugin.name_);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
            {
                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Required plugin " + plugin.name_ + " could not be deactivated.")
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclccp::get_logger("subsystem_controllers", "Failed to deactivate non-required plugin: " 
                    << plugin.name << " Marking as deactivated and unavailable!")); 

                Entry deactivated_entry = plugin;
                deactivated_entry.active = false;
                deactivated_entry.available = false;
                deactivated_entry.user_requested_activation_ = false;
                em_.update_entry(deactivated_entry);
            }

        }
    }
    
    void cleanup()
    {
        // Bring all required or auto activated plugins to the unconfigured state
        for (auto plugin : em_.get_entries())
        {
            
            auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, plugin.name_);

            if(result_state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) 
            {
                // If this plugin was required then trigger exception
                if (required_plugins_.find(plugin.name_) != required_plugins_.end())
                {
                    throw std::runtime_error("Required plugin " + plugin.name_ + " could not be cleaned up.")
                }

                // If this plugin was not required log an error and mark it is unavailable and deactivated               
                RCLCPP_ERROR_STREAM(rclccp::get_logger("subsystem_controllers", "Failed to cleanup non-required plugin: " 
                    << plugin.name << " Marking as deactivated and unavailable!")); 

                Entry deactivated_entry = plugin;
                deactivated_entry.active = false;
                deactivated_entry.available = false;
                deactivated_entry.user_requested_activation_ = false;
                em_.update_entry(deactivated_entry);
            }

        }
    }
    
    void shutdown()
    {
        bool success = plugin_lifecycle_mgr_.shutdown(std_msec(base_config_.service_timeout_ms), std_msec(base_config_.call_timeout_ms), false, em_.get_entries()).empty();

        if (success)
        {

            RCLCPP_INFO_STREAM(get_logger(), "Subsystem able to shutdown");
            return CallbackReturn::SUCCESS;
        }
        else
        {

            RCLCPP_INFO_STREAM(get_logger(), "Subsystem unable to shutdown cleanly");
            return CallbackReturn::FAILURE;
        }
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
        
        if(!requested_plugin) // If not a known plugin then obviously not activated. Though really it would be better to have an indication of name failure in the message
        {
            res.newstate = false;
            return;
        }

        auto result_state = plugin_lifecycle_mgr_->transition_node_to_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, req.plugin_name);

        bool activated = (result_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

        Entry updated_entry(requested_plugin->available_, activated, requested_plugin->name_, requested_plugin->type_, requested_plugin->capability_, true); // Mark as user activated
        em_.update_entry(updated_entry);

        res.newstate = activated;
    }

    void PluginManager::update_plugin_status(const carma_planning_msgs::msg::PluginConstPtr& msg)
    {
        ROS_DEBUG_STREAM("received status from: " << msg->name);
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(msg->name);

        if (!requested_plugin) // This is a new plugin so we need to add it
        {
            Entry plugin(msg->available_, msg->activated_, msg->name_, msg->type_, msg->capability_, false);
            add_plugin(plugin);
            return;
        }

        Entry plugin(msg->available, msg->activated_, msg->name, msg->type, msg->capability, requested_plugin->user_requested_activation_);

        em_.update_entry(plugin);
    }

    bool PluginManager::matching_capability(const std::vector<std::string>& base_capability_levels, const std::vector<std::string>& compared_capability_levels)
    {
        for (size_t i=0; i < base_capability_levels.size() &&  i < compared_capability_levels.size(); i++)
        {
            if (compared_capability_levels[i].compare(req.capability) != 0)
                return false;
        }

        return true;
    }

    void PluginManager::get_control_plugins_by_capability(carma_planning_msgs::srv::GetPluginApiRequest& req, carma_planning_msgs::srv::GetPluginApiResponse& res)
    {
        std::vector<std::string> req_capability_levels;
        boost::split(capability_levels, req.capability, boost::is_any_of("/"))

        for(const auto& plugin : em_.get_entries())
        {

            std::vector<std::string> plugin_capability_levels; 
            boost::split(plugin_capability_levels, plugin.capability_, boost::is_any_of("/"))

            if(plugin.type_ == carma_planning_msgs::msg::Plugin::CONTROL && 
                (req.capability.size() == 0 || (match_capability(plugin_capability_levels, req_capability_levels) == 0 && plugin.active_ && plugin.available_)))
            {
                ROS_DEBUG_STREAM("discovered control plugin: " << plugin.name_);
                res.plan_service.push_back(plugin.name_ + plugin.capability_);
            }
            else
            {
                ROS_DEBUG_STREAM("not valid control plugin: " << plugin.name_);
            } 
        }
    }

    void PluginManager::get_tactical_plugins_by_capability(carma_planning_msgs::srv::GetPluginApiRequest& req, carma_planning_msgs::srv::GetPluginApiResponse& res)
    {
        std::vector<std::string> req_capability_levels;
        boost::split(capability_levels, req.capability, boost::is_any_of("/"))

        for(const auto& plugin : em_.get_entries())
        {

            std::vector<std::string> plugin_capability_levels;
            boost::split(plugin_capability_levels, plugin.capability_, boost::is_any_of("/"))

            if(plugin.type_ == carma_planning_msgs::msg::Plugin::TACTICAL && 
                (req.capability.size() == 0 || (match_capability(plugin_capability_levels, req_capability_levels) == 0 && plugin.active_ && plugin.available_)))
            {
                ROS_DEBUG_STREAM("discovered tactical plugin: " << plugin.name_);
                res.plan_service.push_back(plugin.name_ + plugin.capability_);
            }
            else
            {
                ROS_DEBUG_STREAM("not valid tactical plugin: " << plugin.name_);
            } 
        }
    }

    void PluginManager::get_strategic_plugins_by_capability(carma_planning_msgs::srv::GetPluginApiRequest& req, carma_planning_msgs::srv::GetPluginApiResponse& res)
    {
        std::vector<std::string> req_capability_levels;
        boost::split(capability_levels, req.capability, boost::is_any_of("/"))

        for(const auto& plugin : em_.get_entries())
        {

            std::vector<std::string> plugin_capability_levels; 
            boost::split(plugin_capability_levels, plugin.capability_, boost::is_any_of("/"))

            if(plugin.type_ == carma_planning_msgs::msg::Plugin::STRATEGIC && 
                (req.capability.size() == 0 || (match_capability(plugin_capability_levels, req_capability_levels) == 0 && plugin.active_ && plugin.available_)))
            {
                ROS_DEBUG_STREAM("discovered strategic plugin: " << plugin.name_);
                res.plan_service.push_back(plugin.name_ + plugin.capability_);
            }
            else
            {
                ROS_DEBUG_STREAM("not valid strategic plugin: " << plugin.name_);
            } 
        }
    }

}

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
        : required_plugins_(required_plugins), auto_activated_plugins_(auto_activated_plugins), plugin_lifecycle_mgr_(plugin_lifecycle_mgr_)
    {}

    void add_plugin(const std::string plugin)
    {
        plugin_lifecycle_mgr_.add_managed_node(plugin); 
        // TODO need to bringup to inactive state here 
    }

    bool configure(std::vector<std::string> plugins)
    {
        
    }
    bool activate(std::vector<std::string> plugins)
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

    void PluginManager::get_registered_plugins(carma_planning_msgs::srv::PluginListResponse& res)
    {
        std::vector<Entry> plugins = em_.get_entries();
        // convert to plugin list
        for(auto i = plugins.begin(); i < plugins.end(); ++i)
        {
            carma_planning_msgs::msg::Plugin plugin;
            plugin.activated = i->active_;
            plugin.available = i->available_;
            plugin.name = i->name_;
            plugin.type = i->type_;
            res.plugins.push_back(plugin);
        }
    }

    void PluginManager::get_active_plugins(carma_planning_msgs::srv::PluginListResponse& res)
    {
        std::vector<Entry> plugins = em_.get_entries();
        // convert to plugin list
        for(auto i = plugins.begin(); i < plugins.end(); ++i)
        {
            if(i->active_)
            {
                carma_planning_msgs::msg::Plugin plugin;
                plugin.activated = true;
                plugin.available = i->available_;
                plugin.name = i->name_;
                plugin.type = i->type_;
                res.plugins.push_back(plugin);
            }
        }
    }

    bool PluginManager::activate_plugin(const std::string& name, const bool activate)
    {
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(name);
        if(requested_plugin)
        {
            // params: bool available, bool active, std::string name, long timestamp, uint8_t type
            Entry updated_entry(requested_plugin->available_, activate, requested_plugin->name_, 0, requested_plugin->type_, requested_plugin->capability_);
            em_.update_entry(updated_entry);
            return true;
        }
        return false;
    }

    void PluginManager::update_plugin_status(const carma_planning_msgs::msg::PluginConstPtr& msg)
    {
        ROS_DEBUG_STREAM("received status from: " << msg->name);
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(msg->name);
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry plugin(msg->available, msg->activated, msg->name, 0, msg->type, msg->capability);
        // if it already exists, we do not change its activation status
        if(requested_plugin)
        {
            plugin.active_ = requested_plugin->active_;
        } else if(em_.is_entry_required(msg->name))
        {
            plugin.active_ = true;
        }
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

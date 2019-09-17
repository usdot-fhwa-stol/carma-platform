/*
 * Copyright (C) 2019 LEIDOS.
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

namespace health_monitor
{

    PluginManager::PluginManager(std::vector<std::string> require_plugin_names)
    {
        this->required_plugin_names_ = require_plugin_names;
    }

    void PluginManager::get_registered_plugins(cav_srvs::PluginListResponse& res)
    {
        std::vector<Entry> plugins = em_.get_entries();
        for(size_t i = 0; i < plugins.size(); ++i)
        {
            cav_msgs::Plugin plugin;
            plugin.activated = plugins[i].active_;
            plugin.available = plugins[i].available_;
            plugin.name = plugins[i].name_;
            plugin.required = plugins[i].required_;
            plugin.type = plugins[i].type_;
            res.plugins.push_back(plugin);
        }
    }

    void PluginManager::get_active_plugins(cav_srvs::PluginListResponse& res)
    {
        std::vector<Entry> plugins = em_.get_entries();
        for(size_t i = 0; i < plugins.size(); ++i)
        {
            if(plugins[i].active_)
            {
                cav_msgs::Plugin plugin;
                plugin.activated = true;
                plugin.available = plugins[i].available_;
                plugin.name = plugins[i].name_;
                plugin.required = plugins[i].required_;
                plugin.type = plugins[i].type_;
                res.plugins.push_back(plugin);
            }
        }
    }

    bool PluginManager::activate_plugin(std::string name)
    {
        Entry* plugin_pointer = em_.get_entry_by_name(name);
        if(plugin_pointer != nullptr)
        {
            return plugin_pointer->active_ = true;
        }
        return false;
    }

    void PluginManager::update_plugin_status(const cav_msgs::PluginConstPtr& msg)
    {
        Entry* plugin_pointer = em_.get_entry_by_name(msg->name);
        // params: bool required, bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry plugin(msg->required, msg->available, msg->activated, msg->name, 0, msg->type);
        // if it already exists, we do not change its activation status
        if(plugin_pointer != nullptr)
        {
            plugin.active_ = plugin_pointer->active_;
        }
        em_.update_entry(plugin);
    }

    //TODO: Think about how to handle require plugins!

}
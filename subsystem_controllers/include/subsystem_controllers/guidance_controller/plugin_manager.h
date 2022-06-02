#pragma once

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

#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/srv/plugin_list_request.hpp>
#include <carma_planning_msgs/srv/plugin_list_response.hpp>
#include <carma_planning_msgs/srv/get_plugin_api.hpp>
#include "entry_manager.h"
#include <ros/console.h>
#include <sstream>


namespace subsystem_controllers
{

    struct Plugin
    {
        std::string name;
        long timestamp;
        std::string capability;

        Entry(const std::string& p_name, long p_timestamp, const std::string& p_capability)
         : name(p_name), timestamp(p_timestamp), capability(p_capability) {}
    };

    class PluginManager
    {
        public:
            
            /**
             * \brief Default constructor for PluginManager
             */
            PluginManager() = default;

            /**
             * \brief Constructor for PluginManager takes in require_plugin_names and service names
             */
            PluginManager(const std::vector<std::string>& required_plugins,
                          const std::vector<std::string>& auto_activated_plugins); // TODO take in pointe to lifecycle interface so we can unit test

            void add_plugin(const std::string plugin);

            bool configure(std::vector<std::string> plugins); 
            bool activate(std::vector<std::string> plugins); 
            bool deactivate(std::vector<std::string> plugins); 
            bool cleanup(std::vector<std::string> plugins); 
            bool shutdown(std::vector<std::string> plugins); 

            /**
             * \brief Update the status of a certain plugin
             */
            void update_plugin_status(const carma_planning_msgs::PluginConstPtr& msg);

            /**
             * \brief Get strategic plugins by capability
             */
            bool get_strategic_plugins_by_capability(carma_planning_msgs::GetPluginApiRequest& req, carma_planning_msgs::GetPluginApiResponse& res);

            /**
             * \brief Get tactical plugins by capability
             */
            bool get_tactical_plugins_by_capability(carma_planning_msgs::GetPluginApiRequest& req, carma_planning_msgs::GetPluginApiResponse& res);

        private:

            //! Lifecycle Manager which will track the plugin nodes and call their lifecycle services on request
            ros2_lifecycle_manager::Ros2LifecycleManager plugin_lifecycle_mgr_;

            std::unordered_map<std::string, Plugin> plugin_name_map_;
        
            

    };
}
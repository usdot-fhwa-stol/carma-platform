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
            PluginManager(const std::vector<std::string>& require_plugin_names,
                          const std::string& service_prefix,
                          const std::string& strategic_service_suffix,
                          const std::string& tactical_service_suffix);

            /**
             * \brief Get a list of registered plugins
             */
            void get_registered_plugins(carma_planning_msgs::PluginListResponse& res);

            /**
             * \brief Get a list of active plugins
             */
            void get_active_plugins(carma_planning_msgs::PluginListResponse& res);

            /**
             * \brief Activate or deactivate a certain plugin
             */
            bool activate_plugin(const std::string& name, const bool activate);

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
        
            std::string service_prefix_;
            std::string strategic_service_suffix_;
            std::string tactical_service_suffix_;
            EntryManager em_;

    };
}
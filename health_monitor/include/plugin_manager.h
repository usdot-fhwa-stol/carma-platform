#pragma once

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

#include <cav_msgs/Plugin.h>
#include <cav_srvs/PluginListResponse.h>
#include "entry_manager.h"

namespace health_monitor
{
    class PluginManager
    {
        public:
            
            /*!
             * \brief Default constructor for PluginManager
             */
            PluginManager(std::vector<std::string> require_plugin_names);

            /*!
             * \brief Get a list of registered plugins
             */
            void get_registered_plugins(cav_srvs::PluginListResponse& res);

            /**
             * \brief Get a list of active plugins
             */
            void get_active_plugins(cav_srvs::PluginListResponse& res);

            /**
             * \brief Activate a certain plugin
             */
            bool activate_plugin(std::string name);

            /**
             * \brief Update the status of a certain plugin
             */
            void update_plugin_status(const cav_msgs::PluginConstPtr& msg);

        private:

            EntryManager em_;
            std::vector<std::string> required_plugin_names_;
    };
}
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

#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PluginList.h>
#include <cav_srvs/PluginActivation.h>
#include <cav_msgs/Plugin.h>
#include <cav_msgs/DriverStatus.h>
#include "plugin_manager.h"
#include "driver_manager.h"

namespace health_monitor
{
    class HealthMonitor
    {
        public:
            
            /*!
             * \brief Default constructor for HealthMonitor
             */
            HealthMonitor();

            /*!
             * \brief Begin normal execution of health monitor node. Will take over control flow of program and exit from here.
             * 
             * \return The exit status of this program
             */
            void run();

        private:

            // node handles
            ros::CARMANodeHandle nh_, pnh_;

            // workers
            PluginManager plugin_manager_;
            DriverManager driver_manager_;

            // service servers 
            ros::ServiceServer registered_plugin_service_server_;
            ros::ServiceServer active_plugin_service_server_;
            ros::ServiceServer activate_plugin_service_server_;

            // topic subscribers
            ros::Subscriber plugin_discovery_subscriber_;
            ros::Subscriber driver_discovery_subscriber_;

            // message/service callbacks
            bool registered_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res);
            bool active_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res);
            bool activate_plugin_cb(cav_srvs::PluginActivationRequest& req, cav_srvs::PluginActivationResponse& res);
            void plugin_discovery_cb(cav_msgs::Plugin msg);
            void driver_discovery_cb(cav_msgs::DriverStatus msg);
    };
}
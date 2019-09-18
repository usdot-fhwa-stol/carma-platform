/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#pragma once

#include <string>
#include <ros/ros.h>
#include <atomic>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PluginList.h>
#include <cav_srvs/PluginActivation.h>
#include <cav_srvs/SetGuidanceActive.h>
#include <cav_srvs/SetEnableRobotic.h>
#include <cav_msgs/PluginList.h>
#include <std_msgs/Bool.h>
#include <cav_msgs/GuidanceState.h>
#include <cav_msgs/RobotEnabled.h>
#include "guidance/guidance_state_machine.hpp"

namespace guidance
{
    class GuidanceWorker
    {
        public:
            /*!
             * \brief Default constructor for GuidanceWorker
             */
            GuidanceWorker();

            /*!
             * \brief Begin normal execution of Guidance worker. Will take over control flow of program and exit from here.
             * 
             * \return The exit status of this program
             */
            int run();

        protected:
            // Message/service callbacks
            bool registered_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res);
            bool active_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res);
            bool activate_plugin_cb(cav_srvs::PluginActivationRequest& req, cav_srvs::PluginActivationResponse& res);
            bool guidance_acivation_cb(cav_srvs::SetGuidanceActiveRequest& req, cav_srvs::SetGuidanceActiveResponse& res);
            void robot_status_cb(const cav_msgs::RobotEnabledConstPtr& msg);
            void plugin_discovery_cb(cav_msgs::Plugin msg);
            void system_alert_cb(const cav_msgs::SystemAlertConstPtr& msg);

            // Service servers 
            ros::ServiceServer registered_plugin_service_server_;
            ros::ServiceServer active_plugin_service_server_;
            ros::ServiceServer activate_plugin_service_server_;
            ros::ServiceServer guidance_activate_service_server_;

            // Publishers
            ros::Publisher plugin_publisher_;
            ros::Publisher state_publisher_;
            ros::ServiceClient enable_client_;

            // Subscribers
            ros::Subscriber robot_status_subscriber_;
            ros::Subscriber plugin_discovery_subscriber_;

            // Node handles
            ros::CARMANodeHandle nh_, pnh_;

            // a list to keep track of plugin status
            std::vector<cav_msgs::Plugin> plugins;

            // required plugins
            std::vector<std::string> required_plugins;

            std::atomic<bool> guidance_activated_;
        
        private:
            GuidanceStateMachineFactory guidance_state_machine_factory;
            // Guidance state machine
            GuidanceStateMachine* gsm = guidance_state_machine_factory.createCadilacInstance();
            // Helper functions
            void process_required_plugin_list(std::vector<std::string> list);
            void populate_plugin_list_response(cav_srvs::PluginListResponse& res);
            void populate_active_plugin_list_response(cav_srvs::PluginListResponse& res);
            bool is_required_plugin(std::string plugin_name, std::string version);
            bool spin_cb();
    };
}
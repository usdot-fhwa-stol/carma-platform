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

#include "ui_integration/ui_integration_worker.hpp"

namespace ui_integration
{
    UIIntegrationWorker::UIIntegrationWorker() 
    {
        nh_ = ros::CARMANodeHandle{};
        pnh_ = ros::CARMANodeHandle{"~"};
    }

    void UIIntegrationWorker::populate_plugin_list_response(cav_srvs::PluginListResponse& res)
    {
        cav_msgs::Plugin p;
        p.activated = true;
        p.available = true;
        p.required = true;
        p.name = plugin_name_;
        p.versionId = plugin_version_;
        res.plugins.push_back(p);
    }

    bool UIIntegrationWorker::registered_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res)
    {
        populate_plugin_list_response(res);
        return true;
    }

    bool UIIntegrationWorker::active_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res)
    {
        populate_plugin_list_response(res);
        return true;
    }

    bool UIIntegrationWorker::activate_plugin_cb(cav_srvs::PluginActivationRequest& req, cav_srvs::PluginActivationResponse& res)
    {
        res.newState = true;
        return true;
    }

    bool UIIntegrationWorker::guidance_acivation_cb(cav_srvs::SetGuidanceActiveRequest& req, cav_srvs::SetGuidanceActiveResponse& res)
    {
        // Translate message type from GuidanceActiveRequest to SetEnableRobotic
        ROS_INFO_STREAM("Request for guidance activation recv'd with status " << req.guidance_active);
        cav_srvs::SetEnableRobotic srv;
        if (req.guidance_active) {
            srv.request.set = cav_srvs::SetEnableRobotic::Request::ENABLE;
        } else {
            srv.request.set = cav_srvs::SetEnableRobotic::Request::DISABLE;
        }

        enable_client_.call(srv);
        return true;
    }

    int UIIntegrationWorker::run() 
    {
        ROS_INFO("Initalizing UI integration node...");
        // Init our ROS objects
        registered_plugin_service_server_ = nh_.advertiseService("get_registered_plugins", &UIIntegrationWorker::registered_plugin_cb, this);
        active_plugin_service_server_ = nh_.advertiseService("get_active_plugins", &UIIntegrationWorker::active_plugin_cb, this);
        activate_plugin_service_server_ = nh_.advertiseService("activate_plugin", &UIIntegrationWorker::activate_plugin_cb, this);
        guidance_activate_service_server_ = nh_.advertiseService("set_guidance_active", &UIIntegrationWorker::guidance_acivation_cb, this);

        plugin_publisher_ = nh_.advertise<cav_msgs::PluginList>("available_plugins", 5, true);
        enable_client_ = nh_.serviceClient<cav_srvs::SetEnableRobotic>("controller/enable_robotic");


        // Load the spin rate param to determine how fast to process messages
        // Default rate 10.0 Hz
        double spin_rate = pnh_.param<double>("spin_rate_hz", 10.0);

        plugin_name_ = pnh_.param<std::string>("plugin_name", "Autoware Plugin");
        plugin_version_ = pnh_.param<std::string>("plugin_version", "1.0.0");

        ROS_INFO_STREAM("UI Integration node configured for plugin: " << plugin_name_ << ":" << plugin_version_);

        // Spin until system shutdown
        ROS_INFO_STREAM("UI Integration node initialized, spinning at " << spin_rate << "hz...");
        ros::CARMANodeHandle::setSpinRate(spin_rate);
        ros::CARMANodeHandle::spin();
    } 
}

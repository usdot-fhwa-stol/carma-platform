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

#include "guidance/guidance_worker.hpp"

namespace guidance
{
    GuidanceWorker::GuidanceWorker() 
    {
        nh_ = ros::CARMANodeHandle{};
        pnh_ = ros::CARMANodeHandle{"~"};
    }

    void GuidanceWorker::populate_plugin_list_response(cav_srvs::PluginListResponse& res)
    {
        for(int i = 0; i < plugins.size(); ++i)
        {
            res.plugins.push_back(plugins[i]);
        }
    }

    void GuidanceWorker::populate_active_plugin_list_response(cav_srvs::PluginListResponse& res)
    {
        for(int i = 0; i < plugins.size(); ++i)
        {
            if(plugins[i].activated)
            {
                res.plugins.push_back(plugins[i]);
            }
        }
    }

    bool GuidanceWorker::is_required_plugin(std::string plugin_name, std::string version)
    {
        for(int i = 0; i < required_plugins.size(); i++)
        {
            if(plugin_name.append(version).compare(required_plugins[i]) == 0)
            {
               return true;
            }
        }
        return false;
    }

    bool GuidanceWorker::registered_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res)
    {
        populate_plugin_list_response(res);
        return true;
    }

    bool GuidanceWorker::active_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res)
    {
        populate_active_plugin_list_response(res);
        return true;
    }

    bool GuidanceWorker::activate_plugin_cb(cav_srvs::PluginActivationRequest& req, cav_srvs::PluginActivationResponse& res)
    {
        for(int i = 0; i < plugins.size(); ++i)
        {
            // Go through the plugin list to find the corresponding plugin that the user wants to activate
            if(req.pluginName.compare(plugins[i].name) == 0 && req.pluginVersion.compare(plugins[i].versionId) == 0)
            {
                plugins[i].activated = req.activated;
                res.newState = plugins[i].activated;
                return true;
            }
        }
        return false;
    }

    void GuidanceWorker::system_alert_cb(const cav_msgs::SystemAlertConstPtr& msg)
    {
        gsm->onSystemAlert(msg);
    }

    void GuidanceWorker::plugin_discovery_cb(cav_msgs::Plugin msg)
    {
        // if plugin list does not have anything, add it to the list and return
        if(plugins.size() == 0)
        {
            // check if a plugin required when it is newly discovered
            msg.required = is_required_plugin(msg.name, msg.versionId);
            plugins.push_back(msg);
            return;
        }
        // if we have some entries in the list, check if the plugin is already in the record
        size_t index = 0;
        while(index < plugins.size())
        {
            // check version id to be safe, but in the normal case we do not allow same plugin with different version id existing together
            if(plugins[index].name.compare(msg.name) == 0 && plugins[index].versionId.compare(msg.versionId) == 0)
            {
                // only availability can be changed by plugin itself
                plugins[index].available = msg.available;
                // this break will make sure that "index" variable stays at the location of matching entry if it exists
                return;
            }
            ++index;
        }
        // index points to the end of plugin list without a matching entry
        if(index == plugins.size())
        {
            msg.required = is_required_plugin(msg.name, msg.versionId);
            plugins.push_back(msg);
        }
    }

    void GuidanceWorker::robot_status_cb(const cav_msgs::RobotEnabledConstPtr& msg)
    {
        gsm->onRoboticStatus(msg);
    }

    bool GuidanceWorker::guidance_acivation_cb(cav_srvs::SetGuidanceActiveRequest& req, cav_srvs::SetGuidanceActiveResponse& res)
    {
        // Translate message type from GuidanceActiveRequest to SetEnableRobotic
        ROS_INFO_STREAM("Request for guidance activation recv'd with status " << req.guidance_active);
        gsm->onSetGuidanceActive(req.guidance_active);
        cav_srvs::SetEnableRobotic srv;
        if (gsm->getCurrentState() == GuidanceStateMachine::ENGAGED) {
            srv.request.set = cav_srvs::SetEnableRobotic::Request::ENABLE;
            res.guidance_status = true;
        } else {
            srv.request.set = cav_srvs::SetEnableRobotic::Request::DISABLE;
            res.guidance_status = false;
        }
        enable_client_.call(srv);
        return true;
    }

    bool GuidanceWorker::spin_cb()
    {
        cav_msgs::GuidanceState state;
        state.state = gsm->getCurrentState();
        state_publisher_.publish(state);
        return true;
    }

    void GuidanceWorker::process_required_plugin_list(std::vector<std::string> list)
    {
        for(auto name = list.begin(); name != list.end(); ++name)
        {
            name->erase(std::remove(name->begin(), name->end(), ' '), name->end());
        }
    }

    void GuidanceWorker::create_guidance_state_machine()
    {
        try {
            guidance_state_machine_factory.createStateMachineInstance(vehicle_state_machine_type);
        }
        catch (const std::exception& e) { 
            std::cout << e.what();
        }
    }

    int GuidanceWorker::run()
    {
        ROS_INFO("Initalizing guidance node...");
        ros::CARMANodeHandle::setSystemAlertCallback(std::bind(&GuidanceWorker::system_alert_cb, this, std::placeholders::_1));
        // Init our ROS objects
        registered_plugin_service_server_ = nh_.advertiseService("plugins/get_registered_plugins", &GuidanceWorker::registered_plugin_cb, this);
        active_plugin_service_server_ = nh_.advertiseService("plugins/get_active_plugins", &GuidanceWorker::active_plugin_cb, this);
        activate_plugin_service_server_ = nh_.advertiseService("plugins/activate_plugin", &GuidanceWorker::activate_plugin_cb, this);
        guidance_activate_service_server_ = nh_.advertiseService("set_guidance_active", &GuidanceWorker::guidance_acivation_cb, this);
        guidance_activated_.store(false);
        state_publisher_ = nh_.advertise<cav_msgs::GuidanceState>("state", 5);
        robot_status_subscriber_ = nh_.subscribe<cav_msgs::RobotEnabled>("robot_status", 5, &GuidanceWorker::robot_status_cb, this);
        plugin_discovery_subscriber_ = nh_.subscribe<cav_msgs::Plugin>("plugin_discovery", 5, &GuidanceWorker::plugin_discovery_cb, this);
        plugin_publisher_ = nh_.advertise<cav_msgs::PluginList>("plugins/available_plugins", 5, true);
        enable_client_ = nh_.serviceClient<cav_srvs::SetEnableRobotic>("controller/enable_robotic");

        // Load the spin rate param to determine how fast to process messages
        // Default rate 10.0 Hz
        double spin_rate = pnh_.param<double>("spin_rate_hz", 10.0);

        pnh_.getParam("required_plugins", required_plugins);
        process_required_plugin_list(required_plugins);

        nh_.getParam("vehicle_state_machine_type", vehicle_state_machine_type);
        create_guidance_state_machine();

        // Spin until system shutdown
        ROS_INFO_STREAM("Guidance node initialized, spinning at " << spin_rate << "hz...");
        ros::CARMANodeHandle::setSpinCallback(std::bind(&GuidanceWorker::spin_cb, this));
        ros::CARMANodeHandle::setSpinRate(spin_rate);
        ros::CARMANodeHandle::spin();
    } 
}

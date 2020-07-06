/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "health_monitor.h"

namespace health_monitor
{

    void HealthMonitor::initialize()
    {
        // init node handles
        ROS_INFO_STREAM("Initalizing health_monitor node...");
        nh_ = ros::CARMANodeHandle();
        pnh_ = ros::CARMANodeHandle("~");

        // init ros service servers, publishers and subscribers
        registered_plugin_service_server_ = nh_.advertiseService("plugins/get_registered_plugins", &HealthMonitor::registered_plugin_cb, this);
        active_plugin_service_server_ = nh_.advertiseService("plugins/get_active_plugins", &HealthMonitor::active_plugin_cb, this);
        activate_plugin_service_server_ = nh_.advertiseService("plugins/activate_plugin", &HealthMonitor::activate_plugin_cb, this);
        get_strategic_plugin_by_capability_server_ = nh_.advertiseService("plugins/get_strategic_plugin_by_capability", &PluginManager::get_strategic_plugins_by_capability, &plugin_manager_);
        get_tactical_plugin_by_capability_server_ = nh_.advertiseService("plugins/get_tactical_plugin_by_capability", &PluginManager::get_tactical_plugins_by_capability, &plugin_manager_);
        plugin_discovery_subscriber_ = nh_.subscribe<cav_msgs::Plugin>("plugin_discovery", 5, &HealthMonitor::plugin_discovery_cb, this);
        driver_discovery_subscriber_ = nh_.subscribe<cav_msgs::DriverStatus>("driver_discovery", 5, &HealthMonitor::driver_discovery_cb, this);

        // load params
        spin_rate_ = pnh_.param<double>("spin_rate_hz", 10.0);
        driver_timeout_ = pnh_.param<double>("required_driver_timeout", 500);
        startup_duration_ = pnh_.param<double>("startup_duration", 25);
        plugin_service_prefix_ = pnh_.param<std::string>("plugin_service_prefix", "");
        strategic_plugin_service_suffix_ = pnh_.param<std::string>("strategic_plugin_service_suffix", "");
        tactical_plugin_service_suffix_ = pnh_.param<std::string>("tactical_plugin_service_suffix", "");
        
        pnh_.getParam("required_plugins", required_plugins_);
        pnh_.getParam("required_drivers", required_drivers_);
        pnh_.getParam("lidar_gps_drivers", lidar_gps_drivers_); 

        truck_=false;
        car_=false;
        pnh_.getParam("truck", truck_);
        pnh_.getParam("car", car_);
         

        // initialize worker class
        plugin_manager_ = PluginManager(required_plugins_, plugin_service_prefix_, strategic_plugin_service_suffix_, tactical_plugin_service_suffix_);
        driver_manager_ = DriverManager(required_drivers_, driver_timeout_,lidar_gps_drivers_); 

        // record starup time
        start_up_timestamp_ = ros::Time::now().toNSec() / 1e6;
        start_time_flag_=ros::Time::now();
        
    }
    
    void HealthMonitor::run()
    {
        initialize();
        ros::CARMANodeHandle::setSpinCallback(std::bind(&HealthMonitor::spin_cb, this));
        ros::CARMANodeHandle::setSpinRate(spin_rate_);
        ros::CARMANodeHandle::spin();
    }

    bool HealthMonitor::registered_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res)
    {
        plugin_manager_.get_registered_plugins(res);
        return true;
    }

    bool HealthMonitor::active_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res)
    {
        plugin_manager_.get_active_plugins(res);
        return true;
    }
    
    bool HealthMonitor::activate_plugin_cb(cav_srvs::PluginActivationRequest& req, cav_srvs::PluginActivationResponse& res)
    {
        bool answer = plugin_manager_.activate_plugin(req.pluginName, req.activated);
        if(answer) {
            res.newState = req.activated;
        }
        return answer;
    }

    void HealthMonitor::plugin_discovery_cb(const cav_msgs::PluginConstPtr& msg)
    {
        plugin_manager_.update_plugin_status(msg);
    }

    void HealthMonitor::driver_discovery_cb(const cav_msgs::DriverStatusConstPtr& msg)
    {
        // convert ros nanosecond to millisecond by the factor of 1/1e6
        driver_manager_.update_driver_status(msg, ros::Time::now().toNSec() / 1e6);
    }

    bool HealthMonitor::spin_cb()
    {
        long time_now=(ros::Time::now().toNSec() / 1e6);
        ros::Duration sd(startup_duration_);
        long start_duration=sd.toNSec() / 1e6;

        nh_.publishSystemAlert(driver_manager_.handleSpin(truck_,car_,time_now,start_up_timestamp_,start_duration,start_time_flag_.isZero()));
        return true;
    }

}

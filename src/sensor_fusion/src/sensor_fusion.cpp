/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <sensor_fusion/sensor_fusion.h>

int SensorFusionApplication::run() {
    nh_.reset(new ros::NodeHandle());

    ros::NodeHandle pnh("~filtered");

    ROS_INFO_STREAM("Waiting for Interface Manager");
    ros::service::waitForService("get_drivers_with_capabilities");

    ros::Subscriber bsm_sub = nh_->subscribe<cav_msgs::BSMCoreData>("bsm", 1000, &SensorFusionApplication::bsm_cb, this);
    ros::Timer timer = nh_->createTimer(ros::Duration(5.0),[this](const ros::TimerEvent& ev){get_services();});

    odom_pub_ = pnh.advertise<nav_msgs::Odometry>("odometry",1);
    navsatfix_pub_ = pnh.advertise<sensor_msgs::NavSatFix>("nav_sat_fix",1);
    velocity_pub_ = pnh.advertise<geometry_msgs::TwistStamped>("velocity",1);
    heading_pub_ = pnh.advertise<cav_msgs::HeadingStamped>("heading",1);
    objects_pub_ = pnh.advertise<cav_msgs::ExternalObjectList>("tracked_objects",1);
    vehicles_pub_= pnh.advertise<cav_msgs::BSMCoreData>("bsm",1);


    ros::Rate r(50);
    while(ros::ok())
    {
        ros::spinOnce();
        publish_updates();
        r.sleep();
    }

    return 0;
}

std::vector<std::string>
SensorFusionApplication::get_api(const cav_srvs::GetDriversWithCapabilitiesRequest::_category_type type,
                                 const std::string &name) {
    ros::ServiceClient client = nh_->serviceClient<cav_srvs::GetDriversWithCapabilities>("get_drivers_with_capabilities");
    cav_srvs::GetDriversWithCapabilities srv;
    srv.request.category = type;
    srv.request.capabilities.push_back(name);

    std::vector<std::string> ret;
    if(client.call(srv))
    {
        for(std::string it : srv.response.driver_names)
        {
            if(bond_map_.find(it) == bond_map_.end())
            {
                ros::ServiceClient bond_client = nh_->serviceClient<cav_srvs::Bind>(it+"/bind");
                cav_srvs::Bind req;
                req.request.id = boost::lexical_cast<std::string>(uuid_);

                if(bond_client.call(req))
                {
                    bond_map_[it]= std::unique_ptr<bond::Bond>(new bond::Bond(it+"/bond",
                                                                              boost::lexical_cast<std::string>(uuid_),
                                                                              boost::bind(&SensorFusionApplication::on_broken_cb,
                                                                                          this,
                                                                                          it),boost::bind(&SensorFusionApplication::on_conneced_cb,this,it)));

                    bond_map_[it]->start();
                    if(bond_map_[it]->waitUntilFormed(ros::Duration(1.0)))
                    {
                        ROS_ERROR_STREAM("Failed to form bond");
                        continue;
                    }

                }
            }
            std::string topic_name = it + "/" + name;
            if(sub_map_.find(topic_name) == sub_map_.end())
                ret.push_back(topic_name);
        }

    }

    return ret;
}

void SensorFusionApplication::get_services() {
    //odometry
    std::vector<std::string> ret = get_api(cav_srvs::GetDriversWithCapabilitiesRequest::POSITION, "position/odometry");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<nav_msgs::Odometry>(it,1,[this,it](const ros::MessageEvent<nav_msgs::Odometry const>& msg){ odom_cb(msg);});
    }

    //nav_sat_fix
    ret = get_api(cav_srvs::GetDriversWithCapabilitiesRequest::POSITION, "position/nav_sat_fix");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<sensor_msgs::NavSatFix>(it,1,[this](const ros::MessageEvent<sensor_msgs::NavSatFix const>&msg){ navsatfix_cb(msg);});
    }

    //heading
    ret = get_api(cav_srvs::GetDriversWithCapabilitiesRequest::POSITION, "position/heading");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<cav_msgs::HeadingStamped>(it,1,[this](const ros::MessageEvent<cav_msgs::HeadingStamped>&  msg){ heading_cb(msg);});
    }

    //velocity
    ret = get_api(cav_srvs::GetDriversWithCapabilitiesRequest::POSITION, "position/heading");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<geometry_msgs::TwistStamped>(it,1,[this](const ros::MessageEvent<geometry_msgs::TwistStamped>& msg){ velocity_cb(msg);});
    }

    //tracked_objects
    ret = get_api(cav_srvs::GetDriversWithCapabilitiesRequest::SENSOR, "sensor/tracked_objects");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<cav_msgs::ExternalObjectList>(it,1,[this, it](const ros::MessageEvent<cav_msgs::ExternalObjectList>& msg){ trackedobjects_cb(msg);});
    }

}

void SensorFusionApplication::publish_updates() {
    //TODO: publish processed data
    if(!odom_map_.empty())
        odom_pub_.publish(odom_map_.begin()->second);

    if(!navsatfix_map_.empty())
        navsatfix_pub_.publish(navsatfix_map_.begin()->second);

    if(!heading_map_.empty())
        heading_pub_.publish(heading_map_.begin()->second);

    if(!velocity_map_.empty())
        velocity_pub_.publish(velocity_map_.begin()->second);

    if(!objects_map_.empty())
        objects_pub_.publish(objects_map_.begin()->second);

    while(!bsm_q_.empty())
    {
        vehicles_pub_.publish(bsm_q_.front());
        bsm_q_.pop();
    }
}

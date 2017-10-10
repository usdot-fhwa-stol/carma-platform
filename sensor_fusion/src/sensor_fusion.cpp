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

#include <cav_msgs/ConnectedVehicleList.h>

int SensorFusionApplication::run() {
    nh_.reset(new ros::NodeHandle());

    ros::NodeHandle pnh("~filtered");

    ROS_INFO_STREAM("Waiting for Interface Manager");
    ros::service::waitForService("get_drivers_with_capabilities");

    ros::Subscriber bsm_sub = nh_->subscribe<cav_msgs::BSMCoreData>("bsm", 1000, &SensorFusionApplication::bsm_cb, this);
    ros::Timer timer = nh_->createTimer(ros::Duration(5.0),[this](const ros::TimerEvent& ev){ update_subscribed_services();});

    odom_pub_ = pnh.advertise<nav_msgs::Odometry>("odometry",1);
    navsatfix_pub_ = pnh.advertise<sensor_msgs::NavSatFix>("nav_sat_fix",1);
    velocity_pub_ = pnh.advertise<geometry_msgs::TwistStamped>("velocity",1);
    heading_pub_ = pnh.advertise<cav_msgs::HeadingStamped>("heading",1);
    objects_pub_ = pnh.advertise<cav_msgs::ExternalObjectList>("objects",1);
    vehicles_pub_= pnh.advertise<cav_msgs::ConnectedVehicleList>("tracked_vehicles",1);


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
SensorFusionApplication::get_api(const std::string &name) {

    //Setup service call
    ros::ServiceClient client = nh_->serviceClient<cav_srvs::GetDriversWithCapabilities>("get_drivers_with_capabilities");
    cav_srvs::GetDriversWithCapabilities srv;
    srv.request.capabilities.push_back(name);

    std::vector<std::string> ret;
    if(client.call(srv))
    {

        //The service returns a list of drivers that have the api we provided
        for(std::string fqn : srv.response.driver_data)
        {
            size_t pos = fqn.find(name);
            std::string driverName = fqn.substr(0,pos);

            //Bond with the node if we haven't already
            if(bond_map_.find(driverName) == bond_map_.end())
            {
                ROS_DEBUG_STREAM("Bonding to node: " << driverName);
                ros::ServiceClient bond_client = nh_->serviceClient<cav_srvs::Bind>(driverName+"/bind");
                cav_srvs::Bind req;
                req.request.id = boost::lexical_cast<std::string>(uuid_);

                if(bond_client.call(req))
                {
                    bond_map_[driverName]= std::unique_ptr<bond::Bond>(new bond::Bond(driverName+"/bond",
                                                                              boost::lexical_cast<std::string>(uuid_),
                                                                              boost::bind(&SensorFusionApplication::on_broken_cb,
                                                                                          this,
                                                                                          driverName),boost::bind(
                                    &SensorFusionApplication::on_connected_cb,this,driverName)));

                    bond_map_[driverName]->start();
                    if(!bond_map_[driverName]->waitUntilFormed(ros::Duration(1.0)))
                    {
                        ROS_ERROR_STREAM("Failed to form bond");
                        continue;
                    }

                }
            }

            //If we haven't subscribed to the topic formed by the name of the node and the service
            //add this topic to the return list
            if(sub_map_.find(fqn) == sub_map_.end())
                ret.push_back(fqn);
        }

    }

    return ret;
}

void SensorFusionApplication::update_subscribed_services() {
    //odometry
    std::vector<std::string> ret = get_api("position/odometry");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<nav_msgs::Odometry>(it,1,[this](const ros::MessageEvent<nav_msgs::Odometry const>& msg){ odom_cb(msg);});
    }

    //nav_sat_fix
    ret = get_api("position/nav_sat_fix");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<sensor_msgs::NavSatFix>(it,1,[this](const ros::MessageEvent<sensor_msgs::NavSatFix const>&msg){ navsatfix_cb(msg);});
    }

    //heading
    ret = get_api("position/heading");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<cav_msgs::HeadingStamped>(it,1,[this](const ros::MessageEvent<cav_msgs::HeadingStamped>&  msg){ heading_cb(msg);});
    }

    //velocity
    ret = get_api("position/velocity");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<geometry_msgs::TwistStamped>(it,1,[this](const ros::MessageEvent<geometry_msgs::TwistStamped>& msg){ velocity_cb(msg);});
    }

    //objects
    ret = get_api("sensor/objects");
    for(const std::string& it : ret)
    {
        sub_map_[it] = nh_->subscribe<cav_msgs::ExternalObjectList>(it,1,[this](const ros::MessageEvent<cav_msgs::ExternalObjectList>& msg){ trackedobjects_cb(msg);});
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

    cav_msgs::ConnectedVehicleList msg;
    static unsigned int seq = 1;
    std_msgs::Header header;
    header.frame_id = "base_link";
    header.stamp = ros::Time::now();
    header.seq = seq++;
    while(!bsm_q_.empty())
    {
        cav_msgs::ExternalObject externalObject;
        cav_msgs::BSMCoreDataConstPtr& bsm = bsm_q_.front();
        externalObject.header = header;

        //todo: add real pose calculation
        geometry_msgs::Pose pose;
        pose.position.x = 10;
        externalObject.pose.pose = pose;

        geometry_msgs::Twist twist;
        twist.linear.x = bsm->speed;

        externalObject.velocity.twist = twist;
        externalObject.velocity_inst.twist = twist;

        geometry_msgs::Vector3 size;
        size.x = bsm->size.vehicle_length;
        size.y = bsm->size.vehicle_width;
        size.z = bsm->size.vehicle_width;
        externalObject.size = size;

        //todo: process a bsm message, update this to the BSM message type
        msg.objects.push_back(externalObject);
        msg.bsm_temp_ids.push_back(uint16_t(bsm->id));

        bsm_q_.pop();
    }

    vehicles_pub_.publish(msg);
}

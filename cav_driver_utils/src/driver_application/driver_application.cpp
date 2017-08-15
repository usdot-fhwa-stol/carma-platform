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

#include <driver_application/driver_application.h>

namespace cav
{

int DriverApplication::run()
{
    //Initialize nodehandles
    nh_.reset(new ros::NodeHandle());
    pnh_.reset(new ros::NodeHandle("~"));

    //Setup initial status
    status_.name = ros::this_node::getName();
    status_.status = cav_msgs::DriverStatus::OFF;

    //Initialize DriverApplication services
    ros::ServiceServer bind_service = pnh_->advertiseService("bind", &DriverApplication::bind_service_cb,this);
    ros::ServiceServer api_service = pnh_->advertiseService("get_api_specification",
                                                            &DriverApplication::getApiSpecificationCallback, this);
    driver_status_pub_ = nh_->advertise<cav_msgs::DriverStatus>("driver_discovery",1);
    ros::Timer timer = pnh_->createTimer(ros::Duration(1), &DriverApplication::status_publish_timer,this);
    ROS_INFO("Driver services initialized");

    initialize();

    ros::Rate r(spin_rate);
    while(ros::ok())
    {
        pre_spin();
        ros::spinOnce();
        post_spin();
        r.sleep();
    }
}

bool DriverStatusEquals(cav_msgs::DriverStatus a, cav_msgs::DriverStatus b)
{
    return a.status == b.status &&
            a.position == b.position &&
            a.can_bus == b.can_bus &&
            a.comms == b.comms &&
            a.controller == b.controller
           && a.sensor == b.sensor;
}

void DriverApplication::setStatus(cav_msgs::DriverStatus status)
{
    //If the driver status has been changed we need to release bonds
    //notifying connected nodes there has been a change in the status of this node
    if(!DriverStatusEquals(status,status_))
    {
        ROS_INFO_STREAM("Status updated releasing bonds, Status: " << status.status);
        bond_map_.clear();

        status_ = status;
        status_.name = ros::this_node::getName();
    }
}

void DriverApplication::status_publish_timer(const ros::TimerEvent &) const
{
    driver_status_pub_.publish(status_);
}

bool DriverApplication::bind_service_cb(cav_srvs::Bind::Request &req, cav_srvs::Bind::Response &res)
{
    if(bond_map_.find(req.id) != bond_map_.end())
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " already bonded to id: " << req.id);
        return true;
    }

    ROS_DEBUG_STREAM(ros::this_node::getName() << " binding to id: " << req.id);

    std::shared_ptr<bond::Bond> bond(new bond::Bond(ros::this_node::getName() + "/bond", req.id));
    bond_map_[req.id] = bond;
    bond->start();

    return true;
}

bool DriverApplication::getApiSpecificationCallback(cav_srvs::GetAPISpecificationRequest &req,
                                                    cav_srvs::GetAPISpecificationResponse &res)
{
    res.api_list = get_api();

    ROS_DEBUG_STREAM("Sending API");
    return true;
}


}

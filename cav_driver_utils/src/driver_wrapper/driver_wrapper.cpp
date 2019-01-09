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

#include <driver_wrapper/driver_wrapper.h>

namespace cav {

int DriverWrapper::run()
{
    nh_.reset(new ros::NodeHandle);
    private_nh_.reset(new ros::NodeHandle("~"));

    //Setup initial status
    status_.name = ros::this_node::getName();
    status_.status = cav_msgs::DriverStatus::OFF;

    //Initialize pubs and subs
    driver_status_pub_ = nh_->advertise<cav_msgs::DriverStatus>("driver_discovery", 1);
    system_alert_sub_ = nh_->subscribe("system_alert", 10, &DriverWrapper::system_alert_cb, this);
    ros::Timer timer = private_nh_->createTimer(ros::Duration(1), &DriverWrapper::status_publish_timer, this);

    ROS_INFO_STREAM("Driver Initializing");

    initialize();
    ros::Rate r(spin_rate_);
    while(ros::ok() && !shutdown_)
    {
        pre_spin();
        ros::spinOnce();
        post_spin();
        r.sleep();
    }

    ROS_INFO_STREAM("Driver Shutting Down");
    shutdown();
    ros::shutdown();
}

void DriverWrapper::status_publish_timer(const ros::TimerEvent &) const
{
    driver_status_pub_.publish(status_);
}

void DriverWrapper::system_alert_cb(const cav_msgs::SystemAlertConstPtr &msg)
{
    u_char t = msg->type;
    if(t == cav_msgs::SystemAlert::FATAL || t == cav_msgs::SystemAlert::SHUTDOWN)
    {
        shutdown_ = true;
    }
}

}

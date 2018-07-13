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

#include "node_manager.h"
#include <cav_msgs/SystemAlert.h>
#include <ros/ros.h>

bool should_shutdown = false;

void system_alert_cb(const cav_msgs::SystemAlertConstPtr &msg)
{
    switch(msg->type)
    {
        case cav_msgs::SystemAlert::FATAL:
        case cav_msgs::SystemAlert::SHUTDOWN:
            should_shutdown  = true;
            break;
        default:
            break;
    }

}


int main(int argc, char** argv)
{

    ros::init(argc,argv,"jausNM");
    ros::NodeHandle nh("~");

    //Read in params
    std::string nm_cfg_file;
    nh.param<std::string>("node_manager_config_file", nm_cfg_file, "nodeManager.conf");

    if(!cav::NodeManager::instance().init(nm_cfg_file))
    {
        ROS_ERROR_STREAM("Failed to initialize JAUS NodeManager");
        return EXIT_FAILURE;
    }

    ros::Subscriber sub = nh.subscribe("system_alert",10,&system_alert_cb);

    ros::Rate r(5);
    while(ros::ok() && !should_shutdown )
    {
        ros::spin();
        r.sleep();
    }

    ROS_INFO_STREAM("Shutting down");

    return EXIT_SUCCESS;
}
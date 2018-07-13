#pragma once
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
#include "command_mode.h"

#include <ros/ros.h>
#include <cav_msgs/SpeedAccel.h>
#include <vector>
#include <string>

namespace cav
{
struct LongitudinalSpeedControllerCommand
{
    double speed,max_accel;
};

/**
 * @brief Wrapper for the control/cmd_speed API
 *
 * Manages the topic advertisement and exposes the command through the provided signal
 */
class LongitudinalSpeedController : public CommandProvider<LongitudinalSpeedControllerCommand>
{

    ros::NodeHandle pnh_;
    ros::Subscriber sub_;

    std::vector<std::string> api_;

    /**
     * @brief Callback for the control/cmd_speed topic
     * @param msg
     */
    void _cb(const cav_msgs::SpeedAccelConstPtr& msg)
    {
        command.speed = msg->speed;
        command.max_accel = msg->max_accel;
        onNewCommand(command);
    }

public:

    typedef LongitudinalSpeedControllerCommand Command;
    Command command;

    LongitudinalSpeedController() : pnh_("~control")
    {
        sub_ =  pnh_.subscribe<cav_msgs::SpeedAccel>("cmd_speed",1,&LongitudinalSpeedController::_cb, this);
        api_.push_back(sub_.getTopic());
    }

    /**
     * @brief Returns the fully qualified ROS api for this class
     * @return
     */
    std::vector<std::string>& get_api()
    {
        return api_;
    }


};
}

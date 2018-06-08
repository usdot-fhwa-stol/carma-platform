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
#include <cav_msgs/RobotEnabled.h>
#include <ros/ros.h>
#include <vector>
#include <string>

namespace cav
{

/**
 * @brief ActiveRoboticStatusProvider Wraps the behavior for managing the control/robot_status API
 */
class ActiveRoboticStatusProvider
{
    ros::NodeHandle pnh_;
    ros::Publisher pub_;
    std::vector<std::string> api_;

    bool active_, enabled_;
public:

    ActiveRoboticStatusProvider() : pnh_("~control"), active_(false), enabled_(false)
    {
        pub_ = pnh_.advertise<cav_msgs::RobotEnabled>("robot_status", 1);
        api_.push_back(pub_.getTopic());
    }

    /**
     * @brief Returns the fully scoped API for control/robot_status
     * @return
     */
    std::vector<std::string>& get_api() { return api_; }

    /**
     * @brief Publishes the status to ~control/robot_status
     */
    void publishState()
    {
        cav_msgs::RobotEnabled msg;
        msg.robot_enabled = enabled_;
        msg.robot_active = active_;

        pub_.publish(msg);
    }

    /**
     * @brief sets the robot_active flag
     * @param val
     */
    void setActive(bool val)
    {
        active_ = val;

        publishState();
    }

    /**
     * @brief sets the robot_enabled flag
     * @param val
     */
    void setEnabled(bool val)
    {
        enabled_ = val;

        publishState();
    }

    /**
     * @brief returns the robot_active flag
     * @return
     */
    bool getActive() { return active_; }

    /**
     * @brief returns the robot_enabled flag
     * @return
     */
    bool getEnabled() { return enabled_; }
};

}

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
#include <cav_srvs/SetEnableRobotic.h>
#include <ros/ros.h>
#include <boost/signals2/signal.hpp>
#include <vector>
#include <string>

namespace cav
{

/**
 * @brief Provides a wrapper to the control/enable_robotic service API
 */
class EnableRoboticService
{
    ros::NodeHandle pnh_;
    ros::ServiceServer srv_;
    std::vector<std::string> api_;

    bool robotic_enabled_;

    /**
     * @brief Callback for the control/enable_robotic sservice
     * @param req
     * @return
     */
    bool _cb(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticRequest&) {
        bool tmp = robotic_enabled_;
        robotic_enabled_ = req.set == cav_srvs::SetEnableRoboticRequest::ENABLE;
        if(tmp != robotic_enabled_)
            onEnabledChanged(robotic_enabled_);
        return true;
    }

public:

    /**
     * @brief Signaled on state change
     */
    boost::signals2::signal<void (bool)> onEnabledChanged;

    EnableRoboticService() : pnh_("~control")
    {
        pnh_.param<bool>("enabled_at_start", robotic_enabled_, false);
        srv_ = pnh_.advertiseService("enable_robotic", &EnableRoboticService::_cb, this);
        api_.push_back(srv_.getService());
    }

    /**
     * @brief Returns the enabled flag
     * @return
     */
    bool isEnabled() { return robotic_enabled_; }

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

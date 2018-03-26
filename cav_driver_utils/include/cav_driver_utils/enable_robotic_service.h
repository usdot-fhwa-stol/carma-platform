#pragma once

#include <cav_srvs/SetEnableRobotic.h>
#include <ros/ros.h>
#include <boost/signals2/signal.hpp>
#include <vector>
#include <string>

namespace cav
{

class EnableRoboticService
{
    ros::NodeHandle pnh_;
    ros::ServiceServer srv_;
    std::vector<std::string> api_;

    bool robotic_enabled_;
    bool _cb(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticRequest&) {
        bool tmp = robotic_enabled_;
        robotic_enabled_ = req.set == cav_srvs::SetEnableRoboticRequest::ENABLE;
        if(tmp != robotic_enabled_)
            onEnabledChanged(robotic_enabled_);
        return true;
    }
public:

    boost::signals2::signal<void (bool)> onEnabledChanged;

    EnableRoboticService() : pnh_("~control")
    {
        pnh_.param<bool>("enabled_at_start", robotic_enabled_, false);
        srv_ = pnh_.advertiseService("enable_robotic", &EnableRoboticService::_cb, this);
        api_.push_back(srv_.getService());
    }

    bool isEnabled() { return robotic_enabled_; }

    std::vector<std::string>& get_api()
    {
        return api_;
    }


};

}

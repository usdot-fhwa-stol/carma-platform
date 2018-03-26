#pragma once

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


class LongitudinalSpeedController : public CommandProvider<LongitudinalSpeedControllerCommand>
{

    ros::NodeHandle pnh_;
    ros::Subscriber sub_;

    std::vector<std::string> api_;

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

    std::vector<std::string>& get_api()
    {
        return api_;
    }


};
}

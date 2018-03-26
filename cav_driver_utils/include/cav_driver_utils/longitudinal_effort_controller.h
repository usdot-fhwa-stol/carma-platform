#pragma once
#include "command_mode.h"

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <string>
namespace cav
{

struct LongitudinalEffortControllerCommand
{
    float effort;
};

class LongitudinalEffortController : public CommandProvider<LongitudinalEffortControllerCommand>
{

    ros::NodeHandle pnh_;
    ros::Subscriber sub_;

    std::vector<std::string> api_;

    void _cb(const std_msgs::Float32ConstPtr& msg)
    {
        command.effort = msg->data;
        onNewCommand(command);
    }

public:

    typedef LongitudinalEffortControllerCommand Command;

    Command command;

    LongitudinalEffortController() : pnh_("~control")
    {
        sub_ =  pnh_.subscribe<std_msgs::Float32>("cmd_longitudinal_effort",1,&LongitudinalEffortController::_cb, this);
        api_.push_back(sub_.getTopic());
    }

    std::vector<std::string>& get_api()
    {
        return api_;
    }


};
}

#include <cav_msgs/RobotEnabled.h>
#include <ros/ros.h>
#include <vector>
#include <string>

namespace cav
{

class ActiveRoboticStatusProvider
{
    ros::NodeHandle pnh_;
    ros::Publisher pub_;
    std::vector<std::string> api_;

    bool active_, enabled_;

public:

    cav_msgs::RobotEnabled msg;
    ActiveRoboticStatusProvider() : pnh_("~control"), active_(false), enabled_(false)
    {
        pub_ = pnh_.advertise<cav_msgs::RobotEnabled>("robot_status", 1);
        api_.push_back(pub_.getTopic());
    }

    std::vector<std::string>& get_api() { return api_; }

    void publishState()
    {
        pub_.publish(msg);
    }

    void setActive(bool val)
    {
        active_ = val;

        publishState();
    }

    void setEnabled(bool val)
    {
        enabled_ = val;

        publishState();
    }

    bool getActive() { return active_; }
    bool getEnabled() { return enabled_; }
};

}

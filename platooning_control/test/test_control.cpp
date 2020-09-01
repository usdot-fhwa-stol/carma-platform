#include "platoon_control.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PlatoonControlPluginTest, test1)
{

    platoon_control::PlatoonControlPlugin pc;
    
    ros::Time::init();

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -1.0;
    pc.pose_msg_.reset(new geometry_msgs::PoseStamped(pose));

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 2.0;
    point.target_time = 1e8;
    geometry_msgs::TwistStamped res_twist;
    res_twist = pc.composeTwist(point);
    EXPECT_NEAR(0.0,res_twist.twist.linear.x, 0.1);

}




#include "pure_pursuit.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(PurePursuitTest, test_filter)
{
    PlatooningControlPluginConfig config;
	config.lowpassGain = 0.5;
    
    platoon_control::PurePursuit pp;
    pp.config_ = config;
    double new_angle = pp.lowPassfilter(3.0, 0, config.lowpassGain);
    EXPECT_EQ(1.5, new_angle);
}

TEST(PurePursuitTest, test1)
{

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 1.0;
    point.target_time = ros::Time(1.0);
    platoon_control::PurePursuit pp;
    pp.calculateSteer(point);
    EXPECT_EQ(0, pp.steering_angle_);


}


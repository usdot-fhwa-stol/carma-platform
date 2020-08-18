#include "pure_pursuit.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PurePursuitTest, test1)
{

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 1.0;
    point.target_time = 1e9;
    platoon_control::PurePursuit pp;
    double res = pp.calculateSteer(point);
    EXPECT_EQ(0, res);

    cav_msgs::TrajectoryPlanPoint point2;
    point2.x = 4.0;
    point2.y = 4.0;
    point2.target_time = 2e9;
    double res2 = pp.calculateSteer(point2);
    EXPECT_NEAR(0.5, res2, 0.1);

}


TEST(PurePursuitTest, test2)
{

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 1.0;
    point.target_time = 1e9;
    platoon_control::PurePursuit pp;
    double res = pp.calculateSteer(point);
    EXPECT_EQ(0, res);

    cav_msgs::TrajectoryPlanPoint point2;
    point2.x = -4.0;
    point2.y = -4.0;
    point2.target_time = 1e9;
    double res2 = pp.calculateSteer(point2);
    EXPECT_NEAR(-0.5, res2, 0.1);

}

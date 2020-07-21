#include "platoon_control_worker.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PlatoonControlWorkerTest, test1)
{
    platoon_control::PlatoonControlWorker pcw;
    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 2.0;
    pcw.generateSpeed(point);
    EXPECT_NEAR(0, pcw.speedCmd_, 0.1);
}

TEST(PlatoonControlWorkerTest, test2)
{
    platoon_control::PlatoonControlWorker pcw;
    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 2.0;
    pcw.generateSteer(point);
    EXPECT_NEAR(0, pcw.steerCmd_, 0.1);
}

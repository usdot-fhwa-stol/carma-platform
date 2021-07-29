#include "platoon_control.h"
#include <gtest/gtest.h>
#include <ros/ros.h>



TEST(PlatoonControlPluginTest, test2)
{
    cav_msgs::TrajectoryPlan tp;
    cav_msgs::TrajectoryPlanPoint point1;
    point1.x = 1.0;
    point1.y = 1.0;
    

    cav_msgs::TrajectoryPlanPoint point2;
    point2.x = 10.0;
    point2.y = 10.0;

    cav_msgs::TrajectoryPlanPoint point3;
    point3.x = 20.0;
    point3.y = 20.0;

    tp.trajectory_points = {point1, point2, point3};



    platoon_control::PlatoonControlPlugin pc;
    pc.current_speed_ = 5;
    cav_msgs::TrajectoryPlanPoint out = pc.getLookaheadTrajectoryPoint(tp);
    EXPECT_EQ(out.x, 10.0);
}




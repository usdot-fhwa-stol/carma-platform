#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <chrono>
#include "platoon_control.h"




// Declare a test
TEST(TestSuite, testCase1)
{
    ros::NodeHandle nh = ros::NodeHandle();
    ros::Publisher traj_pub_ = nh.advertise<cav_msgs::TrajectoryPlan>("PlatooningControlPlugin/plan_trajectory", 5);
    cav_msgs::TrajectoryPlan tp;
    traj_pub_.publish(tp);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    auto num = traj_pub_.getNumSubscribers();
    EXPECT_EQ(1, num);

}

TEST(TestSuite, testCase2)
{
    ros::NodeHandle nh = ros::NodeHandle();
    ros::Publisher twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>("current_velocity", 5);
    geometry_msgs::TwistStamped twist1;
    twist1.twist.linear.x = 10.0;
    twist_pub_.publish(twist1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    auto num = twist_pub_.getNumSubscribers();
    EXPECT_EQ(1, num);

}




int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_platoon_control");

    // std::thread spinner([] {while (ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    // ros::shutdown();

    return res;
}
#include "pid_controller.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PIDControllerTest, test1)
{
    platoon_control::PIDController pid;
    double res = pid.calculate(40, 38);
    EXPECT_EQ(-8, res);
}


TEST(PIDControllerTest, test2)
{
    platoon_control::PIDController pid;
    double res = pid.calculate(20, 300);
    EXPECT_EQ(100, res);
}

TEST(PIDControllerTest, test3)
{
    platoon_control::PIDController pid;
    double res = pid.calculate(300, 20);
    EXPECT_EQ(-100, res);
}

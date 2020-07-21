#include "pid_controller.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PIDControllerTest, test1)
{
    platoon_control::PIDController pid;
    double res = pid.calculate(12, 14);
    EXPECT_EQ(1, res);

    
}

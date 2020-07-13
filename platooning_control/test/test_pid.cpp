#include "pid_controller.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PIDControllerTest, test1)
{
    platoon_control::PIDController pid;
    double res = pid.calculate(12, 14);
    EXPECT_EQ(1, res);

    
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
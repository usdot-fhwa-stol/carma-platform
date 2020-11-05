#include "health_monitor.h"
#include "driver_manager.h"
#include <iostream>
#include "plugin_manager.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

namespace health_monitor
{
    

    TEST(HealthMonitorTest, runTest)
    {
        ROS_WARN_STREAM("HealthMonitorTest entered");

        HealthMonitor node;
        ROS_WARN_STREAM("Check2");

        //node.setIsPublishedFalse();

        bool publish_status_c, publish_status_t;
        ROS_WARN_STREAM("Check1");

        auto pub_stat_c = node.getPubStatusCar();
        ROS_WARN_STREAM("Checknext");

        auto pub_stat_t = node.getPubStatusTruck();

        ROS_WARN_STREAM("Check2");
        for(size_t i; i< pub_stat_c.size(); i++)
        {
            if (pub_stat_c[i] == false)
                publish_status_c = false;
            else
                publish_status_c = true;
        }
        ASSERT_EQ(publish_status_c, false); //Assert that none of the car's system alerts have been published

        for(size_t i; i< pub_stat_t.size(); i++)
        {
            if (pub_stat_t[i] == false)
                publish_status_t = false;
            else
                publish_status_t = true;
        }
        ASSERT_EQ(publish_status_t, false); //Assert that none of the truck's system alerts have been published

}





    TEST(HealthMonitorTest, setIsPublishedFalseTest)
    {


    }




}
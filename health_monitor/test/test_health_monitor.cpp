#include "health_monitor.h"
#include "driver_manager.h"
#include <iostream>
#include "plugin_manager.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

namespace health_monitor
{
    

    TEST(HealthMonitorTest, runTest_s_1_l_1_g_1)
    {

        HealthMonitor node;
        bool publish_status_c, publish_status_t;
        auto pub_stat_c = node.getPubStatusCar();
        for(int i = 0; i< pub_stat_c.size(); i++)
        {
            if (pub_stat_c.at(i) == false)
                publish_status_c = false;
            else
                publish_status_c = true;
        }
        ASSERT_EQ(publish_status_c, false); //Assert that none of the car's system alerts have been published

        /*Test the published status of each system alert*/

        // inordinary case where no critical drivers are specified
        DriverManager dm0;
        cav_msgs::DriverStatus msg0;
        msg0.controller = true;
        msg0.name = "controller";
        msg0.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg0_pointer(new cav_msgs::DriverStatus(msg0));
        dm0.update_driver_status(msg0_pointer, 1000);
        EXPECT_EQ("s_0", dm0.are_critical_drivers_operational_car(1500));
        
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);
        
        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_1_l_1_g_1", dm.are_critical_drivers_operational_car(1500));


        node.setDriverManager(dm);
        node.setCarTrue();
        node.spinObject(1500);
        auto pub_vec_c1 = node.getPubStatusCar();
        ASSERT_EQ(pub_vec_c1[0], true); //Check All driver's ready msg published (Car)


}

    TEST(HealthMonitorTest, runTest_s_1_l_1_g_0)
    {   
        HealthMonitor node;
        bool publish_status_c;

        auto pub_stat_c = node.getPubStatusCar();

        for(int i = 0; i< pub_stat_c.size(); i++)
        {
            if (pub_stat_c.at(i) == false)
                publish_status_c = false;
            else
                publish_status_c = true;
        }
        ASSERT_EQ(publish_status_c, false); //Assert that none of the car's system alerts have been published


        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_1_l_1_g_0", dm.are_critical_drivers_operational_car(1500));

        node.setCarTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_c1 = node.getPubStatusCar();
        ASSERT_EQ(pub_vec_c1[2], true); //Check System starting up msg published (Car)

    }



    TEST(HealthMonitorTest, runTest_s_1_l_0_g_1)
    {
        HealthMonitor node;
        bool publish_status_c;
        auto pub_stat_c = node.getPubStatusCar();

        for(int i = 0; i< pub_stat_c.size(); i++)
        {
            if (pub_stat_c.at(i) == false)
                publish_status_c = false;
            else
                publish_status_c = true;
        }
        ASSERT_EQ(publish_status_c, false); //Assert that none of the car's system alerts have been published

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_1_l_0_g_1", dm.are_critical_drivers_operational_car(1500));

        node.setCarTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_c1 = node.getPubStatusCar();
        ASSERT_EQ(pub_vec_c1[3], true);

    }



    TEST(HealthMonitorTest, runTest_s_1_l_0_g_0)
    {
         HealthMonitor node;
        bool publish_status_c;

        auto pub_stat_c = node.getPubStatusCar();
        for(int i = 0; i< pub_stat_c.size(); i++)
        {
            if (pub_stat_c.at(i) == false)
                publish_status_c = false;
            else
                publish_status_c = true;
        }
        ASSERT_EQ(publish_status_c, false); //Assert that none of the car's system alerts have been published

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_1_l_0_g_0", dm.are_critical_drivers_operational_car(1500));
        node.setCarTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_c1 = node.getPubStatusCar();
        ASSERT_EQ(pub_vec_c1[4], true);


    }

    TEST(HealthMonitorTest, runTest_s_0)
    {
        HealthMonitor node;
        bool publish_status_c;
        auto pub_stat_c = node.getPubStatusCar();

        for(int i = 0; i< pub_stat_c.size(); i++)
        {
            if (pub_stat_c.at(i) == false)
                publish_status_c = false;
            else
                publish_status_c = true;
        }
        ASSERT_EQ(publish_status_c, false); //Assert that none of the car's system alerts have been published


        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_car(1500));

        node.setCarTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_c1 = node.getPubStatusCar();
        ASSERT_EQ(pub_vec_c1[5], true);
    }

    TEST(HealthMonitorTest, runTest_s_1_l1_1_l2_1_g_1)
    {
          HealthMonitor node;
        bool publish_status_t;
        auto pub_stat_t = node.getPubStatusTruck();

        for(int i = 0; i< pub_stat_t.size(); i++)
        {
            if (pub_stat_t[i] == false)
                publish_status_t = false;
            else
                publish_status_t = true;
        }
        ASSERT_EQ(publish_status_t, false); //Assert that none of the truck's system alerts have been published*/

                 std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_1_g_1", dm.are_critical_drivers_operational_truck(1500));

        node.setTruckTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_t1 = node.getPubStatusTruck();
        ASSERT_EQ(pub_vec_t1[0], true); //Check All driver's ready msg published (Truck)

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_0_l2_1_g_1)
    {
        HealthMonitor node;
        bool publish_status_t;
        auto pub_stat_t = node.getPubStatusTruck();

        for(int i = 0; i< pub_stat_t.size(); i++)
        {
            if (pub_stat_t[i] == false)
                publish_status_t = false;
            else
                publish_status_t = true;
        }
        ASSERT_EQ(publish_status_t, false); //Assert that none of the truck's system alerts have been published*/


        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_1_g_1", dm.are_critical_drivers_operational_truck(1500));

        node.setTruckTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_t1 = node.getPubStatusTruck();
        ASSERT_EQ(pub_vec_t1[2], true);

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_0_l2_1_g_0)
    {
        HealthMonitor node;
        bool publish_status_t;
        auto pub_stat_t = node.getPubStatusTruck();

        for(int i = 0; i< pub_stat_t.size(); i++)
        {
            if (pub_stat_t[i] == false)
                publish_status_t = false;
            else
                publish_status_t = true;
        }
        ASSERT_EQ(publish_status_t, false); //Assert that none of the truck's system alerts have been published*/

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_1_g_0", dm.are_critical_drivers_operational_truck(1500));

        node.setTruckTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_t1 = node.getPubStatusTruck();
        ASSERT_EQ(pub_vec_t1[3], true);

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_1_l2_1_g_0)
    {
        HealthMonitor node;
        bool publish_status_t;
        auto pub_stat_t = node.getPubStatusTruck();

        for(int i = 0; i< pub_stat_t.size(); i++)
        {
            if (pub_stat_t[i] == false)
                publish_status_t = false;
            else
                publish_status_t = true;
        }
        ASSERT_EQ(publish_status_t, false); //Assert that none of the truck's system alerts have been published*/

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_1_g_0", dm.are_critical_drivers_operational_truck(1500));

        node.setTruckTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_t1 = node.getPubStatusTruck();
        ASSERT_EQ(pub_vec_t1[4], true);


    }

    TEST(HealthMonitorTest, runTest_s_1_l1_0_l2_0_g_1)
    {
        HealthMonitor node;
        bool publish_status_t;
        auto pub_stat_t = node.getPubStatusTruck();

        for(int i = 0; i< pub_stat_t.size(); i++)
        {
            if (pub_stat_t[i] == false)
                publish_status_t = false;
            else
                publish_status_t = true;
        }
        ASSERT_EQ(publish_status_t, false); //Assert that none of the truck's system alerts have been published*/

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_0_g_1", dm.are_critical_drivers_operational_truck(1500));

        node.setTruckTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_t1 = node.getPubStatusTruck();
        ASSERT_EQ(pub_vec_t1[5], true);

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_0_l2_0_g_0)
    {
        HealthMonitor node;
        bool publish_status_t;
        auto pub_stat_t = node.getPubStatusTruck();

        for(int i = 0; i< pub_stat_t.size(); i++)
        {
            if (pub_stat_t[i] == false)
                publish_status_t = false;
            else
                publish_status_t = true;
        }
        ASSERT_EQ(publish_status_t, false); //Assert that none of the truck's system alerts have been published*/


        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_0_g_0", dm.are_critical_drivers_operational_truck(1500));

        node.setTruckTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_t1 = node.getPubStatusTruck();
        ASSERT_EQ(pub_vec_t1[6], true);
    }

    TEST(HealthMonitorTest, runTest_truck_s_0)
    {
        HealthMonitor node;
        bool publish_status_t;
        auto pub_stat_t = node.getPubStatusTruck();

        for(int i = 0; i< pub_stat_t.size(); i++)
        {
            if (pub_stat_t[i] == false)
                publish_status_t = false;
            else
                publish_status_t = true;
        }
        ASSERT_EQ(publish_status_t, false); //Assert that none of the truck's system alerts have been published*/

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_truck(1500));

        node.setTruckTrue();
        node.setDriverManager(dm);
        node.spinObject(1500);
        auto pub_vec_t1 = node.getPubStatusTruck();
        ASSERT_EQ(pub_vec_t1[7], true);


    }




}
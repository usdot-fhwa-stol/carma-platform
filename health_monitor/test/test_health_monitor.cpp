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
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);
        
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

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "camera";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l_1_g_1_c_1", dm.are_critical_drivers_operational_car(1500));

}

    TEST(HealthMonitorTest, runTest_s_1_l_1_g_0)
    {   
        HealthMonitor node;

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "camera";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l_1_g_0", dm.are_critical_drivers_operational_car(1500));

    }



    TEST(HealthMonitorTest, runTest_s_1_l_0_g_1)
    {
        HealthMonitor node;

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "camera";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l_0_g_1", dm.are_critical_drivers_operational_car(1500));

    }



    TEST(HealthMonitorTest, runTest_s_1_l_0_g_0)
    {
         HealthMonitor node;

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "camera";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l_0_g_0", dm.are_critical_drivers_operational_car(1500));

    }

    TEST(HealthMonitorTest, runTest_s_0)
    {
        HealthMonitor node;


        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "camera";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_car(1500));

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_1_l2_1_g_1)
    {
          HealthMonitor node;

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg5;
        msg5.gnss = true;
        msg5.name = "camera";
        msg5.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg5_pointer(new cav_msgs::DriverStatus(msg5));
        dm.update_driver_status(msg5_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_1_g_1_c_1", dm.are_critical_drivers_operational_truck(1500));

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_0_l2_1_g_1)
    {
        HealthMonitor node;


        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg5;
        msg5.gnss = true;
        msg5.name = "camera";
        msg5.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg5_pointer(new cav_msgs::DriverStatus(msg5));
        dm.update_driver_status(msg5_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_1_g_1", dm.are_critical_drivers_operational_truck(1500));

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_0_l2_1_g_0)
    {
        HealthMonitor node;
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg5;
        msg5.gnss = true;
        msg5.name = "camera";
        msg5.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg5_pointer(new cav_msgs::DriverStatus(msg5));
        dm.update_driver_status(msg5_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_1_g_0", dm.are_critical_drivers_operational_truck(1500));

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_1_l2_1_g_0)
    {
        HealthMonitor node;

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg5;
        msg5.gnss = true;
        msg5.name = "camera";
        msg5.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg5_pointer(new cav_msgs::DriverStatus(msg5));
        dm.update_driver_status(msg5_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_1_g_0", dm.are_critical_drivers_operational_truck(1500));


    }

    TEST(HealthMonitorTest, runTest_s_1_l1_0_l2_0_g_1)
    {
        HealthMonitor node;

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg5;
        msg5.gnss = true;
        msg5.name = "camera";
        msg5.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg5_pointer(new cav_msgs::DriverStatus(msg5));
        dm.update_driver_status(msg5_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_0_g_1", dm.are_critical_drivers_operational_truck(1500));

    }

    TEST(HealthMonitorTest, runTest_s_1_l1_0_l2_0_g_0)
    {
        HealthMonitor node;


        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg5;
        msg5.gnss = true;
        msg5.name = "camera";
        msg5.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg5_pointer(new cav_msgs::DriverStatus(msg5));
        dm.update_driver_status(msg5_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_0_g_0", dm.are_critical_drivers_operational_truck(1500));

    }

    TEST(HealthMonitorTest, runTest_truck_s_0)
    {
        HealthMonitor node;

        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers,camera_drivers);

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

        cav_msgs::DriverStatus msg5;
        msg5.gnss = true;
        msg5.name = "camera";
        msg5.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg5_pointer(new cav_msgs::DriverStatus(msg5));
        dm.update_driver_status(msg5_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_truck(1500));


    }




}
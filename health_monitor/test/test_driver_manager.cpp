/*
 * Copyright (C) 2019-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "driver_manager.h"
#include <gtest/gtest.h>

namespace health_monitor
{

// Unit test for car part

    TEST(DriverManagerTest, testCarNormalDriverStatus)
    {
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
    }

    TEST(DriverManagerTest, testCarSsc_1_Lidar_0_Gps_0)
    {
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
    }

        TEST(DriverManagerTest, testCarSsc_0_Lidar_1_Gps_1)
    {
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
    }

    TEST(DriverManagerTest, testCarSsc_1_Lidar_0_Gps_1)
    {
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
    }

    TEST(DriverManagerTest, testCarSsc_1_Lidar_1_Gps_0)
    {
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
    }

        TEST(DriverManagerTest, testCarErrorDriverStatusTimeOut)
    {
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
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_car(2100));
    }

// Unit test for truck part
    
    TEST(DriverManagerTest, testTruckNormalDriverStatus)
    {
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
    }

    TEST(DriverManagerTest, testTruckSsc_0_Lidar1_1_Lidar2_1_Gps_1)
    {
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
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_0_Lidar2_0_Gps_0)
    {
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
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_0_Lidar2_0_Gps_1)
    {
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
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_0_Lidar2_1_Gps_0)
    {
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
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_0_Lidar2_1_Gps_1)
    {
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
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_1_Lidar2_0_Gps_0)
    {
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
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_0_g_0", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_1_Lidar2_0_Gps_1)
    {
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
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_0_g_1", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_1_Lidar2_1_Gps_0)
    {
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
    }

    TEST(DriverManagerTest, testTruckDriverStatusTimeout)
    {
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

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_truck(2001));
    }

}
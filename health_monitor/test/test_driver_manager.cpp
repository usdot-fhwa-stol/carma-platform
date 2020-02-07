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
    
    TEST(DriverManagerTest, testNormalDriverStatus)
    {
        std::vector<std::string> required_drivers{"controller", "lidar"};
        DriverManager dm(required_drivers, 1000L);
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
        EXPECT_EQ(true, dm.are_critical_drivers_operational(1500));
    }

    TEST(DriverManagerTest, testDriverStatusTimeout)
    {
        std::vector<std::string> required_drivers{"controller", "lidar"};
        DriverManager dm(required_drivers, 1000L);
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
        EXPECT_EQ(false, dm.are_critical_drivers_operational(2001));
    }

    TEST(DriverManagerTest, testErrorDriverStatus1)
    {
        std::vector<std::string> required_drivers{"controller", "lidar"};
        DriverManager dm(required_drivers, 1000L);
        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::FAULT;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);
        EXPECT_EQ(false, dm.are_critical_drivers_operational(1500));
    }

    TEST(DriverManagerTest, testErrorDriverStatus2)
    {
        std::vector<std::string> required_drivers{"controller", "lidar"};
        DriverManager dm(required_drivers, 1000L);
        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);
        EXPECT_EQ(false, dm.are_critical_drivers_operational(1500));
    }

    TEST(DriverManagerTest, testMissingCriticalDriver)
    {
        std::vector<std::string> required_drivers{"controller", "lidar"};
        DriverManager dm(required_drivers, 1000L);
        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        cav_msgs::DriverStatus msg2;
        msg2.radar = true;
        msg2.name = "radar";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);
        EXPECT_EQ(false, dm.are_critical_drivers_operational(1500));
    }

}
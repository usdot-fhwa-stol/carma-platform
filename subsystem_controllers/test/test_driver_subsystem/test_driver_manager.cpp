/*
 * Copyright (C) 2023 LEIDOS.
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

#include <subsystem_controllers/drivers_controller/driver_manager.hpp>
#include <gtest/gtest.h>


namespace subsystem_controllers
{
    TEST(DriverManagerTest, testCarNormalDriverStatus)
    {
        // inordinary case where no critical drivers are specified
        DriverManager dm0({"ssc"}, {}, 0);
        carma_driver_msgs::msg::DriverStatus msg0;
        msg0.controller = true;
        msg0.name = "controller";
        msg0.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        
        auto msg0_pointer = std::make_shared<carma_driver_msgs::msg::DriverStatus>(msg0);
        
        dm0.update_driver_status(msg0_pointer, 1000);
        
        EXPECT_EQ("s_0", dm0.are_critical_drivers_operational(1500));
        
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, camera_drivers, 1000L);
        
        carma_driver_msgs::msg::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(new carma_driver_msgs::msg::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
       carma_driver_msgs::msg::DriverStatus msg2;
        msg2.gnss = true;
        msg2.name = "camera";
        msg2.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg2_pointer(new carma_driver_msgs::msg::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        EXPECT_EQ("s_1_c_1", dm.are_critical_drivers_operational(1500));
    }

    TEST(DriverManagerTest, testSsc_1_Camera_0)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};


        DriverManager dm(required_drivers,camera_drivers,  1000L);

        carma_driver_msgs::msg::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(new carma_driver_msgs::msg::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        

        carma_driver_msgs::msg::DriverStatus msg2;
        msg2.gnss = true;
        msg2.name = "camera";
        msg2.status = carma_driver_msgs::msg::DriverStatus::OFF;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg2_pointer(new carma_driver_msgs::msg::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        EXPECT_EQ("s_1_c_0", dm.are_critical_drivers_operational(1500));
    }

    TEST(DriverManagerTest, testCarErrorDriverStatusTimeOut)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, camera_drivers, 1000L);

        carma_driver_msgs::msg::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(new carma_driver_msgs::msg::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);

        carma_driver_msgs::msg::DriverStatus msg2;
        msg2.gnss = true;
        msg2.name = "camera";
        msg2.status = carma_driver_msgs::msg::DriverStatus::DEGRADED;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg2_pointer(new carma_driver_msgs::msg::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational(2100));
    }

    TEST(DriverManagerTest, testCarHandleSpinDriversReady)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers,camera_drivers, 1000L);

        carma_driver_msgs::msg::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(new carma_driver_msgs::msg::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        

        carma_driver_msgs::msg::DriverStatus msg2;
        msg2.gnss = true;
        msg2.name = "camera";
        msg2.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg2_pointer(new carma_driver_msgs::msg::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        carma_msgs::msg::SystemAlert alert;
        alert=dm.handle_spin(1500,150,750);

        EXPECT_EQ(5, alert.type);
    }

    TEST(DriverManagerTest, testCarHandleSpinFatalSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers, camera_drivers, 1000L);

        carma_driver_msgs::msg::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = carma_driver_msgs::msg::DriverStatus::OFF;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(new carma_driver_msgs::msg::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        

        carma_driver_msgs::msg::DriverStatus msg2;
        msg2.gnss = true;
        msg2.name = "camera";
        msg2.status = carma_driver_msgs::msg::DriverStatus::DEGRADED;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg2_pointer(new carma_driver_msgs::msg::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        
        carma_msgs::msg::SystemAlert alert;
        alert=dm.handle_spin(1500,150,750);

        EXPECT_EQ(6, alert.type);
    }

    TEST(DriverManagerTest, testCarHandleSpinFatalUnknownInside)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers,camera_drivers, 1000L);
        
        carma_msgs::msg::SystemAlert alert;
        alert=dm.handle_spin(1500,150,750);

        EXPECT_EQ(6, alert.type);
    }  


    TEST(DriverManagerTest, testCarHandleSpinNotReadyCase1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers,camera_drivers, 1000L);

        carma_driver_msgs::msg::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = carma_driver_msgs::msg::DriverStatus::OFF;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(new carma_driver_msgs::msg::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        

        carma_driver_msgs::msg::DriverStatus msg2;
        msg2.gnss = true;
        msg2.name = "camera";
        msg2.status = carma_driver_msgs::msg::DriverStatus::DEGRADED;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg2_pointer(new carma_driver_msgs::msg::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        
        carma_msgs::msg::SystemAlert alert;
        alert=dm.handle_spin(1500,1000,750);

        EXPECT_EQ(4, alert.type);
    }

    TEST(DriverManagerTest, testCarHandleSpinNotReadyCase2)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers,camera_drivers, 1000L);

        carma_driver_msgs::msg::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = carma_driver_msgs::msg::DriverStatus::OFF;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(new carma_driver_msgs::msg::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);

        carma_driver_msgs::msg::DriverStatus msg2;
        msg2.gnss = true;
        msg2.name = "camera";
        msg2.status = carma_driver_msgs::msg::DriverStatus::DEGRADED;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg2_pointer(new carma_driver_msgs::msg::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        carma_msgs::msg::SystemAlert alert;
        alert=dm.handle_spin(1500,5000,750);

        EXPECT_EQ(4, alert.type);
    }

    TEST(DriverManagerTest, testCarTruckHandleSpinFatalUnknown)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> camera_drivers{"camera"};

        DriverManager dm(required_drivers,camera_drivers, 1000L);

        carma_driver_msgs::msg::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = carma_driver_msgs::msg::DriverStatus::OFF;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(new carma_driver_msgs::msg::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);

        carma_driver_msgs::msg::DriverStatus msg2;
        msg2.gnss = true;
        msg2.name = "camera";
        msg2.status = carma_driver_msgs::msg::DriverStatus::OFF;
        carma_driver_msgs::msg::DriverStatus::SharedPtr msg2_pointer(new carma_driver_msgs::msg::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        carma_msgs::msg::SystemAlert alert;
        alert=dm.handle_spin(1500,150,750);

        EXPECT_EQ(6, alert.type);
    } 
}
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

#include <gtest/gtest.h>

#include <subsystem_controllers/drivers_controller/ssc_driver_manager.hpp>

namespace subsystem_controllers
{
TEST(DriverManagerTest, testCarNormalDriverStatus)
{
  // inordinary case where no critical drivers are specified
  SSCDriverManager dm0("ssc", 0);
  carma_driver_msgs::msg::DriverStatus msg0;
  msg0.controller = true;
  msg0.name = "controller";
  msg0.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;

  auto msg0_pointer = std::make_shared<carma_driver_msgs::msg::DriverStatus>(msg0);

  dm0.update_driver_status(msg0_pointer, 1000);

  EXPECT_FALSE(dm0.is_ssc_driver_operational(1500));

  std::string ssc_driver_name = "controller";

  carma_driver_msgs::msg::DriverStatus msg1;
  msg1.controller = true;
  msg1.name = "controller";
  msg1.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
  carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(
    new carma_driver_msgs::msg::DriverStatus(msg1));
  dm0.update_driver_status(msg1_pointer, 1000);

  // the name of the controller did not match, so still unoperational
  EXPECT_FALSE(dm0.is_ssc_driver_operational(1500));
}

TEST(DriverManagerTest, testCarErrorDriverStatusTimeOut)
{
  std::string ssc_driver_name = "controller";

  SSCDriverManager dm(ssc_driver_name, 1000L);

  carma_driver_msgs::msg::DriverStatus msg1;
  msg1.controller = true;
  msg1.name = "controller";
  msg1.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
  carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(
    new carma_driver_msgs::msg::DriverStatus(msg1));
  dm.update_driver_status(msg1_pointer, 1000);

  EXPECT_FALSE(dm.is_ssc_driver_operational(2100));
}

TEST(DriverManagerTest, testCarHandleSpinDriversReady)
{
  std::string ssc_driver_name = "controller";

  SSCDriverManager dm(ssc_driver_name, 1000L);

  carma_driver_msgs::msg::DriverStatus msg1;
  msg1.controller = true;
  msg1.name = "controller";
  msg1.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
  carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(
    new carma_driver_msgs::msg::DriverStatus(msg1));
  dm.update_driver_status(msg1_pointer, 1000);

  carma_msgs::msg::SystemAlert alert;
  alert = dm.get_latest_system_alert(1500, 150, 750);

  EXPECT_EQ(5, alert.type);
}

TEST(DriverManagerTest, testCarHandleSpinFatalSscWorking)
{
  std::string ssc_driver_name = "controller";

  SSCDriverManager dm(ssc_driver_name, 1000L);

  carma_driver_msgs::msg::DriverStatus msg1;
  msg1.controller = true;
  msg1.name = "controller";
  msg1.status = carma_driver_msgs::msg::DriverStatus::OFF;
  carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(
    new carma_driver_msgs::msg::DriverStatus(msg1));
  dm.update_driver_status(msg1_pointer, 1000);

  carma_msgs::msg::SystemAlert alert;
  alert = dm.get_latest_system_alert(1500, 150, 750);

  EXPECT_EQ(6, alert.type);
}

TEST(DriverManagerTest, testCarHandleSpinFatalUnknownInside)
{
  std::string ssc_driver_name = "controller";
  SSCDriverManager dm(ssc_driver_name, 1000L);

  carma_msgs::msg::SystemAlert alert;
  alert = dm.get_latest_system_alert(1500, 150, 750);

  EXPECT_EQ(6, alert.type);
}

TEST(DriverManagerTest, testCarHandleSpinNotReadyCase1)
{
  std::string ssc_driver_name = "controller";

  SSCDriverManager dm(ssc_driver_name, 1000L);

  carma_driver_msgs::msg::DriverStatus msg1;
  msg1.controller = true;
  msg1.name = "controller";
  msg1.status = carma_driver_msgs::msg::DriverStatus::OFF;
  carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(
    new carma_driver_msgs::msg::DriverStatus(msg1));
  dm.update_driver_status(msg1_pointer, 1000);

  carma_msgs::msg::SystemAlert alert;
  alert = dm.get_latest_system_alert(1500, 1000, 750);

  EXPECT_EQ(4, alert.type);
}

TEST(DriverManagerTest, testCarHandleSpinNotReadyCase2)
{
  std::string ssc_driver_name = "controller";

  SSCDriverManager dm(ssc_driver_name, 1000L);

  carma_driver_msgs::msg::DriverStatus msg1;
  msg1.controller = true;
  msg1.name = "controller";
  msg1.status = carma_driver_msgs::msg::DriverStatus::OFF;
  carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(
    new carma_driver_msgs::msg::DriverStatus(msg1));
  dm.update_driver_status(msg1_pointer, 1000);

  carma_msgs::msg::SystemAlert alert;
  alert = dm.get_latest_system_alert(1500, 5000, 750);

  EXPECT_EQ(4, alert.type);
}

TEST(DriverManagerTest, testCarTruckHandleSpinFatalUnknown)
{
  std::string ssc_driver_name = "controller";

  SSCDriverManager dm(ssc_driver_name, 1000L);

  carma_driver_msgs::msg::DriverStatus msg1;
  msg1.controller = true;
  msg1.name = "controller";
  msg1.status = carma_driver_msgs::msg::DriverStatus::OFF;
  carma_driver_msgs::msg::DriverStatus::SharedPtr msg1_pointer(
    new carma_driver_msgs::msg::DriverStatus(msg1));
  dm.update_driver_status(msg1_pointer, 1000);

  carma_msgs::msg::SystemAlert alert;
  alert = dm.get_latest_system_alert(1500, 150, 750);

  EXPECT_EQ(6, alert.type);
}
}  // namespace subsystem_controllers

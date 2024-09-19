#pragma once

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

#include <gtest/gtest_prod.h>
#include <rmw/types.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

#include "entry.hpp"
#include <ros2_lifecycle_manager/lifecycle_manager_interface.hpp>

#include <carma_driver_msgs/msg/driver_status.hpp>
#include <carma_msgs/msg/system_alert.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/srv/get_plugin_api.hpp>
#include <carma_planning_msgs/srv/plugin_activation.hpp>
#include <carma_planning_msgs/srv/plugin_list.hpp>

namespace subsystem_controllers
{
/**
 * \brief The SSCDriverManager serves as a component to manage ROS1 SSC Driver in CARMA which is
 * primarily in ROS2
 */
class SSCDriverManager
{
public:
  /*!
   * \brief Default constructor for SSCDriverManager with driver_timeout_ = 1000ms
   */
  SSCDriverManager();

  /**
   * \brief Constructor for SSCDriverManager
   *
   * \param ssc_driver_name The driver name which will be treated as required. A failure in
   * this plugin will result in an exception
   * \param driver_timeout The timeout threshold for the driver
   */
  SSCDriverManager(const std::string & ssc_driver_name, const long driver_timeout);

  /*!
   * \brief Update driver status
   */
  void update_driver_status(
    const carma_driver_msgs::msg::DriverStatus::SharedPtr msg, long current_time);

  /*!
   * \brief Check if all critical drivers are operational
   */
  bool is_ssc_driver_operational(long current_time);

  /*!
   * \brief Handle the spin and publisher
   */
  carma_msgs::msg::SystemAlert get_latest_system_alert(
    long time_now, long start_up_timestamp, long startup_duration);

protected:
  // list of critical drivers
  std::string ssc_driver_name_ = "";

  //! Latest SSC Status entry to keep track
  std::shared_ptr<Entry> latest_ssc_status_entry_ = std::make_shared<Entry>();

  // timeout for critical driver timeout in milliseconds
  long driver_timeout_ = 1000;

  bool starting_up_ = true;

  FRIEND_TEST(DriverManagerTest, testCarTruckHandleSpinFatalUnknown);
};
}  // namespace subsystem_controllers

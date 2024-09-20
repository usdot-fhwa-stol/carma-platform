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

#include "subsystem_controllers/drivers_controller/ssc_driver_manager.hpp"

#include <boost/algorithm/string.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <lifecycle_msgs/msg/state.hpp>

using std_msec = std::chrono::milliseconds;

namespace subsystem_controllers
{
SSCDriverManager::SSCDriverManager() {}

SSCDriverManager::SSCDriverManager(const std::string & ssc_driver_name, const long driver_timeout)
: ssc_driver_name_(ssc_driver_name), driver_timeout_(driver_timeout)
{
}

carma_msgs::msg::SystemAlert SSCDriverManager::get_latest_system_alert(
  long time_now, long start_up_timestamp, long startup_duration)
{
  carma_msgs::msg::SystemAlert alert;

  bool ssc_is_operational = is_ssc_driver_operational(time_now);
  if (ssc_is_operational) {
    starting_up_ = false;
    alert.description = "All essential drivers are ready";
    alert.type = carma_msgs::msg::SystemAlert::DRIVERS_READY;
    return alert;
  } else if (starting_up_ && (time_now - start_up_timestamp <= startup_duration)) {
    alert.description = "System is starting up...";
    alert.type = carma_msgs::msg::SystemAlert::NOT_READY;
    return alert;
  } else if (!ssc_is_operational) {
    alert.description = "SSC Failed";
    alert.type = carma_msgs::msg::SystemAlert::SHUTDOWN;
    return alert;
  } else {
    alert.description = "Unknown problem assessing SSC driver availability";
    alert.type = carma_msgs::msg::SystemAlert::FATAL;
    return alert;
  }
}

void SSCDriverManager::update_driver_status(
  const carma_driver_msgs::msg::DriverStatus::SharedPtr msg, long current_time)
{
  // update driver status is only called in response to a message received on driver_discovery.
  // This topic is only being published in ros1. Check only SSC

  if (ssc_driver_name_ == msg->name) {
    Entry driver_status(
      msg->status == carma_driver_msgs::msg::DriverStatus::OPERATIONAL ||
        msg->status == carma_driver_msgs::msg::DriverStatus::DEGRADED,
      msg->name, current_time);

    latest_ssc_status_entry_ = std::make_shared<Entry>(driver_status);
  }
}

bool SSCDriverManager::is_ssc_driver_operational(long current_time)
{
  // Manual disable of ssc entry in case ssc wrapper is in ros2
  if (ssc_driver_name_.empty()) {
    return true;
  }

  // No entry recorded, then return false
  if (!latest_ssc_status_entry_) {
    return false;
  }

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("subsystem_controller"),
    "latest_ssc_status_entry_->name_: "
      << latest_ssc_status_entry_->name_ << ", current_time: " << current_time
      << ", latest_ssc_status_entry_->timestamp_: " << latest_ssc_status_entry_->timestamp_
      << ", difference: " << current_time - (latest_ssc_status_entry_->timestamp_));

  // Detect whether if available or timed out
  if (
    (!latest_ssc_status_entry_->available_) ||
    (current_time - latest_ssc_status_entry_->timestamp_ > driver_timeout_)) {
    return false;
  } else {
    return true;
  }
}

}  // namespace subsystem_controllers

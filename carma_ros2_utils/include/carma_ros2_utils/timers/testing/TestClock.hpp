#pragma once
/*
 * Copyright (C) 2022 LEIDOS.
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

#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include "../Timer.hpp"

namespace carma_ros2_utils
{
namespace timers
{
namespace testing
{

/**
 * @brief Implementation of the Clock interface that is targeted for use in Unit Testing.
 *        Internally rclcpp::Time objects are used for getting the clock time meaning this class does support simulated
 * time and equivalent of ROS1 ros::Time::setNow() semantics. By default internal time is RCL_SYSTEM_TIME unless changed by setClockType()
 * This class should NOT be used in production code as it does not provide the
 * same threading behavior as rclcpp::Timer.
 */
class TestClock
{

public:
  /**
   * @brief Constructor
   */
  TestClock(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  rclcpp::Time now();

  /**
   * @brief Sets the simulated time to given time. Clock type will be created from internal saved rcl_clock_type_t.
   */
  void setNow(const rclcpp::Time& time);

  /**
   * @brief Sets the rcl_clock_type_t to use for when returning current time.
   */
  void setClockType(rcl_clock_type_t clock_type);

  typedef std::shared_ptr<TestClock> SharedPtr;

private:
  rclcpp::Time current_time_{0, 0};
  rcl_clock_type_t clock_type_ ;

};
}  // namespace testing
}  // namespace timers
}  // namespace carma_ros2_utils
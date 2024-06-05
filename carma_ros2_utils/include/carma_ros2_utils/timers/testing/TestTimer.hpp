#pragma once
/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include "TestClock.hpp"
#include "../Timer.hpp"

namespace carma_ros2_utils
{
namespace timers
{
namespace testing
{
/**
 * @brief Implementation of the Timer interface that is targeted for use in Unit Testing.
 *        Internally rclcpp::Time objects are used for getting the clock time meaning this class does support simulated
 * time and rclcpp::Time::setNow() semantics. This class should NOT be used in production code as it does not provide the
 * same threading behavior as rclcpp::Timer.
 *
 * TestTimers utilize a separate std::thread to generate and process the callback.
 */
class TestTimer : public Timer
{

private:
  std::mutex clock_mutex_;

  std::function<void()> callback_;

  rclcpp::Time start_time_ = rclcpp::Time(0);
  rclcpp::Duration duration_ = rclcpp::Duration(std::chrono::nanoseconds{0});
  carma_ros2_utils::timers::testing::TestClock::SharedPtr clock_; //! Interface used for accessing current time from rclcpp
  bool oneshot_ = false;

  std::thread timer_thread_;
  std::mutex timer_mutex_;

  std::atomic_bool running_ = ATOMIC_VAR_INIT(false);

  void startImpl();  // Implementation of start to prevent deadlock

public:
  TestTimer(carma_ros2_utils::timers::testing::TestClock::SharedPtr clock);
  ~TestTimer();

  rclcpp::Time getTime();

  //// Overrides
  void initializeTimer(rclcpp::Duration duration, std::function<void()> callback,
                       bool oneshot = false, bool autostart = true);

  void start() override;

  void stop() override;
};
}  // namespace testing
}  // namespace timers
}  // namespace carma_ros2_utils

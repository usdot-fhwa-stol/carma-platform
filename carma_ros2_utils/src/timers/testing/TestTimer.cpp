
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
#include <iostream>
#include <carma_ros2_utils/timers/testing/TestTimer.hpp>

namespace carma_ros2_utils
{
namespace timers
{
namespace testing
{

rclcpp::Time TestTimer::getTime()
{
  const std::lock_guard<std::mutex> lock(clock_mutex_);
  return clock_->now();
}

TestTimer::TestTimer(carma_ros2_utils::timers::testing::TestClock::SharedPtr clock) : clock_(clock) {}

TestTimer::~TestTimer()
{
  {
    const std::lock_guard<std::mutex> lock(timer_mutex_);
    running_.store(false);  // Close thread
  }
  if (timer_thread_.joinable())
  {
    timer_thread_.join();
  }
}

//// Overrides
void TestTimer::initializeTimer(rclcpp::Duration duration, std::function<void()> callback,
                                bool oneshot, bool autostart)
{
  const std::lock_guard<std::mutex> lock(timer_mutex_);
  if (running_.load())
  {
    throw std::invalid_argument("Tried to initialize an already running timer");
  }
  callback_ = callback;
  duration_ = duration;
  oneshot_ = oneshot;

  if (autostart)
  {
    startImpl();
  }
}

void TestTimer::startImpl()
{
  if (running_.load())
  {
    std::cerr << "Duplicate call to start on running thread" << std::endl;
    return;
  }
  start_time_ = getTime();

  // Before creating a new thread verify that the current thread is properly closed
  if (timer_thread_.joinable()) {
    timer_thread_.join();
  }

  running_.store(true);

  timer_thread_ = std::thread([this]() {
    while (running_.load())
    {
      const std::lock_guard<std::mutex> lock_timer(timer_mutex_);
      if (!running_.load()) { // Need to check running flag again after mutex loc to support start/stop operations
        continue;
      }
      bool triggerCallback = false;

      // Check if the duration of the timer has expired
      triggerCallback = getTime() >= start_time_ + duration_;

      if (triggerCallback)
      {
        callback_();
        start_time_ = getTime();  // Reset the countdown
        if (oneshot_)
        {  // If only running once, shutdown the thread
          running_.store(false);
        }
      }
      auto period = std::chrono::milliseconds(1);
      std::this_thread::sleep_for(period);
    }
  });
}

void TestTimer::start()
{
  const std::lock_guard<std::mutex> lock(timer_mutex_);
  startImpl();
}

void TestTimer::stop()
{
  const std::lock_guard<std::mutex> lock(timer_mutex_);
  running_.store(false);
}
}  // namespace testing
}  // namespace timers
}  // namespace carma_ros2_utils
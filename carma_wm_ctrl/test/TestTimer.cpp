
/*
 * Copyright (C) 2020 LEIDOS.
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
#include "TestTimer.h"

namespace carma_wm_ctrl
{
// Initialize static variables
std::mutex TestTimer::clock_mutex_;

ros::Time TestTimer::getTime()
{
  const std::lock_guard<std::mutex> lock(clock_mutex_);
  return ros::Time::now();
}

TestTimer::TestTimer(){};
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
};

//// Overrides
void TestTimer::initializeTimer(ros::Duration duration, std::function<void(const ros::TimerEvent&)> callback,
                                bool oneshot, bool autostart)
{
  const std::lock_guard<std::mutex> lock(timer_mutex_);
  if (running_.load())
  {
    throw std::invalid_argument("Tried to intialize an already running timer");
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

  running_.store(true);

  timer_thread_ = std::thread([this]() {
    while (running_.load())
    {
      const std::lock_guard<std::mutex> lock_timer(timer_mutex_);
      bool triggerCallback = false;

      // Check if the duration of the timer has expired
      triggerCallback = getTime() >= start_time_ + duration_;

      if (triggerCallback)
      {
        ros::TimerEvent event;
        callback_(event);
        start_time_ = getTime();  // Reset the countdown
        if (oneshot_)
        {  // If only running once, shutdown the thread
          running_.store(false);
        }
      }
      auto period = std::chrono::milliseconds(10);
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
}  // namespace carma_wm_ctrl
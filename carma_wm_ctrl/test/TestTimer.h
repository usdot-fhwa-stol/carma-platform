#pragma once
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
#include <ros/time.h>
#include <carma_wm_ctrl/Timer.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>

namespace carma_wm_ctrl
{
// Timer for unit testing which is implemented using ros::Time and std::thread instead of ros::Timer which requires a
// node handle to build
class TestTimer : public Timer
{
public:
  static ros::Time getTime();

private:
  static std::mutex clock_mutex_;

  std::function<void(const ros::TimerEvent&)> callback_;

  ros::Time start_time_ = ros::Time(0);
  ros::Duration duration_ = ros::Duration(0);
  bool oneshot_ = false;

  std::thread timer_thread_;
  std::mutex timer_mutex_;

  std::atomic_bool running_ = ATOMIC_VAR_INIT(false);

  void startImpl();  // Implementation of start to prevent deadlock

public:
  TestTimer();
  ~TestTimer();

  //// Overrides
  void initializeTimer(ros::Duration duration, std::function<void(const ros::TimerEvent&)> callback,
                       bool oneshot = false, bool autostart = true);

  void start() override;

  void stop() override;
};
}  // namespace carma_wm_ctrl
#pragma once
/*
 * Copyright (C) 2020-2021 LEIDOS.
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
#include <ros/timer.h>
#include "Timer.h"

namespace carma_utils
{
namespace timers
{
/**
 * @brief TimerFactory provides an interface for factory classes that can build Timer objects.
 *        This class provides a mechanism for separating the initialization of ROS style timers from the
 * implementation.
 */
class TimerFactory
{
public:
  /**
   * @brief Destructor
   */
  virtual ~TimerFactory(){};

  /**
   * @brief Function to build a timer which will wait the provided duration before triggering the provided callback
   *
   * @param duration The duration the timer will wait for before a callback is triggered
   * @param callback The callback to trigger after duration has elapsed
   * @param oneshot If true the timer will only trigger one. If false it will trigger repeatedly with duration length
   * increments
   * @param autostart If true the timer will immediately start after this function is called. Otherwise the start()
   * function must be called
   *
   * @return A unique pointer to a Timer object which was intialized using the provided parameters
   */
  virtual std::unique_ptr<Timer> buildTimer(uint32_t id, ros::Duration duration,
                                            std::function<void(const ros::TimerEvent&)> callback, bool oneshot = false,
                                            bool autostart = true) = 0;
};
}  // namespace timers
}  // namespace carma_utils
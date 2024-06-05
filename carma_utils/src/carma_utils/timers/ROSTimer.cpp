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
#include <carma_utils/timers/ROSTimer.h>

// <SONAR_IGNORE_START> // Disable sonar cloud analysis for ROS dependant logic
namespace carma_utils
{
namespace timers
{
ROSTimer::~ROSTimer(){};

void ROSTimer::initializeTimer(ros::Duration duration, std::function<void(const ros::TimerEvent&)> callback,
                               bool oneshot, bool autostart)
{
  timer_.stop();
  timer_ = nh_.createTimer(duration, callback, oneshot, autostart);
}

void ROSTimer::start()
{
  timer_.start();
}

void ROSTimer::stop()
{
  timer_.stop();
}
}  // namespace timers
}  // namespace carma_utils
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
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <ros/time.h>
#include "Timer.h"

namespace carma_utils
{
namespace timers
{
/**
 * @brief Implementation of the Timer interface that uses ros::Timer objects on the backend.
 *
 */
class ROSTimer : public Timer
{
  ros::Timer timer_;
  ros::NodeHandle nh_;

public:
  ~ROSTimer();

  //// Overrides
  void initializeTimer(ros::Duration duration, std::function<void(const ros::TimerEvent&)> callback,
                       bool oneshot = false, bool autostart = true) override;

  void start() override;

  void stop() override;
};
}  // namespace timers
}  // namespace carma_utils
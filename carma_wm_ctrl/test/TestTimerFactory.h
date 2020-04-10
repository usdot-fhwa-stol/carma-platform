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
#include <carma_wm_ctrl/TimerFactory.h>
#include <carma_wm_ctrl/Timer.h>
#include "TestTimer.h"

namespace carma_wm_ctrl
{
// Timer factory for unit testing
class TestTimerFactory : public TimerFactory
{
public:
  TestTimerFactory(){};
  ~TestTimerFactory(){};

  //// Overrides
  std::unique_ptr<Timer> buildTimer(uint32_t id, ros::Duration duration,
                                    std::function<void(const ros::TimerEvent&)> callback, bool oneshot = false,
                                    bool autostart = true) override
  {
    std::unique_ptr<Timer> timer = std::make_unique<TestTimer>();
    timer->setId(id);
    timer->initializeTimer(duration, callback, oneshot, autostart);
    return timer;
  }
};
}  // namespace carma_wm_ctrl
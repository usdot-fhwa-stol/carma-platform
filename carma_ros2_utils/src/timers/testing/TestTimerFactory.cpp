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
#include <carma_ros2_utils/timers/testing/TestTimerFactory.hpp>
#include <iostream>
namespace carma_ros2_utils
{
namespace timers
{
namespace testing
{
TestTimerFactory::~TestTimerFactory(){}

void TestTimerFactory::setNow(const rclcpp::Time& current_time)
{
  if (!clock_) // lazy initialization of clock
  {
    clock_ = std::make_shared<TestClock>();
  }
  clock_->setNow(current_time);
}

rclcpp::Time TestTimerFactory::now()
{
  if (!clock_) // lazy initialization of clock
  {
    clock_ = std::make_shared<TestClock>();
  }
  return clock_->now();
}

std::unique_ptr<Timer> TestTimerFactory::buildTimer(uint32_t id, rclcpp::Duration duration,
                                                    std::function<void()> callback, bool oneshot,
                                                    bool autostart)
{
  if (!clock_) // lazy initialization of clock
  {
    clock_ = std::make_shared<TestClock>();
  }
  std::unique_ptr<Timer> timer(new TestTimer(clock_));

  timer->setId(id);
  timer->initializeTimer(duration, callback, oneshot, autostart);

  return timer;
}
}  // namespace testing
}  // namespace timers
}  // namespace carma_ros2_utils
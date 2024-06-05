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

#include <gmock/gmock.h>
#include <ros/ros.h>
#include <carma_utils/timers/testing/TestTimer.h>
#include <carma_utils/timers/testing/TestTimerFactory.h>
#include <carma_utils/testing/TestHelpers.h>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_utils
{
namespace timers
{
namespace testing
{
TEST(TestingTimers, buildTimer)
{
  ros::Time::setNow(ros::Time(0));  // Set current time

  std::unique_ptr<carma_utils::timers::TimerFactory> factory(new TestTimerFactory());  // Verify casting behavior
  uint32_t id = 1;
  std::atomic<int> call_count(0);
  std::unique_ptr<carma_utils::timers::Timer> timer = factory->buildTimer(
      id, ros::Duration(1.0), [&](const ros::TimerEvent&) -> void { call_count++; }, true);

  // Test setting and getting the timer id
  ASSERT_EQ(id, timer->getId());
  timer->setId(2);
  ASSERT_EQ(2, timer->getId());

  // Test the timing behavior
  ASSERT_EQ(0, call_count.load());
  ros::Time::setNow(ros::Time(0.9));
  ASSERT_FALSE(carma_utils::testing::waitForEqOrTimeout(1.0, 1, call_count));  // Timer should not be called yet as we
                                                                               // are using simulated time
  ASSERT_EQ(0, call_count.load());
  ros::Time::setNow(ros::Time(1.2));
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, 1, call_count));
  ros::Time::setNow(ros::Time(2.2));
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, 1, call_count));  // One shot should not trigger again

  // Test autostart
  id = 3;
  call_count.store(0);
  timer = factory->buildTimer(
      id, ros::Duration(1.0), [&](const ros::TimerEvent&) -> void { call_count++; }, true, false);

  ASSERT_EQ(0, call_count.load());
  ros::Time::setNow(ros::Time(3.2));
  ASSERT_FALSE(carma_utils::testing::waitForEqOrTimeout(2.0, 1, call_count));  // Give some time for thread to hopefully
                                                                               // NOT trigger
  ASSERT_EQ(0, call_count.load());  // Timer should not have started so callcount should still be 0
  timer->start();
  ros::Time::setNow(ros::Time(4.2));
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, 1, call_count));  // Expect timer to trigger after starting

  // Test repeated operation and stop
  id = 4;
  call_count.store(0);
  timer = factory->buildTimer(
      id, ros::Duration(1.0), [&](const ros::TimerEvent&) -> void { call_count++; }, false, true);

  ASSERT_EQ(0, call_count.load());
  ros::Time::setNow(ros::Time(5.2));
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, 1, call_count));
  ros::Time::setNow(ros::Time(6.2));
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, 2, call_count));
  ros::Time::setNow(ros::Time(7.2));
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, 3, call_count));
  timer->stop();  // Test stopping behavior
  ros::Time::setNow(ros::Time(8.2));
  ASSERT_FALSE(carma_utils::testing::waitForEqOrTimeout(2.0, 4, call_count));  // Expect timer to NOT trigger after
                                                                               // stopping
  ASSERT_EQ(3, call_count.load());

  timer->start();
  timer->start(); // Run start twice; This requires visual inspection of output to verify

  // // Verify duplicate initialization throw
  ASSERT_THROW(timer->initializeTimer(ros::Duration(1.0), [&](const ros::TimerEvent&){}), std::invalid_argument);

}
}  // namespace testing
}  // namespace timers
}  // namespace carma_utils
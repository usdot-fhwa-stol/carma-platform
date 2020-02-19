/*
 * Copyright (C) 2019 LEIDOS.
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
#include <carma_wm_ctrl/GeofenceSchedule.h>
#include <carma_wm_ctrl/Geofence.h>
#include <carma_wm_ctrl/GeofenceScheduler.h>
#include <carma_wm_ctrl/ROSTimerFactory.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include "TestHelpers.h"
#include "TestTimer.h"
#include "TestTimerFactory.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm_ctrl
{
TEST(GeofenceScheduler, Constructor)
{
  GeofenceScheduler scheduler(std::make_unique<TestTimerFactory>());  // Create scheduler with test timers. Having this
                                                                      // check helps verify that the timers do not crash
                                                                      // on destruction
}

TEST(GeofenceScheduler, addGeofence)
{
  // Test adding then evaulate if the calls to active and inactive are done correctly
  // Finally test cleaing the timers
  Geofence gf;
  gf.id_ = 1;
  gf.schedule =
      GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 6
                       ros::Time(8),
                       ros::Duration(2),    // Start's at 2
                       ros::Duration(5.5),  // Ends at by 5.5
                       ros::Duration(1),    // Duration of 1 and interval of two so active durations are (2-3 and 4-5)
                       ros::Duration(2));
  ros::Time::setNow(ros::Time(0));  // Set current time

  GeofenceScheduler scheduler(std::make_unique<TestTimerFactory>());  // Create scheduler

  std::atomic<uint32_t> active_call_count(0);
  std::atomic<uint32_t> inactive_call_count(0);
  std::atomic<uint32_t> last_active_gf(0);
  std::atomic<uint32_t> last_inactive_gf(0);
  scheduler.onGeofenceActive([&](const Geofence& gf) {
    active_call_count.store(active_call_count.load() + 1);
    last_active_gf.store(gf.id_);
  });

  scheduler.onGeofenceInactive([&](const Geofence& gf) {
    inactive_call_count.store(inactive_call_count.load() + 1);
    last_inactive_gf.store(gf.id_);
  });

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  scheduler.addGeofence(gf);

  ros::Time::setNow(ros::Time(1.0));  // Set current time

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  ros::Time::setNow(ros::Time(2.1));  // Set current time

  ASSERT_TRUE(carma_wm::waitForEqOrTimeout(10.0, 1, last_active_gf));
  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  ros::Time::setNow(ros::Time(3.1));  // Set current time

  ASSERT_TRUE(carma_wm::waitForEqOrTimeout(10.0, 1, last_inactive_gf));
  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(1, last_active_gf.load());

  ros::Time::setNow(ros::Time(3.5));  // Set current time

  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(1, last_active_gf.load());
  ASSERT_EQ(1, last_inactive_gf.load());

  ros::Time::setNow(ros::Time(4.2));  // Set current time

  ASSERT_TRUE(carma_wm::waitForEqOrTimeout(10.0, 2, active_call_count));
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(1, last_active_gf.load());

  ros::Time::setNow(ros::Time(5.5));  // Set current time

  ASSERT_TRUE(carma_wm::waitForEqOrTimeout(10.0, 2, inactive_call_count));
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(1, last_active_gf.load());

  ros::Time::setNow(ros::Time(9.5));  // Set current time

  ASSERT_EQ(2, inactive_call_count.load());
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(1, last_active_gf.load());
  ASSERT_EQ(1, last_inactive_gf.load());

  // Basic check that expired geofence is not added
  Geofence gf2 = gf;
  gf2.id_ = 2;
  scheduler.addGeofence(gf2);

  ros::Time::setNow(ros::Time(11.0));  // Set current time

  carma_wm::waitForEqOrTimeout(3.0, 10, inactive_call_count);  // Let some time pass just in case
  ASSERT_EQ(2, inactive_call_count.load());
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(1, last_active_gf.load());
  ASSERT_EQ(1, last_inactive_gf.load());
}

}  // namespace carma_wm_ctrl
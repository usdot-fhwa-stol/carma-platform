/*
 * Copyright (C) 2022 LEIDOS.
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

#include <gtest/gtest.h>
#include <carma_wm_ctrl/GeofenceSchedule.hpp>

namespace carma_wm_ctrl
{
TEST(GeofenceSchedule, scheduleStarted)
{
  GeofenceSchedule sch;
  sch.schedule_start_ = rclcpp::Time(0);
  sch.schedule_end_ = rclcpp::Time(1);
  sch.control_start_ = rclcpp::Duration(0);
  sch.control_duration_ = rclcpp::Duration(1);
  sch.control_offset_ = rclcpp::Duration(0);
  sch.control_span_ = rclcpp::Duration(1);
  sch.control_period_ = rclcpp::Duration(2);

  ASSERT_TRUE(sch.scheduleStarted(rclcpp::Time(0)));
  ASSERT_TRUE(sch.scheduleStarted(rclcpp::Time(0.9)));
  ASSERT_TRUE(sch.scheduleStarted(rclcpp::Time(1.0)));
  ASSERT_TRUE(sch.scheduleStarted(rclcpp::Time(1.1)));

  sch.schedule_start_ = rclcpp::Time(1579882740.000);  // EST Mon Jan 24 1970 11:19:00
  sch.schedule_end_ = rclcpp::Time(1579886340.000);    // 1 hr total duration

  ASSERT_FALSE(sch.scheduleStarted(rclcpp::Time(1579882739.000)));
  ASSERT_TRUE(sch.scheduleStarted(rclcpp::Time(1579882740.000)));
  ASSERT_TRUE(sch.scheduleStarted(rclcpp::Time(1579882741.000)));
  ASSERT_TRUE(sch.scheduleStarted(rclcpp::Time(1579886340.000)));
  ASSERT_TRUE(sch.scheduleStarted(rclcpp::Time(1579886341.000)));
}

TEST(GeofenceSchedule, getNextInterval)
{
  // Test before start

  GeofenceSchedule sch(rclcpp::Time(1e9 * 1), rclcpp::Time(1e9 * 6), rclcpp::Duration(1e9 * 2), rclcpp::Duration(1e9 * 1), rclcpp::Duration(1e9 * 0), rclcpp::Duration(1e9 * 1),
                       rclcpp::Duration(1e9 * 2)  // This means the next schedule is a 4 (2+2)
  );

  // Test before control start
  ASSERT_NEAR(2, sch.getNextInterval(rclcpp::Time(1e9 * 0)).second.seconds(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(rclcpp::Time(1e9 * 0)).first);
  // Test after start but before control_start
  ASSERT_NEAR(2, sch.getNextInterval(rclcpp::Time(1e9 * 1.5)).second.seconds(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(rclcpp::Time(1e9 * 1.5)).first);
  // Test between first control_start and control_end
  ASSERT_NEAR(0.0, sch.getNextInterval(rclcpp::Time(1e9 * 2.5)).second.seconds(), 0.00001);
  ASSERT_TRUE(sch.getNextInterval(rclcpp::Time(1e9 * 2.5)).first);
  // Test after control ends
  ASSERT_NEAR(0.0, sch.getNextInterval(rclcpp::Time(1e9 * 3.5)).second.seconds(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(rclcpp::Time(1e9 * 3.5)).first);

  sch = GeofenceSchedule(rclcpp::Time(1e9 * 1), rclcpp::Time(1e9 * 6), rclcpp::Duration(1e9 * 2), rclcpp::Duration(1e9 * 3), rclcpp::Duration(1e9 * 0), rclcpp::Duration(1e9 * 1),
                         rclcpp::Duration(1e9 * 2)  // This means the next schedule is a 4 (2+2)
  );
  // Test between end of first control and start of second
  ASSERT_NEAR(4, sch.getNextInterval(rclcpp::Time(1e9 * 3.5)).second.seconds(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(rclcpp::Time(1e9 * 3.5)).first);
  // Test between 2nd control start and control end
  ASSERT_NEAR(0.0, sch.getNextInterval(rclcpp::Time(1e9 * 4.5)).second.seconds(), 0.00001);
  ASSERT_TRUE(sch.getNextInterval(rclcpp::Time(1e9 * 4.5)).first);
  // Test after control_end
  ASSERT_NEAR(0.0, sch.getNextInterval(rclcpp::Time(1e9 * 5.5)).second.seconds(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(rclcpp::Time(1e9 * 5.5)).first);
  // Test other day of the week
  ASSERT_NEAR(0.0, sch.getNextInterval(rclcpp::Time(1e9 * 90000)).second.seconds(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(rclcpp::Time(1e9 * 90000)).first);
  // Test after schedule end
  ASSERT_NEAR(0.0, sch.getNextInterval(rclcpp::Time(1e9 * 7.0)).second.seconds(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(rclcpp::Time(1e9 * 7.0)).first);
}
}  // namespace carma_wm_ctrl
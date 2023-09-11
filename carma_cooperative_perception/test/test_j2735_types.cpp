// Copyright 2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <carma_cooperative_perception/j2735_types.hpp>
#include <carma_cooperative_perception/units_extensions.hpp>
#include <carma_v2x_msgs/msg/position3_d.hpp>
#include <j2735_v2x_msgs/msg/position3_d.hpp>
#include <optional>

TEST(DDateTime, FromJ2735MsgAllAvailable)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_yr;
  using units::literals::operator""_d;
  using units::literals::operator""_hr;
  using units::literals::operator""_min;
  using units::literals::operator""_s;

  j2735_v2x_msgs::msg::DDateTime msg;
  msg.presence_vector = 0b1111'1111;
  msg.year.year = 1;
  msg.month.month = 2;
  msg.day.day = 3;
  msg.hour.hour = 4;
  msg.minute.minute = 5;
  msg.second.millisecond = 6000;
  msg.offset.offset_minute = 7;

  const auto d_date_time{carma_cooperative_perception::DDateTime::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.

  ASSERT_TRUE(d_date_time.year.has_value());
  EXPECT_EQ(d_date_time.year.value(), 1_yr);

  ASSERT_TRUE(d_date_time.month.has_value());
  EXPECT_EQ(d_date_time.month.value(), carma_cooperative_perception::February);

  ASSERT_TRUE(d_date_time.day.has_value());
  EXPECT_EQ(d_date_time.day.value(), 3_d);

  ASSERT_TRUE(d_date_time.hour.has_value());
  EXPECT_EQ(d_date_time.hour.value(), 4_hr);

  ASSERT_TRUE(d_date_time.minute.has_value());
  EXPECT_EQ(d_date_time.minute.value(), 5_min);

  ASSERT_TRUE(d_date_time.second.has_value());
  EXPECT_EQ(d_date_time.second.value(), 6_s);

  ASSERT_TRUE(d_date_time.time_zone_offset.has_value());
  EXPECT_EQ(d_date_time.time_zone_offset.value(), 7_min);
}

TEST(DDateTime, FromJ2735MsgNoneAvailable)
{
  j2735_v2x_msgs::msg::DDateTime msg;
  msg.presence_vector = 0b0000'0000;

  const auto d_date_time{carma_cooperative_perception::DDateTime::from_msg(msg)};

  ASSERT_FALSE(d_date_time.year.has_value());
  ASSERT_FALSE(d_date_time.month.has_value());
  ASSERT_FALSE(d_date_time.day.has_value());
  ASSERT_FALSE(d_date_time.hour.has_value());
  ASSERT_FALSE(d_date_time.minute.has_value());
  ASSERT_FALSE(d_date_time.second.has_value());
  ASSERT_FALSE(d_date_time.time_zone_offset.has_value());
}

TEST(AccelerationSet4Way, FromJ2735Msg)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_centi_mps_sq;
  using units::literals::operator""_two_centi_SG;
  using units::literals::operator""_centi_deg_per_s;

  j2735_v2x_msgs::msg::AccelerationSet4Way msg;
  msg.longitudinal = 100;  // centimeters per second squared
  msg.lateral = 200;       // centimeters per second squared
  msg.vert = 30;           // 2 centi G
  msg.yaw_rate = 400;      // centi degrees per second

  const auto accel_set{carma_cooperative_perception::AccelerationSet4Way::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(accel_set.longitudinal, 100_centi_mps_sq);
  EXPECT_EQ(accel_set.lateral, 200_centi_mps_sq);
  EXPECT_EQ(accel_set.vert, 30_two_centi_SG);
  EXPECT_EQ(accel_set.yaw_rate, 400_centi_deg_per_s);
}

TEST(AccelerationSet4Way, FromCarmaMsg)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_mps_sq;
  using units::literals::operator""_deg_per_s;

  carma_v2x_msgs::msg::AccelerationSet4Way msg;
  msg.longitudinal = 100;  // meters per second squared
  msg.lateral = 200;       // meters per second squared
  msg.vert = 300;          // meters per second squared
  msg.yaw_rate = 400;      // degrees per second

  const auto accel_set{carma_cooperative_perception::AccelerationSet4Way::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(accel_set.longitudinal, 100_mps_sq);
  EXPECT_EQ(accel_set.lateral, 200_mps_sq);
  EXPECT_EQ(accel_set.vert, 300_mps_sq);
  EXPECT_EQ(accel_set.yaw_rate, 400_deg_per_s);
}

TEST(Position3D, FromJ2735MsgAllAvailable)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deci_udeg;
  using units::literals::operator""_deca_cm;

  j2735_v2x_msgs::msg::Position3D msg;
  msg.latitude = 100;   // deci micro degrees
  msg.longitude = 200;  // deci micro degrees
  msg.elevation_exists = true;
  msg.elevation = 300;  // deca centimeters

  const auto position{carma_cooperative_perception::Position3D::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(position.latitude, 100_deci_udeg);
  EXPECT_EQ(position.longitude, 200_deci_udeg);

  ASSERT_TRUE(position.elevation.has_value());
  EXPECT_EQ(position.elevation.value(), 300_deca_cm);
}

TEST(Position3D, FromJ2735MsgNoElevation)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deci_udeg;

  j2735_v2x_msgs::msg::Position3D msg;
  msg.latitude = 100;
  msg.longitude = 200;
  msg.elevation = msg.ELEVATION_UNAVAILABLE;

  const auto position{carma_cooperative_perception::Position3D::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(position.latitude, 100_deci_udeg);
  EXPECT_EQ(position.longitude, 200_deci_udeg);

  EXPECT_FALSE(position.elevation.has_value());
}

TEST(Position3D, FromCarmaMsgAllAvailable)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deg;
  using units::literals::operator""_m;

  carma_v2x_msgs::msg::Position3D msg;
  msg.latitude = 100;   // degrees
  msg.longitude = 200;  // degrees
  msg.elevation_exists = true;
  msg.elevation = 300;  // meters

  const auto position{carma_cooperative_perception::Position3D::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(position.latitude, 100_deg);
  EXPECT_EQ(position.longitude, 200_deg);

  ASSERT_TRUE(position.elevation.has_value());
  EXPECT_EQ(position.elevation.value(), 300_m);
}

TEST(Position3D, FromCarmaMsgNoElevation)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deg;

  carma_v2x_msgs::msg::Position3D msg;
  msg.latitude = 100;
  msg.longitude = 200;
  msg.elevation = msg.ELEVATION_UNAVAILABLE;

  const auto position{carma_cooperative_perception::Position3D::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(position.latitude, 100_deg);
  EXPECT_EQ(position.longitude, 200_deg);

  EXPECT_FALSE(position.elevation.has_value());
}

TEST(Heading, FromJ2735Msg)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_eighth_ddeg;

  j2735_v2x_msgs::msg::Heading msg;
  msg.heading = 100;  // eighth deci degrees

  const auto heading{carma_cooperative_perception::Heading::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(heading.heading, 100_eighth_ddeg);
}

TEST(Heading, FromCarmaMsg)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deg;

  carma_v2x_msgs::msg::Heading msg;
  msg.heading = 100;  // degrees

  const auto heading{carma_cooperative_perception::Heading::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(heading.heading, 100_deg);
}

TEST(Speed, FromJ2735Msg)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_two_centi_mps;

  j2735_v2x_msgs::msg::Speed msg;
  msg.speed = 100;  // 2 cm per second

  const auto speed{carma_cooperative_perception::Speed::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(speed.speed, 100_two_centi_mps);
}

TEST(Speed, FromCarmaMsg)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_mps;

  carma_v2x_msgs::msg::Speed msg;
  msg.speed = 100;  // meters per second

  const auto speed{carma_cooperative_perception::Speed::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(speed.speed, 100_mps);
}

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

#include <carma_cooperative_perception/j3224_types.hpp>

TEST(PositionOffsetXYZ, FromJ3224MsgAllAvailable)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_dm;

  j3224_v2x_msgs::msg::PositionOffsetXYZ msg;
  msg.offset_x.object_distance = 1;  // decimeters
  msg.offset_y.object_distance = 2;  // decimeters
  msg.presence_vector |= msg.HAS_OFFSET_Z;
  msg.offset_z.object_distance = 3;  // decimeters

  const auto position{carma_cooperative_perception::PositionOffsetXYZ::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(position.offset_x, 1_dm);
  EXPECT_EQ(position.offset_y, 2_dm);

  ASSERT_TRUE(position.offset_z.has_value());
  EXPECT_EQ(position.offset_z, 3_dm);
}

TEST(PositionOffsetXYZ, FromJ3224MsgNoZOffset)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_dm;

  j3224_v2x_msgs::msg::PositionOffsetXYZ msg;
  msg.offset_x.object_distance = 1;  // decimeters
  msg.offset_y.object_distance = 2;  // decimeters
  msg.presence_vector &= ~msg.HAS_OFFSET_Z;

  const auto position{carma_cooperative_perception::PositionOffsetXYZ::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(position.offset_x, 1_dm);
  EXPECT_EQ(position.offset_y, 2_dm);

  EXPECT_FALSE(position.offset_z.has_value());
}

TEST(PositionOffsetXYZ, FromCarmaMsgAllAvailable)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_m;

  carma_v2x_msgs::msg::PositionOffsetXYZ msg;
  msg.offset_x.object_distance = 1;  // meters
  msg.offset_y.object_distance = 2;  // meters
  msg.presence_vector |= msg.HAS_OFFSET_Z;
  msg.offset_z.object_distance = 3;  // meters

  const auto position{carma_cooperative_perception::PositionOffsetXYZ::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(position.offset_x, 1_m);
  EXPECT_EQ(position.offset_y, 2_m);

  ASSERT_TRUE(position.offset_z.has_value());
  EXPECT_EQ(position.offset_z, 3_m);
}

TEST(PositionOffsetXYZ, FromCarmaMsgNoZOffset)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_m;

  carma_v2x_msgs::msg::PositionOffsetXYZ msg;
  msg.offset_x.object_distance = 1;  // meters
  msg.offset_y.object_distance = 2;  // meters
  msg.presence_vector &= ~msg.HAS_OFFSET_Z;

  const auto position{carma_cooperative_perception::PositionOffsetXYZ::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(position.offset_x, 1_m);
  EXPECT_EQ(position.offset_y, 2_m);

  EXPECT_FALSE(position.offset_z.has_value());
}

TEST(MeasurementTimeOffset, FromJ3224Msg)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_ms;

  j3224_v2x_msgs::msg::MeasurementTimeOffset msg;
  msg.measurement_time_offset = 1;  // milliseconds

  const auto offset{carma_cooperative_perception::MeasurementTimeOffset::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(offset.measurement_time_offset, 1_ms);
}

TEST(MeasurementTimeOffset, FromCarmaMsg)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_s;

  carma_v2x_msgs::msg::MeasurementTimeOffset msg;
  msg.measurement_time_offset = 1;  // seconds

  const auto offset{carma_cooperative_perception::MeasurementTimeOffset::from_msg(msg)};

  // Due to nholthaus/units's equality operator implementation,
  // this will check almost-equality.
  EXPECT_EQ(offset.measurement_time_offset, 1_s);
}

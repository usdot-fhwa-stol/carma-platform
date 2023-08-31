/*
 * Copyright 2023 Leidos
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <carma_cooperative_perception/j2735_types.hpp>
#include <carma_cooperative_perception/units_extensions.hpp>
#include <j2735_v2x_msgs/msg/position3_d.hpp>
#include <optional>

TEST(Position3D, FromMsgAllAvailable)
{
  using namespace units::literals;

  j2735_v2x_msgs::msg::Position3D msg;
  msg.longitude = 100;
  msg.latitude = 200;
  msg.elevation = 300;

  const carma_cooperative_perception::Position3D expected_position_3d{
    units::angle::deci_micro_degrees_t{100}, units::angle::deci_micro_degrees_t{200},
    units::length::decimeter_t{300}};

  EXPECT_EQ(expected_position_3d.latitude, 100_deci_udeg);
  EXPECT_EQ(expected_position_3d.longitude, 200_deci_udeg);

  ASSERT_TRUE(expected_position_3d.elevation.has_value());
  EXPECT_EQ(expected_position_3d.elevation.value(), 300_dm);
}

TEST(Position3D, FromMsgNoElevation)
{
  using namespace units::literals;

  j2735_v2x_msgs::msg::Position3D msg;
  msg.longitude = 100;
  msg.latitude = 200;
  msg.elevation = msg.ELEVATION_UNAVAILABLE;

  const carma_cooperative_perception::Position3D expected_position_3d{
    units::angle::deci_micro_degrees_t{100}, units::angle::deci_micro_degrees_t{200}, std::nullopt};

  EXPECT_EQ(expected_position_3d.latitude, 100_deci_udeg);
  EXPECT_EQ(expected_position_3d.longitude, 200_deci_udeg);

  EXPECT_FALSE(expected_position_3d.elevation.has_value());
}

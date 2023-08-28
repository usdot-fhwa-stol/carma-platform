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
    .latitude{units::angle::deci_micro_degrees_t{100}},
    .longitude{units::angle::deci_micro_degrees_t{200}},
    .elevation{units::length::decimeter_t{300}}};

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
    .latitude{units::angle::deci_micro_degrees_t{100}},
    .longitude{units::angle::deci_micro_degrees_t{200}},
    .elevation{std::nullopt}};

  EXPECT_EQ(expected_position_3d.latitude, 100_deci_udeg);
  EXPECT_EQ(expected_position_3d.longitude, 200_deci_udeg);

  EXPECT_FALSE(expected_position_3d.elevation.has_value());
}

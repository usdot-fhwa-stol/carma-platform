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

#include <carma_cooperative_perception/geodetic.hpp>
#include <carma_cooperative_perception/units_extensions.hpp>
#include <vector>

TEST(CalculateUtmZone, Wgs84Coordinate)
{
  struct TestDataPair
  {
    double test_longitude;
    carma_cooperative_perception::UtmZone expected_zone;
  };

  static constexpr std::size_t zone_width{6};

  // Test data will be {(-180, 1), (-179, 1), (-173, 1), (-167, 2), ..., (180, 60)};
  std::vector<TestDataPair> test_data{
    {-180, {1, carma_cooperative_perception::Hemisphere::kNorth}}};
  for (std::size_t i{0U}; i < 60UL; ++i) {
    test_data.push_back(
      {zone_width * i + 1 - 180.0, {i + 1, carma_cooperative_perception::Hemisphere::kNorth}});
  }

  test_data.push_back({180.0, {60, carma_cooperative_perception::Hemisphere::kNorth}});

  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deg;
  using units::literals::operator""_m;

  for (const auto [test_longitude, expected_zone] : test_data) {
    const carma_cooperative_perception::Wgs84Coordinate coord{
      0.0_deg, units::angle::degree_t{test_longitude}, 0.0_m};
    EXPECT_EQ(carma_cooperative_perception::calculate_utm_zone(coord), expected_zone);
  }
}

TEST(ProjectToUtm, BasicCases)
{
  struct TestDataPair
  {
    carma_cooperative_perception::Wgs84Coordinate test_wgs84;
    carma_cooperative_perception::UtmCoordinate expected_utm;
  };

  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deg;
  using units::literals::operator""_m;

  std::vector<TestDataPair> test_data{
    {carma_cooperative_perception::Wgs84Coordinate{61.15880_deg, 10.36924_deg, 25.6_m},
     carma_cooperative_perception::UtmCoordinate{
       {32, carma_cooperative_perception::Hemisphere::kNorth}, 573682.75_m, 6781246.70_m, 25.6_m}},

    {carma_cooperative_perception::Wgs84Coordinate{-7.96383_deg, 97.00547_deg, 45.7_m},
     carma_cooperative_perception::UtmCoordinate{
       {47, carma_cooperative_perception::Hemisphere::kSouth}, 280142.09_m, 9119170.45_m, 45.7_m}},

    {carma_cooperative_perception::Wgs84Coordinate{19.93875_deg, -151.15646_deg, -12.1_m},
     carma_cooperative_perception::UtmCoordinate{
       {5, carma_cooperative_perception::Hemisphere::kNorth}, 692944.13_m, 2205762.20_m, -12.1_m}}};

  auto test_index{0U};
  for (const auto [test_wgs84, expected_utm] : test_data) {
    const auto utm{carma_cooperative_perception::project_to_utm(test_wgs84)};

    EXPECT_EQ(expected_utm.utm_zone, utm.utm_zone) << "Test index: " << test_index;
    EXPECT_NEAR(
      carma_cooperative_perception::remove_units(expected_utm.easting),
      carma_cooperative_perception::remove_units(utm.easting), 1e-2)
      << "Test index: " << test_index;
    EXPECT_NEAR(
      carma_cooperative_perception::remove_units(expected_utm.northing),
      carma_cooperative_perception::remove_units(utm.northing), 1e-2)
      << "Test index: " << test_index;
    EXPECT_NEAR(
      carma_cooperative_perception::remove_units(expected_utm.elevation),
      carma_cooperative_perception::remove_units(utm.elevation), 1e-2)
      << "Test index: " << test_index;

    ++test_index;
  }
}

TEST(CalculateGridConvergence, RightHalf)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deg;
  using units::literals::operator""_m;

  const carma_cooperative_perception::UtmZone utm_zone{
    32, carma_cooperative_perception::Hemisphere::kNorth};

  // UTM Zone 32: 510500 easting, 7043500 northing
  const carma_cooperative_perception::Wgs84Coordinate position_wgs84{
    63.510617_deg, 9.210989_deg, 25.3_m};

  const auto result =
    carma_cooperative_perception::calculate_grid_convergence(position_wgs84, utm_zone);

  EXPECT_NEAR(carma_cooperative_perception::remove_units(result), 0.188839, 1e-6);
}

TEST(CalculateGridConvergence, LeftHalf)
{
  // Note: Google C++ style guide prohibits namespace using-directives
  using units::literals::operator""_deg;
  using units::literals::operator""_m;

  const carma_cooperative_perception::UtmZone utm_zone{
    32, carma_cooperative_perception::Hemisphere::kNorth};

  // UTM Zone 32: 480500 easting, 7043500 northing
  const carma_cooperative_perception::Wgs84Coordinate position_wgs84{
    63.510617_deg, 8.608168_deg, 25.3_m};

  const auto result =
    carma_cooperative_perception::calculate_grid_convergence(position_wgs84, utm_zone);

  EXPECT_NEAR(carma_cooperative_perception::remove_units(result), -0.350697, 1e-6);
}

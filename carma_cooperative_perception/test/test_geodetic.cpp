#include <gtest/gtest.h>

#include <carma_cooperative_perception/geodetic.hpp>

TEST(CalculateUtmZone, Wgs84Coordinate)
{
  struct TestDataPair
  {
    double test_longitude;
    carma_cooperative_perception::UtmZone expected_zone;
  };

  static constexpr std::size_t zone_width{6};

  // Test data will be {(-180, 1), (-179, 1), (-173, 1), (-167, 2), ..., (180, 60)};
  std::vector<TestDataPair> test_data{{-180, 1}};
  for (std::size_t i{0U}; i < 60UL; ++i) {
    test_data.push_back(
      {zone_width * i + 1 - 180.0, carma_cooperative_perception::UtmZone{
                                     i + 1, carma_cooperative_perception::Hemisphere::kNorth}});
  }

  test_data.push_back({180.0, 60});

  using namespace units::literals;
  for (const auto [test_longitude, expected_zone] : test_data) {
    const carma_cooperative_perception::Wgs84Coordinate coord{
      .latitude = 0.0_deg, .longitude = units::angle::degree_t{test_longitude}, .elevation = 0.0_m};
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

  using namespace units::literals;
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
      units::unit_cast<double>(expected_utm.easting), units::unit_cast<double>(utm.easting), 1e-2)
      << "Test index: " << test_index;
    EXPECT_NEAR(
      units::unit_cast<double>(expected_utm.northing), units::unit_cast<double>(utm.northing), 1e-2)
      << "Test index: " << test_index;
    EXPECT_NEAR(
      units::unit_cast<double>(expected_utm.elevation), units::unit_cast<double>(utm.elevation),
      1e-2)
      << "Test index: " << test_index;

    ++test_index;
  }
}

TEST(CalculateGridConvergence, RightHalf)
{
  using namespace units::literals;

  const carma_cooperative_perception::UtmZone utm_zone{
    .number = 32, .hemisphere = carma_cooperative_perception::Hemisphere::kNorth};

  // UTM Zone 32: 510500 easting, 7043500 northing
  const carma_cooperative_perception::Wgs84Coordinate position_wgs84{
    .latitude = 63.510617_deg, .longitude = 9.210989_deg, .elevation = 25.3_m};

  const auto result =
    carma_cooperative_perception::calculate_grid_convergence(position_wgs84, utm_zone);

  EXPECT_NEAR(units::unit_cast<double>(result), 0.188839, 1e-6);
}

TEST(CalculateGridConvergence, LeftHalf)
{
  using namespace units::literals;

  const carma_cooperative_perception::UtmZone utm_zone{
    .number = 32, .hemisphere = carma_cooperative_perception::Hemisphere::kNorth};

  // UTM Zone 32: 480500 easting, 7043500 northing
  const carma_cooperative_perception::Wgs84Coordinate position_wgs84{
    .latitude = 63.510617_deg, .longitude = 8.608168_deg, .elevation = 25.3_m};

  const auto result =
    carma_cooperative_perception::calculate_grid_convergence(position_wgs84, utm_zone);

  EXPECT_NEAR(units::unit_cast<double>(result), -0.350697, 1e-6);
}

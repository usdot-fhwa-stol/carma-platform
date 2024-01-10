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

#include "carma_cooperative_perception/geodetic.hpp"

#include <proj.h>
#include <gsl/pointers>

#include <algorithm>
#include <string>

#include "carma_cooperative_perception/units_extensions.hpp"

namespace carma_cooperative_perception
{
auto calculate_utm_zone(const Wgs84Coordinate & coordinate) -> UtmZone
{
  // Note: std::floor prevents this function from being constexpr (until C++23)

  static constexpr std::size_t zone_width{6};
  static constexpr std::size_t max_zones{60};

  // Works for longitudes [-180, 360). Longitude of 360 will assign 61.
  const auto number{
    static_cast<std::size_t>(
      (std::floor(carma_cooperative_perception::remove_units(coordinate.longitude) + 180) /
       zone_width)) +
    1};

  UtmZone zone;

  // std::min is used to handle the "UTM Zone 61" case.
  zone.number = std::min(number, max_zones);

  if (coordinate.latitude < units::angle::degree_t{0.0}) {
    zone.hemisphere = Hemisphere::kSouth;
  } else {
    zone.hemisphere = Hemisphere::kNorth;
  }

  return zone;
}

auto project_to_utm(const Wgs84Coordinate & coordinate) -> UtmCoordinate
{
  gsl::owner<PJ_CONTEXT *> context = proj_context_create();
  proj_log_level(context, PJ_LOG_NONE);

  if (context == nullptr) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not create PROJ context: " + error_string + '.');
  }

  const auto utm_zone{calculate_utm_zone(coordinate)};
  std::string proj_string{"+proj=utm +zone=" + std::to_string(utm_zone.number) + " +datum=WGS84"};

  if (utm_zone.hemisphere == Hemisphere::kSouth) {
    proj_string += " +south";
  }

  gsl ::owner<PJ *> utm_transformation =
    proj_create_crs_to_crs(context, "EPSG:4326", proj_string.c_str(), nullptr);

  if (utm_transformation == nullptr) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not create PROJ transform: " + error_string + '.');
  }

  auto coord_wgs84 = proj_coord(
    carma_cooperative_perception::remove_units(coordinate.latitude),
    carma_cooperative_perception::remove_units(coordinate.longitude), 0, 0);
  auto coord_utm = proj_trans(utm_transformation, PJ_FWD, coord_wgs84);

  proj_destroy(utm_transformation);
  proj_context_destroy(context);

  return {
    utm_zone, units::length::meter_t{coord_utm.enu.e}, units::length::meter_t{coord_utm.enu.n},
    units::length::meter_t{coordinate.elevation}};
}

auto calculate_grid_convergence(const Wgs84Coordinate & position, const UtmZone & zone)
  -> units::angle::degree_t
{
  gsl::owner<PJ_CONTEXT *> context = proj_context_create();
  proj_log_level(context, PJ_LOG_NONE);

  if (context == nullptr) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not create PROJ context: " + error_string + '.');
  }

  // N.B. developers: PROJ and the related geodetic calculations seem particularly sensitive
  // to the parameters in this PROJ string. If you run into problems with you calculation
  // results, carefully check this or any other PROJ string.
  std::string proj_string{
    "+proj=utm +zone=" + std::to_string(zone.number) + " +datum=WGS84 +units=m +no_defs"};
  if (zone.hemisphere == Hemisphere::kSouth) {
    proj_string += " +south";
  }

  gsl::owner<PJ *> transform = proj_create(context, proj_string.c_str());

  const auto factors = proj_factors(
    transform, proj_coord(
                 proj_torad(carma_cooperative_perception::remove_units(position.longitude)),
                 proj_torad(carma_cooperative_perception::remove_units(position.latitude)), 0, 0));

  if (proj_context_errno(context) != 0) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not calculate PROJ factors: " + error_string + '.');
  }

  proj_destroy(transform);
  proj_context_destroy(context);

  return units::angle::degree_t{proj_todeg(factors.meridian_convergence)};
}

}  // namespace carma_cooperative_perception

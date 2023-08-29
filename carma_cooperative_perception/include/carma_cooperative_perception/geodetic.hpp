#ifndef CARMA_COOPERATIVE_PERCEPTION_GEODETIC_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_GEODETIC_HPP_

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

/**
 * This file contains functions and helper structs to facilitate transforming
 * WGS-84 coordinates to UTM ones.
*/

#include <proj.h>
#include <units.h>

#include <gsl/pointers>

#include "carma_cooperative_perception/utm_zone.hpp"

namespace carma_cooperative_perception
{

/**
 * @brief Represents a position using WGS-84 coordinates
*/
struct Wgs84Coordinate
{
  units::angle::degree_t latitude;  /** Decimal degrees [-180, 180]. */
  units::angle::degree_t longitude; /** Decimal degrees [-90, 90]. */
  units::length::meter_t elevation; /** With respect to the reference ellipsoid. */
};

/**
 * @brief Represents a position using UTM coordinates
*/
struct UtmCoordinate
{
  UtmZone utm_zone;
  units::length::meter_t easting;
  units::length::meter_t northing;
  units::length::meter_t elevation; /** With respect to the reference ellipsoid. */
};

/**
 * @brief Represent a displacement from a UTM coordinate
*/
struct UtmDisplacement
{
  units::length::meter_t easting;
  units::length::meter_t northing;
  units::length::meter_t elevation;
};

/**
 * @brief Addition-assignment operator overload
 *
 * @param[in] coordinate Position represented in UTM coordinates
 * @param[in] displacement Displacement from coordinate
 *
 * @return Reference to the coordinate's updated position
*/
inline constexpr auto operator+=(
  UtmCoordinate & coordinate, const UtmDisplacement & displacement) noexcept -> UtmCoordinate &
{
  coordinate.easting += displacement.easting;
  coordinate.northing += displacement.northing;
  coordinate.elevation += displacement.elevation;

  return coordinate;
}

/**
 * @brief Addition operator overload
 *
 * @param[in] coordinate Position represented in UTM coordinates
 * @param[in] displacement Displacement form coordinate
 *
 * @return A new UtmCoordinate representing the new position
*/
inline constexpr auto operator+(
  UtmCoordinate coordinate, const UtmDisplacement & displacement) noexcept -> UtmCoordinate
{
  return coordinate += displacement;
}

/**
 * @brief Addition operator overload
 *
 * @param[in] displacement Displacement from coordinate
 * @param[in] coordinate Position represented in UTM coordinates
 *
 * @return A new UtmCoordinate representing the new position
*/
inline constexpr auto operator+(
  const UtmDisplacement & displacement, UtmCoordinate coordinate) noexcept -> UtmCoordinate
{
  return coordinate += displacement;
}

/**
 * @brief Subtraction-assignment operator overload
 *
 * @param[in] coordinate Position represented in UTM coordinates
 * @param[in] displacement Displacement from coordinate
 *
 * @return Reference to the coordinate's updated position
*/
inline constexpr auto operator-=(
  UtmCoordinate & coordinate, const UtmDisplacement & displacement) noexcept -> UtmCoordinate &
{
  coordinate.easting += displacement.easting;
  coordinate.northing += displacement.northing;
  coordinate.elevation += displacement.elevation;

  return coordinate;
}

/**
 * @brief Subtraction operator overload
 *
 * @param[in] coordinate Position represented in UTM coordinates
 * @param[in] displacement Displacement form coordinate
 *
 * @return A new UtmCoordinate representing the new position
*/
inline constexpr auto operator-(
  UtmCoordinate coordinate, const UtmDisplacement & displacement) noexcept -> UtmCoordinate
{
  return coordinate -= displacement;
}

/**
 * @brief Subtraction operator overload
 *
 * @param[in] displacement Displacement from coordinate
 * @param[in] coordinate Position represented in UTM coordinates
 *
 * @return A new UtmCoordinate representing the new position
*/
inline constexpr auto operator-(
  const UtmDisplacement & displacement, UtmCoordinate coordinate) noexcept -> UtmCoordinate
{
  return coordinate -= displacement;
}

/**
 * @brief Get the UTM zone number from a WGS-84 coordinate
 *
 * Note: This function will not work for coordinates in the special UTM zones
 * Svalbard and Norway.
 *
 * @param[in] coordinate WGS-84 coordinate
 *
 * @return The UTM zone containing the coordinate
*/
inline auto calculate_utm_zone(const Wgs84Coordinate & coordinate) -> UtmZone
{
  // Note: std::floor prevents this function from being constexpr (until C++23)

  static constexpr std::size_t zone_width{6};
  static constexpr std::size_t max_zones{60};

  // Works for longitudes [-180, 360). Longitude of 360 will assign 61.
  const auto number{
    static_cast<std::size_t>(
      (std::floor(units::unit_cast<double>(coordinate.longitude) + 180) / 6)) +
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

/**
 * @brief Projects a Wgs84Coordinate to its corresponding UTM zone
 *
 * @param[in] coordinate Position represented in WGS-84 coordinates
 *
 * @return Coordinate's position represented in UTM coordinates
*/
inline auto project_to_utm(const Wgs84Coordinate & coordinate) -> UtmCoordinate
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
    units::unit_cast<double>(coordinate.latitude), units::unit_cast<double>(coordinate.longitude),
    0, 0);
  auto coord_utm = proj_trans(utm_transformation, PJ_FWD, coord_wgs84);

  proj_destroy(utm_transformation);
  proj_context_destroy(context);

  return {
    .utm_zone = utm_zone,
    .easting = units::length::meter_t{coord_utm.enu.e},
    .northing = units::length::meter_t{coord_utm.enu.n},
    .elevation = units::length::meter_t{coordinate.elevation}};
}

/**
 * @brief Calculate grid convergence at a given position
 *
 * This function calculates the grid convergence at a specific coordinate with respect to
 * a specified UTM zone. Grid convergence is the angle between true north and grid north.
 *
 * @param[in] position Position represented in WGS-84 coordinates
 * @param[in] zone The UTM zone
 *
 * @return Grid convergence angle
*/
inline auto calculate_grid_convergence(const Wgs84Coordinate & position, const UtmZone & zone)
  -> units::angle::degree_t
{
  gsl::owner<PJ_CONTEXT *> context = proj_context_create();
  proj_log_level(context, PJ_LOG_NONE);

  if (context == nullptr) {
    const std::string error_string{proj_errno_string(proj_context_errno(context))};
    throw std::invalid_argument("Could not create PROJ context: " + error_string + '.');
  }

  std::string proj_string{
    "+proj=utm +zone=" + std::to_string(zone.number) + " +datum=WGS84 +units=m +no_defs +type=crs"};
  if (zone.hemisphere == Hemisphere::kSouth) {
    proj_string += " +south";
  }

  gsl::owner<PJ *> transform = proj_create(context, proj_string.c_str());

  const auto factors = proj_factors(
    transform, proj_coord(
                 proj_torad(units::unit_cast<double>(position.longitude)),
                 proj_torad(units::unit_cast<double>(position.latitude)), 0, 0));

  proj_destroy(transform);
  proj_context_destroy(context);

  return units::angle::degree_t{proj_todeg(factors.meridian_convergence)};
}

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION_GEODETIC_HPP_

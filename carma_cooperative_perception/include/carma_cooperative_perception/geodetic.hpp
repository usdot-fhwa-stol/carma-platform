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

#ifndef CARMA_COOPERATIVE_PERCEPTION__GEODETIC_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__GEODETIC_HPP_

/**
 * This file contains functions and helper structs to facilitate transforming
 * WGS-84 coordinates to UTM ones.
*/

#include <units.h>

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
auto calculate_utm_zone(const Wgs84Coordinate & coordinate) -> UtmZone;

/**
 * @brief Projects a Wgs84Coordinate to its corresponding UTM zone
 *
 * @param[in] coordinate Position represented in WGS-84 coordinates
 *
 * @return Coordinate's position represented in UTM coordinates
*/
auto project_to_utm(const Wgs84Coordinate & coordinate) -> UtmCoordinate;

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
auto calculate_grid_convergence(const Wgs84Coordinate & position, const UtmZone & zone)
  -> units::angle::degree_t;

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__GEODETIC_HPP_

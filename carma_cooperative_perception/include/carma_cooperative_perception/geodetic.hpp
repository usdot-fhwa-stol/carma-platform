#ifndef CARMA_COOPERATIVE_PERCEPTION_GEODETIC_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_GEODETIC_HPP_

#include <proj.h>
#include <units.h>

#include <gsl/pointers>

#include "carma_cooperative_perception/utm_zone.hpp"

namespace carma_cooperative_perception
{

struct Wgs84Coordinate
{
  units::angle::degree_t latitude;  /** Decimal degrees [-180, 180]. */
  units::angle::degree_t longitude; /** Decimal degrees [-90, 90]. */
  units::length::meter_t elevation; /** With respect to the reference ellipsoid. */
};

struct UtmCoordinate
{
  UtmZone utm_zone;
  units::length::meter_t easting;
  units::length::meter_t northing;
  units::length::meter_t elevation; /** With respect to the reference ellipsoid. */
};

struct UtmDisplacement
{
  units::length::meter_t easting;
  units::length::meter_t northing;
  units::length::meter_t elevation;
};

inline constexpr auto operator+=(
  UtmCoordinate & coordinate, const UtmDisplacement & displacement) noexcept -> UtmCoordinate &
{
  coordinate.easting += displacement.easting;
  coordinate.northing += displacement.northing;
  coordinate.elevation += displacement.elevation;

  return coordinate;
}

inline constexpr auto operator+(
  UtmCoordinate coordinate, const UtmDisplacement & displacement) noexcept -> UtmCoordinate
{
  return coordinate += displacement;
}

inline constexpr auto operator+(
  const UtmDisplacement & displacement, UtmCoordinate coordinate) noexcept -> UtmCoordinate
{
  return coordinate += displacement;
}

inline constexpr auto operator-=(
  UtmCoordinate & coordinate, const UtmDisplacement & displacement) noexcept -> UtmCoordinate &
{
  coordinate.easting += displacement.easting;
  coordinate.northing += displacement.northing;
  coordinate.elevation += displacement.elevation;

  return coordinate;
}

inline constexpr auto operator-(
  UtmCoordinate coordinate, const UtmDisplacement & displacement) noexcept -> UtmCoordinate
{
  return coordinate -= displacement;
}

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
 * @param coordinate WGS-84 coordinate
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

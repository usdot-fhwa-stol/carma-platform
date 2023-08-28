#ifndef CARMA_COOPERATIVE_PERCEPTION_UNITS_EXTENSIONS_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_UNITS_EXTENSIONS_HPP_

#include <units.h>

#include <ratio>

namespace carma_cooperative_perception
{

template <typename T>
inline constexpr auto remove_units(const T & value) noexcept
{
  return units::unit_cast<double>(value);
}

}  // namespace carma_cooperative_perception

namespace units
{

UNIT_ADD(
  acceleration, centi_meters_per_second_squared, centimeters_per_second_squared, centi_mps_sq,
  unit<std::centi, meters_per_second_squared>)

UNIT_ADD(
  acceleration, two_centi_standard_gravities, two_centi_standard_gravities, two_centi_SG,
  unit<std::ratio_multiply<std::ratio<2>, std::centi>, standard_gravity>)

UNIT_ADD(
  angular_velocity, centi_degrees_per_second, centi_degrees_per_second, centi_deg_per_s,
  unit<std::centi, degrees_per_second>)

UNIT_ADD(
  angle, deci_micro_degrees, deci_micro_degrees, deci_udeg,
  unit<std::ratio_multiply<std::deci, std::micro>, degrees>)

UNIT_ADD(length, deca_centimeters, deca_centimeters, deca_cm, unit<std::ratio<10>, centimeters>)

UNIT_ADD(
  angle, eighth_deci_degrees, eighth_deci_degrees, eighth_ddeg,
  unit<std::ratio_multiply<std::ratio<1, 8>, std::deci>, degrees>)

UNIT_ADD(
  velocity, two_milli_meters_per_second, two_milli_meters_per_second, two_milli_mps,
  unit<std::ratio_multiply<std::ratio<2>, std::milli>, meters_per_second>)

UNIT_ADD(
  velocity, two_centi_meters_per_second, two_centi_meters_per_second, two_centi_mps,
  unit<std::ratio_multiply<std::ratio<2>, std::centi>, meters_per_second>)

}  // namespace units

#endif  // CARMA_COOPERATIVE_PERCEPTION_UNITS_EXTENSIONS_HPP_

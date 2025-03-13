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

#ifndef CARMA_COOPERATIVE_PERCEPTION__UNITS_EXTENSIONS_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__UNITS_EXTENSIONS_HPP_

#include <units.h>

#include <ratio>

namespace carma_cooperative_perception
{
template <typename T>
constexpr auto remove_units(const T & value)
{
  return units::unit_cast<typename T::underlying_type>(value);
}

}  // namespace carma_cooperative_perception

/*
 * The nholthaus/units library does not include an exhaustive list of units, so
 * this is how we can add missing/new ones. See the following for more information:
 * - https://github.com/nholthaus/units?tab=readme-ov-file#defining-new-units
 * - https://github.com/nholthaus/units?tab=readme-ov-file#unit-definition-macros
 */
namespace units
{

// NOLINTBEGIN
UNIT_ADD(
  acceleration, centi_meters_per_second_squared, centimeters_per_second_squared,
  centi_mps_sq,
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

UNIT_ADD(
  length, deca_centimeters, deca_centimeters, deca_cm, unit<std::ratio<10>, centimeters>)

UNIT_ADD(
  angle, eighth_deci_degrees, eighth_deci_degrees, eighth_ddeg,
  unit<std::ratio_multiply<std::ratio<1, 8>, std::deci>, degrees>)

UNIT_ADD(
  velocity, two_milli_meters_per_second, two_milli_meters_per_second, two_milli_mps,
  unit<std::ratio_multiply<std::ratio<2>, std::milli>, meters_per_second>)

UNIT_ADD(
  velocity, two_centi_meters_per_second, two_centi_meters_per_second, two_centi_mps,
  unit<std::ratio_multiply<std::ratio<2>, std::centi>, meters_per_second>)
// NOLINTEND

}  // namespace units

#endif  // CARMA_COOPERATIVE_PERCEPTION__UNITS_EXTENSIONS_HPP_

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
constexpr auto remove_units(const T & value) noexcept
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
// These are not our macros, so we should not worry about linting them.
// clang-tidy added support for ignoring system macros in release 14.0.0 (see the release notes
// here: https://releases.llvm.org/14.0.0/tools/clang/tools/extra/docs/ReleaseNotes.html), but
// ament_clang_tidy for ROS 2 Foxy specifically looks for clang-tidy-6.0. Note also that
// clang-tidy release 14.0.0 adds NOLINTBEGIN...NOLINTEND, so we can remove the individual NOLINT
// calls in the future.

UNIT_ADD(                                                                         // NOLINT
  acceleration, centi_meters_per_second_squared, centimeters_per_second_squared,  // NOLINT
  centi_mps_sq,                                                                   // NOLINT
  unit<std::centi, meters_per_second_squared>)                                    // NOLINT

UNIT_ADD(                                                                                  // NOLINT
  acceleration, two_centi_standard_gravities, two_centi_standard_gravities, two_centi_SG,  // NOLINT
  unit<std::ratio_multiply<std::ratio<2>, std::centi>, standard_gravity>)                  // NOLINT

UNIT_ADD(                                                                                 // NOLINT
  angular_velocity, centi_degrees_per_second, centi_degrees_per_second, centi_deg_per_s,  // NOLINT
  unit<std::centi, degrees_per_second>)                                                   // NOLINT

UNIT_ADD(                                                     // NOLINT
  angle, deci_micro_degrees, deci_micro_degrees, deci_udeg,   // NOLINT
  unit<std::ratio_multiply<std::deci, std::micro>, degrees>)  // NOLINT

UNIT_ADD(                                                                                  // NOLINT
  length, deca_centimeters, deca_centimeters, deca_cm, unit<std::ratio<10>, centimeters>)  // NOLINT

UNIT_ADD(                                                           // NOLINT
  angle, eighth_deci_degrees, eighth_deci_degrees, eighth_ddeg,     // NOLINT
  unit<std::ratio_multiply<std::ratio<1, 8>, std::deci>, degrees>)  // NOLINT

UNIT_ADD(                                                                             // NOLINT
  velocity, two_milli_meters_per_second, two_milli_meters_per_second, two_milli_mps,  // NOLINT
  unit<std::ratio_multiply<std::ratio<2>, std::milli>, meters_per_second>)            // NOLINT

UNIT_ADD(                                                                             // NOLINT
  velocity, two_centi_meters_per_second, two_centi_meters_per_second, two_centi_mps,  // NOLINT
  unit<std::ratio_multiply<std::ratio<2>, std::centi>, meters_per_second>)            // NOLINT

}  // namespace units

#endif  // CARMA_COOPERATIVE_PERCEPTION__UNITS_EXTENSIONS_HPP_

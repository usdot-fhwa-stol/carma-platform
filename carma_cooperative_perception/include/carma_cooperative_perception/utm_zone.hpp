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

#ifndef CARMA_COOPERATIVE_PERCEPTION__UTM_ZONE_HPP_
#define CARMA_COOPERATIVE_PERCEPTION__UTM_ZONE_HPP_

#include <cstdint>
#include <string>

namespace carma_cooperative_perception
{
enum class Hemisphere
{
  kNorth,
  kSouth
};

struct UtmZone
{
  std::size_t number;
  Hemisphere hemisphere;
};

constexpr inline auto operator==(const UtmZone & lhs, const UtmZone & rhs) -> bool
{
  return lhs.number == rhs.number && lhs.hemisphere == rhs.hemisphere;
}

constexpr inline auto operator!=(const UtmZone & lhs, const UtmZone & rhs) -> bool
{
  return !(lhs == rhs);
}

auto to_string(const UtmZone & zone) -> std::string;

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION__UTM_ZONE_HPP_

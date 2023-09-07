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

#include "carma_cooperative_perception/utm_zone.hpp"

#include <string>

namespace carma_cooperative_perception
{
auto to_string(const UtmZone & zone) -> std::string
{
  if (zone.hemisphere == Hemisphere::kNorth) {
    return std::to_string(zone.number) + "N";
  }

  return std::to_string(zone.number) + "S";
}

}  // namespace carma_cooperative_perception

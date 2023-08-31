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

} // namespace carma_cooperative_perception

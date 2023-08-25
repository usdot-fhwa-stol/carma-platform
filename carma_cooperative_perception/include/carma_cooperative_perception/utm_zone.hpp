#ifndef CARMA_COOPERATIVE_PERCEPTION_UTM_ZONE_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_UTM_ZONE_HPP_

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

auto to_string(const UtmZone& zone) -> std::string {
  if (zone.hemisphere == Hemisphere::kNorth) {
    return std::to_string(zone.number) + "N";
  }

  return std::to_string(zone.number) + "S";
}

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION_UTM_ZONE_HPP_
